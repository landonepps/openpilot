#!/usr/bin/env python3
"""Bounded, receive-only startup recorder. Arm before ignition; never sends CAN.

Uses separate subscriptions and writes shadow radar Events to disk only.
"""
import argparse
import fcntl
import hashlib
import json
import os
import resource
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.bosch_c_radar import BoschCRadarInterface, CandidateCalibration
from opendbc.car.honda.hondacan import CanBus
from openpilot.tools.car_porting.bosch_c_radar_capture import CALIBRATION_PATH, MIN_FREE_BYTES
from openpilot.tools.car_porting.bosch_c_radar_logger import diagnostic_record, write_record
from openpilot.tools.car_porting.bosch_c_radar_time import LOG_CLOCK_NAME, log_time_ns


class Shadow:
  def __init__(self, cp, messaging):
    if cp.brand != 'honda' or cp.carFingerprint != 'HONDA_CRV_6G':
      raise ValueError('Shadow requires HONDA_CRV_6G')
    self.messaging = messaging
    self.adapter = BoschCRadarInterface(cp, structs.CarParamsSP(), bus=CanBus(cp).radar,
      calibration=CandidateCalibration(**json.loads(CALIBRATION_PATH.read_text())['parameters']), clock=log_time_ns)
    # Only update_msgs is used. No socket is read here, no PubMaster is created.
    self.health = messaging.SubMaster(['modelV2', 'radarTracks'], poll='modelV2')
    self.health.simulation = False
    self.latest = None
    self.last_receive = time.monotonic()
    self.late_packets = 0

  def timeout(self):
    # Match a CAN receive timeout. A regular health tick must not advance the
    # decoder past packets still in transit while CAN is flowing.
    return self.update([]) if time.monotonic() - self.last_receive >= .1 else None

  def update(self, packets):
    if packets:
      self.last_receive = time.monotonic()
      watermark = self.adapter.decoder.now_ns
      self.late_packets += sum(watermark is not None and stamp < watermark for stamp, _ in packets)
    record = diagnostic_record(self.adapter, packets)
    if record is not None and record['radar_data'] is not None:
      rr = record['radar_data']
      event = self.messaging.new_message('radarTracks')
      event.valid = not any(rr['errors'].values())
      event.radarTracks = rr
      # card timestamps publication at send time, not at CAN measurement time.
      event.logMonoTime = log_time_ns()
      encoded = event.to_bytes()
      from openpilot.cereal import log
      with log.Event.from_bytes(encoded) as parsed:
        self.latest = parsed.as_builder()
      return {'kind': 'shadow', 'timestamp_ns': event.logMonoTime, 'event_hex': encoded.hex(),
              'diagnostics': record['diagnostics'], 'late_packets': record['late_packets_not_decoded']}
    return None

  def tick(self):
    self.health.update_msgs(time.monotonic(), [] if self.latest is None else [self.latest])
    self.latest = None
    return {'kind': 'health', 'timestamp_ns': log_time_ns(), 'monotonic_ns': time.monotonic_ns(),
            'valid': self.health.valid['radarTracks'], 'alive': self.health.alive['radarTracks'],
            'frequency_ok': self.health.freq_ok['radarTracks'],
            'bank_ns': self.adapter.decoder.last_bank_ns,
            'banks': self.adapter.decoder.counters['accepted_banks'], 'late_packets_total': self.late_packets,
            'points': len(self.health['radarTracks'].points),
            'stale_points': bool(self.health['radarTracks'].points and
              (self.adapter.decoder.last_bank_ns is None or log_time_ns()-self.adapter.decoder.last_bank_ns > 200_000_000))}


def provenance():
  root = Path(__file__).resolve().parents[3]
  result = {'runtime': {'python': sys.version, 'uname': list(platform.uname())}}
  for name, directory in [('sunnypilot', root), ('opendbc', root / 'opendbc_repo')]:
    def git(*args, directory=directory):
      return subprocess.check_output(['git', '-C', str(directory), *args], text=True, timeout=5).strip()
    result[name] = {'revision': git('rev-parse', 'HEAD'), 'branch': git('branch', '--show-current'),
                    'status': git('status', '--porcelain'), 'diff_sha256': hashlib.sha256(git('diff', 'HEAD').encode()).hexdigest()}
  files = [Path(__file__), Path(__file__).with_name('bosch_c_radar_logger.py'), CALIBRATION_PATH,
           root / 'opendbc_repo/opendbc/car/honda/bosch_c_radar.py',
           root / 'opendbc_repo/opendbc/car/honda/carcontroller.py', root / 'openpilot/cereal/messaging/__init__.py']
  files.extend(Path(__file__).with_name(name) for name in
               ('bosch_c_radar_capture.py', 'bosch_c_radar_time.py', 'bosch_c_coexistence_analyze.py', 'bosch_c_coexistence_boot.py'))
  files.extend([root / 'opendbc_repo/opendbc/car/honda/hondacan.py', root / 'launch_chffrplus.sh'])
  result['files'] = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
  return result


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--output', type=Path, required=True)
  ap.add_argument('--ready-file', type=Path, help='Optional startup handshake after subscriptions and header are ready')
  ap.add_argument('--duration', type=float, default=180)
  ap.add_argument('--max-mib', type=int, default=128)
  args = ap.parse_args()
  if not 1 <= args.duration <= 600 or not 1 <= args.max_mib <= 256:
    ap.error('Duration must be 1..600 seconds; max-mib must be 1..256')
  if hasattr(os, 'sched_setscheduler'):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.nice(10)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  if shutil.disk_usage(args.output.parent).free < MIN_FREE_BYTES:
    raise RuntimeError('At least 512 MiB free storage required')
  stopped = False

  def stop(*_):
    nonlocal stopped
    stopped = True

  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  from openpilot.cereal import messaging
  # Subscribe before reading configuration. No CarParams wait loses startup CAN.
  services = ('can', 'sendcan', 'carParams', 'carParamsSP', 'carState', 'deviceState', 'pandaStates',
              'radarTracks', 'radarState')
  sockets = {s: messaging.sub_sock(s, conflate=False) for s in services}
  status_path = args.output.parent / 'coexistence.status.json'
  started = time.monotonic()
  shadow = None
  written = 0
  limit = args.max_mib * 1024 * 1024
  reason = 'duration'
  last_tick = last_status = 0.
  shadow_error = None
  configuration = None
  health = {}
  with (args.output.parent / '.capture.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    with args.output.open('xb') as output:
      def put(record):
        nonlocal written
        count = write_record(output, record, written, limit - 4096)
        if count is None:
          return False
        written = count
        return True

      put({'kind': 'session', 'schema_version': 1, 'clock': LOG_CLOCK_NAME, 'boot_ns': log_time_ns(),
           'monotonic_ns': time.monotonic_ns(), 'utc_ns': time.time_ns(), 'software': provenance(),
           'duration_s': args.duration, 'max_bytes': limit})
      output.flush()
      if args.ready_file is not None:
        args.ready_file.write_text(json.dumps({'pid': os.getpid(), 'boot_ns': log_time_ns(), 'output': str(args.output)}))
      try:
        while not stopped and time.monotonic() - started < args.duration:
          for service, sock in sockets.items():
            # Bounded work per service, even when CAN is continuously arriving.
            for _ in range(128):
              e = messaging.recv_one_or_none(sock)
              if e is None:
                break
              record = {'kind': 'event', 'service': service, 'timestamp_ns': e.logMonoTime,
                        'received_boot_ns': log_time_ns(), 'received_monotonic_ns': time.monotonic_ns(), 'valid': e.valid}
              if service in ('can', 'sendcan'):
                frames = getattr(e, service)
                record['frames'] = [{'address': c.address, 'src': c.src, 'payload': bytes(c.dat).hex()} for c in frames]
              else:
                record['event_hex'] = e.as_builder().to_bytes().hex()
              if not put(record):
                reason, stopped = 'size_limit', True
                break
              if service == 'carParams':
                current = e.carParams.to_dict()
                if configuration is not None and current != configuration:
                  shadow_error, shadow = 'CarParams changed during capture', None
                configuration = current
              if service == 'carParams' and shadow is None and shadow_error is None:
                try:
                  shadow = Shadow(e.carParams.as_builder(), messaging)
                except Exception as exc:
                  shadow_error = str(exc)
              if service == 'can' and shadow is not None:
                try:
                  update = shadow.update([(e.logMonoTime, [CanData(c.address, bytes(c.dat), c.src) for c in e.can])])
                  if update and not put(update):
                    reason, stopped = 'size_limit', True
                except Exception as exc:
                  shadow_error, shadow = str(exc), None
            if stopped:
              break
          now = time.monotonic()
          if shadow is not None and now - last_tick >= .05:
            update = shadow.timeout()  # boot-clock expiry only on a real CAN receive timeout
            health = shadow.tick()
            if (update and not put(update)) or not put(health):
              reason, stopped = 'size_limit', True
            last_tick = now
          if now - last_status >= .5:
            output.flush()
            status = {'recording': True, 'output': str(args.output), 'reported_monotonic_s': now,
                      'object_stream': bool(health.get('valid') and health.get('alive')),
                      'bytes': written, 'shadow_error': shadow_error}
            temporary = status_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(status))
            temporary.replace(status_path)
            last_status = now
          time.sleep(.005)
      except Exception:
        reason = 'capture_error'
        raise
      finally:
        footer = {'kind': 'end', 'reason': reason if not stopped or reason in ('size_limit', 'capture_error') else 'stopped',
                  'timestamp_ns': log_time_ns(), 'shadow_error': shadow_error,
                  'elapsed_s': time.monotonic()-started, 'cpu_user_s': resource.getrusage(resource.RUSAGE_SELF).ru_utime,
                  'cpu_system_s': resource.getrusage(resource.RUSAGE_SELF).ru_stime}
        write_record(output, footer, written, limit)
        output.flush()
        status_path.write_text(json.dumps({'recording': False, 'reported_monotonic_s': time.monotonic(), **footer}))


if __name__ == '__main__':
  main()
