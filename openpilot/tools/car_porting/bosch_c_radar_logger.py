#!/usr/bin/env python3
"""Read-only Bosch C diagnostics alongside stock Honda longitudinal control.

Writes local JSONL only. Does not publish radarData, send CAN, alter parameters,
or register an interface. Requires explicit provisional calibration. Existing
rlogs remain the authoritative raw capture; this file adds decoded diagnostics.
"""
import argparse
import fcntl
import os
import signal
import threading
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import bosch_c_radar
from opendbc.car.honda.bosch_c_radar import BoschCRadarInterface, CandidateCalibration, OBJECT_IDS, STALE_NS


def diagnostic_record(adapter, packets, *, now_nanos=None):
  # A timeout heartbeat can advance past a subsequently delivered packet's
  # timestamp. Retain its raw bytes in this log, but never rewind the decoder.
  watermark = adapter.decoder.now_ns
  timely = [(stamp, frames) for stamp, frames in packets if watermark is None or stamp >= watermark]
  result = adapter.update(timely, now_nanos=now_nanos)
  raw = [{'timestamp_ns': stamp, 'address': c.address, 'src': c.src, 'payload': c.dat.hex()}
         for stamp, frames in packets for c in frames if c.src == adapter.decoder.bus and c.address in OBJECT_IDS]
  if result is None and not raw:
    return None
  return {'kind': 'update', 'radar_data': result.to_dict() if result is not None else None,
              'diagnostics': adapter.diagnostics(), 'raw_frames': raw, 'late_packets_not_decoded': len(packets) - len(timely)}


def write_record(output, record, written_bytes, max_bytes):
  data = (json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n').encode()
  if written_bytes + len(data) > max_bytes:
    return None
  output.write(data)
  return written_bytes + len(data)


def run_capture(adapter, receive, output, *, duration_s, max_bytes, clock=time.monotonic, should_stop=lambda: False, progress=None):
  """Injectable receive loop. Receive must return promptly, including on silence."""
  started = clock()
  count = 0
  header = {'kind': 'session', 'schema_version': 1, 'experimental': True,
                'platform': str(adapter.CP.carFingerprint), 'receive_bus': adapter.decoder.bus,
                'calibration': asdict(adapter.calibration), 'firmware': [{'ecu': str(f.ecu), 'address': f.address,
                  'version_hex': bytes(f.fwVersion).hex()} for f in adapter.CP.carFw],
                'duration_s': duration_s, 'max_bytes': max_bytes,
                'adapter_sha256': hashlib.sha256(Path(bosch_c_radar.__file__).read_bytes()).hexdigest()}
  written = write_record(output, header, 0, max_bytes)
  if written is None:
    raise ValueError('Output limit is too small for the session header')
  def report(state, reason=None):
    result = {'state': state, 'reason': reason, 'updates': count, 'bytes': written,
              'elapsed_s': max(0., clock() - started), 'banks': adapter.decoder.counters['accepted_banks']}
    if progress is not None:
      progress(result)
    return result

  if progress is not None:
    report('waiting')
  reason = 'duration'
  while clock() - started < duration_s:
    if should_stop():
      reason = 'stopped'
      break
    packets = receive()
    record = diagnostic_record(adapter, packets)
    if record is not None:
      new_written = write_record(output, record, written, max_bytes)
      if new_written is None:
        reason = 'size_limit'
        break
      written = new_written
      count += 1
      output.flush()
    if progress is not None:
      last_bank = adapter.decoder.last_bank_ns
      fresh = last_bank is not None and adapter.decoder.now_ns - last_bank <= STALE_NS and not adapter.decoder.fault
      report('recording' if fresh else 'waiting')
  if progress is not None:
    return report('finished', reason)
  return {'reason': reason, 'updates': count, 'bytes': written}


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--calibration', required=True, type=Path, help='Research calibration JSON with a parameters object')
  ap.add_argument('--output', required=True, type=Path, help='New local JSONL file; existing files are never overwritten')
  ap.add_argument('--bus', type=int, default=1, help='Observed receive bus; default 1, never inferred from transmit receipts')
  ap.add_argument('--duration', type=float, default=600, help='Maximum capture seconds, default 600')
  ap.add_argument('--max-mib', type=int, default=256, help='Maximum output size, default 256 MiB')
  ap.add_argument('--status', type=Path, help='Optional atomic progress file for the comma four capture button')
  ap.add_argument('--parent-pid', type=int, help='Stop if the launching UI process exits')
  args = ap.parse_args()
  if not 0 < args.duration < float('inf') or args.max_mib <= 0:
    ap.error('Duration and output size must be positive and finite')
  calibration = CandidateCalibration(**json.loads(args.calibration.read_text())['parameters'])

  stopped = threading.Event()
  signal.signal(signal.SIGTERM, lambda *_: stopped.set())
  signal.signal(signal.SIGINT, lambda *_: stopped.set())
  last_status = 0.

  def progress(record):
    nonlocal last_status
    if args.status is None:
      return
    now = time.monotonic()
    if now - last_status < .5 and record['state'] not in ('finished', 'error'):
      return
    temporary = args.status.with_suffix('.tmp')
    temporary.write_text(json.dumps(record, allow_nan=False) + '\n')
    temporary.replace(args.status)
    last_status = now

  def should_stop():
    return stopped.is_set() or (args.parent_pid is not None and os.getppid() != args.parent_pid)

  # A lock prevents duplicate UI/CLI captures in this output directory. It is
  # released by the OS even if the UI or worker crashes.
  with (args.output.parent / '.capture.lock').open('a') as lock:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      # Live dependencies are deliberately lazy: help and offline tests need no IPC.
      from openpilot.cereal import messaging
      from openpilot.common.params import Params
      data = Params().get('CarParams')
      if not data:
        raise RuntimeError('No CarParams available')
      with structs.CarParams.from_bytes(data) as reader:
        cp = reader.as_builder()
      if cp.openpilotLongitudinalControl:
        raise RuntimeError('Passive capture requires Honda longitudinal control')
      adapter = BoschCRadarInterface(cp, structs.CarParamsSP(), calibration=calibration, bus=args.bus)
      sock = messaging.sub_sock('can', timeout=100)

      def receive():
        return [(e.logMonoTime, [CanData(c.address, bytes(c.dat), c.src) for c in e.can])
                for e in messaging.drain_sock(sock, wait_for_one=True)]

      with args.output.open('xb') as output:
        summary = run_capture(adapter, receive, output, duration_s=args.duration, max_bytes=args.max_mib * 1024 * 1024,
                              should_stop=should_stop, progress=progress)
        print(json.dumps(summary))
    except Exception as exc:
      progress({'state': 'error', 'error': str(exc)})
      raise


if __name__ == '__main__':
  main()
