"""Bounded receive-only observer; original messages remain in normal route rlogs."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import time

from openpilot.tools.car_porting.bosch_c_radar_time import log_time_ns


class CaptureWindow:
  def __init__(self, now, duration=600, wait=900):
    self.started, self.duration, self.wait = now, duration, wait
    self.moving_since = self.parked_since = None

  def update(self, now, moving, parked):
    if moving and self.moving_since is None:
      self.moving_since = now
    if self.moving_since is None:
      return 'waiting_timeout' if now-self.started >= self.wait else None
    if now-self.moving_since >= self.duration:
      return 'duration'
    if parked and now-self.moving_since >= 60:
      if self.parked_since is None:
        self.parked_since = now
      if now-self.parked_since >= 30:
        return 'parked'
    else:
      self.parked_since = None
    return None


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--duration', type=int, default=600)
  args = parser.parse_args()
  if not 60 <= args.duration <= 1200:
    parser.error('Duration must be 60..1200 seconds')
  args.output.parent.mkdir(parents=True, exist_ok=True)
  if shutil.disk_usage(args.output.parent).free < 512*1024*1024:
    raise RuntimeError('At least 512 MiB free required')
  os.nice(10)
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  params = Params()
  if not params.get_bool('HondaBoschCExperimentalRadar'):
    raise RuntimeError('Experimental input must already be enabled; observer never changes it')
  services = ['radarState', 'radarTracks', 'carState', 'selfdriveState', 'longitudinalPlan', 'carControl', 'managerState']
  stopped = False

  def stop(*_):
    nonlocal stopped
    stopped = True

  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  status = args.output.parent/'coexistence.status.json'
  with (args.output.parent/'.capture.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sm = messaging.SubMaster(services, poll='radarState')
    window = CaptureWindow(time.monotonic(), args.duration)
    routes = set()
    last_write = last_status = last_route = 0.
    reason = 'error'
    with args.output.open('x') as output:
      def put(row):
        output.write(json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n')

      put({'kind': 'session', 'boot_ns': log_time_ns(), 'duration_s': args.duration, 'waiting_limit_s': 900,
           'original_data': 'normal rlogs for routes listed below; this file is a decimated health/signal summary',
           'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
           'configuration_hex': {k: (params.get(k) or b'').hex() for k in ('CarParams', 'CarParamsSP')}})
      try:
        while not stopped:
          sm.update(100)
          now = time.monotonic()
          cs, sd = sm['carState'], sm['selfdriveState']
          radar, tracks = sm['radarState'], sm['radarTracks']
          plan, control = sm['longitudinalPlan'], sm['carControl']
          # Fields are only used to end capture when carState is actually alive.
          reason = window.update(now, sm.alive['carState'] and cs.vEgo > .5,
                                 sm.alive['carState'] and str(cs.gearShifter) == 'park' and cs.vEgo < .1)
          if reason:
            break
          if output.tell() >= 32*1024*1024:
            reason = 'size_limit'
            break
          if now-last_route >= 1:
            route = params.get('CurrentRoute')
            if route and route not in routes:
              routes.add(route)
              put({'kind': 'route', 'boot_ns': log_time_ns(), 'route': route})
            last_route = now
          healthy = sm.valid['radarTracks'] and sm.alive['radarTracks'] and sm.freq_ok['radarTracks']
          if now-last_write >= .1:
            put({'kind': 'sample', 'boot_ns': log_time_ns(), 'valid': sm.valid, 'alive': sm.alive, 'frequency_ok': sm.freq_ok,
                 'source_times_ns': sm.logMonoTime, 'speed': cs.vEgo, 'gear': str(cs.gearShifter),
                 'can_valid': cs.canValid, 'acc_fault': cs.accFaulted, 'enabled': sd.enabled, 'active': sd.active,
                 'long_active': control.longActive, 'actuator_accel': control.actuators.accel,
                 'planner_accel': plan.aTarget, 'planner_source': str(plan.longitudinalPlanSource), 'fcw': plan.fcw,
                 'radar_errors': tracks.errors.to_dict(), 'points': len(tracks.points),
                 'lead_one': radar.leadOne.to_dict(), 'lead_two': radar.leadTwo.to_dict(),
                 'logger_running': any(p.name == 'loggerd' and p.running for p in sm['managerState'].processes)})
            last_write = now
          if now-last_status >= .5:
            output.flush()
            row = {'recording': True, 'reported_monotonic_s': now, 'object_stream': healthy, 'output': str(args.output),
                   'mode': 'live_drive', 'moving_started': window.moving_since is not None, 'routes': sorted(routes)}
            temporary = status.with_suffix('.tmp')
            temporary.write_text(json.dumps(row))
            temporary.replace(status)
            last_status = now
        if stopped:
          reason = 'stopped'
      finally:
        put({'kind': 'end', 'reason': reason or 'error', 'boot_ns': log_time_ns(), 'routes': sorted(routes)})
        output.flush()
        status.write_text(json.dumps({'recording': False, 'reported_monotonic_s': time.monotonic(), 'reason': reason,
                                      'mode': 'live_drive', 'output': str(args.output)}))


if __name__ == '__main__':
  main()
