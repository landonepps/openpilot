"""One-shot pre-manager launch, only when explicitly armed while parked."""
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

CAPTURE_DIR = Path('/data/media/0/bosch_c_radar')


def launch_armed(directory=CAPTURE_DIR, *, popen=subprocess.Popen, clock=time.monotonic, sleep=time.sleep, timeout=5.):
  directory = Path(directory)
  arm = directory / 'arm-next-startup'
  name = 'startup-' + uuid.uuid4().hex
  claim = directory / (name + '.armed')
  try:
    arm.rename(claim)  # Consume once, including failed attempts. Never auto-retry.
  except FileNotFoundError:
    return {'state': 'not_armed'}
  output = directory / (name + '.jsonl')
  ready = directory / (name + '.ready.json')
  result = {'state': 'starting', 'output': str(output)}
  try:
    with output.with_suffix('.stderr.log').open('xb') as errors:
      process = popen([sys.executable, '-m', 'openpilot.tools.car_porting.bosch_c_coexistence',
                       '--output', str(output), '--duration', '180', '--max-mib', '128', '--ready-file', str(ready)],
                      cwd=Path(__file__).resolve().parents[3], stdin=subprocess.DEVNULL,
                      stdout=errors, stderr=errors, start_new_session=True)
    result['pid'] = process.pid
    deadline = clock() + timeout
    while clock() < deadline:
      if process.poll() is not None:
        result['state'] = 'recorder_exited'
        break
      try:
        record = json.loads(ready.read_text())
        if record['pid'] == process.pid and record['output'] == str(output):
          result['state'] = 'ready'
          break
      except (OSError, ValueError, KeyError):
        pass
      sleep(.05)
    else:
      # The bounded recorder may finish starting later; the analyzer will refuse
      # a verdict if it missed the transition. Never block normal manager startup.
      result['state'] = 'readiness_timeout'
  except Exception as exc:
    result.update(state='launch_failed', error=str(exc))
  (directory / (name + '.launch.json')).write_text(json.dumps(result) + '\n')
  return result


def main():
  try:
    print(json.dumps(launch_armed()))
  except Exception as exc:
    # Diagnostics must not prevent manager startup, including full/read-only disk.
    print(f'Passive startup capture unavailable: {exc}', file=sys.stderr)


if __name__ == '__main__':
  main()
