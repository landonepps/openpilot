"""Nonblocking UI controller and low-priority entry point for passive capture."""
from datetime import datetime, UTC
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

from openpilot.tools.car_porting.bosch_c_radar_time import LOG_CLOCK_NAME, log_time_ns

CALIBRATION_PATH = Path(__file__).with_name('bosch_c_candidate_calibration.json')
DURATION_S = 600
MAX_MIB = 256
MIN_FREE_BYTES = 512 * 1024 * 1024
PREVIEW_TIMEOUT_S = 1.5


def append_marker(path, record):
  with path.open('ab') as output:
    output.write((json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n').encode())


def capture_allowed(device_type, started, cp):
  return (device_type == 'mici' and started and cp is not None and cp.brand == 'honda' and
          cp.carFingerprint == 'HONDA_CRV_6G')


class RadarCaptureController:
  def __init__(self, output_dir, *, clock=time.monotonic, popen=subprocess.Popen, disk_usage=shutil.disk_usage):
    self.output_dir = Path(output_dir)
    self.clock = clock
    self.popen = popen
    self.disk_usage = disk_usage
    self.process = None
    self.state = 'idle'
    self.progress = {}
    self.output_path = None
    self.status_path = None
    self.error = None
    self.started_at = 0.
    self.stop_at = None
    self.finished_at = 0.
    self.last_poll = -1.
    self.marker_executor = None
    self.marker_future = None
    self.marker_notice = ''
    self.marker_notice_until = 0.
    self.coexistence_status = {}
    self.coexistence_poll = -1.

  @property
  def preview(self):
    age = self.clock() - self.progress.get('reported_monotonic_s', float('-inf'))
    if not self.active or self.state != 'recording' or not 0 <= age <= PREVIEW_TIMEOUT_S:
      return None
    return self.progress.get('radar')

  def mark(self):
    if not self.active or self.stop_at is not None or self.marker_future is not None:
      return
    record = {'kind': 'bookmark', 'timestamp_ns': log_time_ns(), 'timestamp_clock': LOG_CLOCK_NAME, 'utc': datetime.now(UTC).isoformat(),
              'capture_file': self.output_path.name, 'elapsed_s': self.clock() - self.started_at,
              'preview_fresh': self.preview is not None, 'radar': self.progress.get('radar')}
    # Sparse bookmark writes stay off the UI thread. At most one is pending.
    if self.marker_executor is None:
      self.marker_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='radar-marker')
    self.marker_future = self.marker_executor.submit(append_marker, self.output_path.with_suffix('.markers.jsonl'), record)

  def _poll_marker(self):
    if self.marker_future is not None and self.marker_future.done():
      self.marker_notice = 'Mark failed' if self.marker_future.exception() else 'Mark saved'
      self.marker_future = None
      self.marker_notice_until = self.clock() + 3
    if self.clock() > self.marker_notice_until:
      self.marker_notice = ''

  @property
  def active(self):
    return self.process is not None

  def start(self, allowed):
    if self.active or not allowed:
      return
    self.error = None
    self.progress = {}
    try:
      self.output_dir.mkdir(parents=True, exist_ok=True)
      if self.disk_usage(self.output_dir).free < MIN_FREE_BYTES:
        raise OSError('At least 512 MiB of free storage is required')
      if not CALIBRATION_PATH.is_file():
        raise OSError('Bundled candidate calibration is missing')
      name = datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
      self.output_path = self.output_dir / f'{name}.jsonl'
      self.status_path = self.output_dir / f'{name}.status.json'
      args = [sys.executable, '-m', 'openpilot.tools.car_porting.bosch_c_radar_capture',
              '--calibration', str(CALIBRATION_PATH), '--output', str(self.output_path),
              '--status', str(self.status_path), '--parent-pid', str(os.getpid()),
              '--duration', str(DURATION_S), '--max-mib', str(MAX_MIB), '--bus', '1']
      with self.output_path.with_suffix('.stderr.log').open('xb') as errors:
        self.process = self.popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                  stderr=errors, start_new_session=True, cwd=Path(__file__).resolve().parents[3])
      self.state = 'starting'
      self.started_at = self.clock()
      self.stop_at = None
      self.last_poll = -1.
    except OSError as exc:
      self.state, self.error, self.finished_at = 'error', str(exc), self.clock()

  def stop(self):
    if not self.active or self.stop_at is not None:
      return
    self.state = 'stopping'
    self.stop_at = self.clock()
    try:
      self.process.send_signal(signal.SIGTERM)
    except ProcessLookupError:
      pass

  def toggle(self, allowed):
    if self.coexistence_status.get('recording') and 0 <= self.clock() - self.coexistence_status.get('reported_monotonic_s', -100) < 1.5:
      return
    if self.active:
      self.stop()
    else:
      self.start(allowed)

  def poll(self, allowed):
    now = self.clock()
    self._poll_marker()
    if now - self.coexistence_poll >= .5:
      self.coexistence_poll = now
      try:
        self.coexistence_status = json.loads((self.output_dir / 'coexistence.status.json').read_text())
      except (OSError, ValueError):
        self.coexistence_status = {}
    if not self.active:
      if self.state in ('saved', 'stopped') and now - self.finished_at > 5:
        self.state = 'idle'
      return
    if not allowed:
      self.stop()
    if now - self.last_poll < .2:
      return
    self.last_poll = now
    try:
      self.progress = json.loads(self.status_path.read_text())
    except (OSError, ValueError):
      pass
    code = self.process.poll()
    if code is not None:
      self.process = None
      self.finished_at = now
      if self.progress.get('state') == 'error':
        self.state, self.error = 'error', self.progress.get('error', 'Capture failed')
      elif code == 0 and self.progress.get('state') == 'finished':
        self.state = 'saved' if self.progress.get('banks', 0) else 'stopped'
      elif self.stop_at is not None and code == -signal.SIGTERM and not self.progress:
        self.state = 'stopped'  # Stopped during Python startup, before any data.
      else:
        self.state, self.error = 'error', f'Capture exited with status {code}'
      return
    if self.stop_at is not None:
      if now - self.stop_at > 3:
        try:
          self.process.kill()
        except ProcessLookupError:
          pass
      return
    state = self.progress.get('state')
    if state in ('recording', 'waiting'):
      self.state = state
    if (state is None and now - self.started_at > 15) or now - self.started_at > DURATION_S + 5:
      self.stop()

  @property
  def label(self):
    coex = self.coexistence_status
    if coex.get('recording') and 0 <= self.clock() - coex.get('reported_monotonic_s', -100) < 1.5:
      return 'REC banks' if coex.get('object_stream') else 'REC wait'
    if self.state == 'recording':
      elapsed = max(0, int(self.progress.get('elapsed_s', 0)))
      return f'Stop {elapsed // 60}:{elapsed % 60:02d}'
    if self.state == 'error' and self.error and 'free storage' in self.error:
      return 'Low storage'
    return {'idle': 'Radar log', 'starting': 'Starting', 'waiting': 'Stop / wait', 'stopping': 'Saving',
            'saved': 'Saved', 'stopped': 'No radar', 'error': 'Log error'}.get(self.state, 'Radar log')


def main():
  # The UI runs at realtime priority. Do not inherit that priority for decoding
  # and disk I/O, and do this before importing the heavier logger dependencies.
  if hasattr(os, 'sched_setscheduler'):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.nice(10)
  from openpilot.tools.car_porting.bosch_c_radar_logger import main as logger_main
  logger_main()


if __name__ == '__main__':
  main()
