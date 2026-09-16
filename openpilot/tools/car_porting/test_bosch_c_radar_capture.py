import json
import signal
from types import SimpleNamespace

import pytest

from openpilot.tools.car_porting.bosch_c_radar_capture import (
  CALIBRATION_PATH, MIN_FREE_BYTES, RadarCaptureController, capture_allowed,
)


def cp(longitudinal=False, fingerprint='HONDA_CRV_6G'):
  return SimpleNamespace(brand='honda', carFingerprint=fingerprint, openpilotLongitudinalControl=longitudinal)


@pytest.mark.parametrize('device', ['tici', 'tizi', 'pc', 'unknown'])
def test_other_devices_never_enable_capture(device):
  assert not capture_allowed(device, True, cp())


def test_capture_gate_requires_comma_four_crv_and_stock_longitudinal():
  assert capture_allowed('mici', True, cp())
  assert not capture_allowed('mici', False, cp())
  assert not capture_allowed('mici', True, None)
  assert not capture_allowed('mici', True, cp(True))
  assert not capture_allowed('mici', True, cp(fingerprint='HONDA_CIVIC'))


class FakeProcess:
  def __init__(self):
    self.code = None
    self.signals = []
    self.killed = False

  def poll(self):
    return self.code

  def send_signal(self, sig):
    self.signals.append(sig)

  def kill(self):
    self.killed = True


@pytest.fixture
def capture(tmp_path):
  processes = []
  commands = []
  now = [0.]

  def popen(args, **kwargs):
    commands.append((args, kwargs))
    processes.append(FakeProcess())
    return processes[-1]

  obj = RadarCaptureController(tmp_path, clock=lambda: now[0], popen=popen,
                               disk_usage=lambda _: SimpleNamespace(free=MIN_FREE_BYTES))
  return obj, now, processes, commands


def status(obj, state, **kwargs):
  obj.status_path.write_text(json.dumps({'state': state, **kwargs}))


def test_start_launches_one_bounded_child_with_bundled_calibration(capture):
  obj, _, processes, commands = capture
  obj.start(False)
  assert not processes
  obj.start(True)
  obj.start(True)
  assert len(processes) == 1
  args, kwargs = commands[0]
  assert args[1:3] == ['-m', 'openpilot.tools.car_porting.bosch_c_radar_capture']
  assert args[args.index('--calibration') + 1] == str(CALIBRATION_PATH)
  assert args[args.index('--duration') + 1] == '600'
  assert args[args.index('--max-mib') + 1] == '256'
  assert '--parent-pid' in args and kwargs['start_new_session']
  assert not kwargs.get('shell', False)
  assert obj.state == 'starting'


def test_recording_only_after_worker_reports_data_then_stop_and_save(capture):
  obj, now, processes, _ = capture
  obj.toggle(True)
  obj.poll(True)
  assert obj.state == 'starting'
  status(obj, 'waiting', banks=0, elapsed_s=1)
  now[0] = 1
  obj.poll(True)
  assert obj.state == 'waiting'
  status(obj, 'recording', banks=15, elapsed_s=65)
  now[0] = 2
  obj.poll(True)
  assert obj.label == 'Stop 1:05'
  obj.toggle(True)
  obj.toggle(True)
  assert processes[0].signals == [signal.SIGTERM]
  status(obj, 'finished', banks=15, reason='stopped')
  processes[0].code = 0
  now[0] = 3
  obj.poll(True)
  assert not obj.active and obj.state == 'saved'
  now[0] = 9
  obj.poll(True)
  assert obj.state == 'idle'


def test_offroad_or_platform_change_stops_capture(capture):
  obj, _, processes, _ = capture
  obj.start(True)
  obj.poll(False)
  assert processes[0].signals == [signal.SIGTERM]
  assert obj.state == 'stopping'


def test_hung_child_is_killed_without_waiting_in_ui(capture):
  obj, now, processes, _ = capture
  obj.start(True)
  now[0] = 16
  obj.poll(True)
  assert obj.state == 'stopping'
  now[0] = 20
  obj.poll(True)
  assert processes[0].killed
  processes[0].code = -signal.SIGKILL
  now[0] = 21
  obj.poll(True)
  assert obj.state == 'error'


def test_worker_failure_is_visible_and_retry_uses_new_files(capture):
  obj, now, processes, _ = capture
  obj.start(True)
  first = obj.output_path
  status(obj, 'error', error='CAN unavailable')
  processes[0].code = 1
  obj.poll(True)
  assert obj.state == 'error' and obj.error == 'CAN unavailable'
  now[0] = 1
  obj.toggle(True)
  assert obj.output_path != first and len(processes) == 2


def test_no_false_saved_state_without_any_radar_banks(capture):
  obj, _, processes, _ = capture
  obj.start(True)
  status(obj, 'finished', banks=0)
  processes[0].code = 0
  obj.poll(True)
  assert obj.label == 'No radar'


def test_low_storage_and_launch_failure_are_reported(capture):
  obj, _, processes, _ = capture
  obj.disk_usage = lambda _: SimpleNamespace(free=0)
  obj.start(True)
  assert obj.state == 'error' and not processes
  assert obj.label == 'Low storage'
  obj.disk_usage = lambda _: SimpleNamespace(free=MIN_FREE_BYTES)
  obj.popen = lambda *args, **kwargs: (_ for _ in ()).throw(OSError('launch failed'))
  obj.start(True)
  assert obj.state == 'error' and obj.error == 'launch failed'


def test_bundled_calibration_is_explicitly_provisional():
  calibration = json.loads(CALIBRATION_PATH.read_text())
  assert 'candidate' in calibration['status']
  assert calibration['parameters']['x_reference_offset'] == pytest.approx(-4.296)
