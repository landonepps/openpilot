from openpilot.tools.car_porting.bosch_c_drive_observer import CaptureWindow


def test_waiting_and_driving_are_bounded():
  w = CaptureWindow(0, duration=600, wait=900)
  assert w.update(899, False, True) is None
  assert w.update(900, False, True) == 'waiting_timeout'
  w = CaptureWindow(0, duration=600)
  assert w.update(50, True, False) is None
  assert w.update(649, False, False) is None
  assert w.update(650, False, False) == 'duration'


def test_parked_end_requires_departure_and_stable_park():
  w = CaptureWindow(0)
  assert w.update(60, False, True) is None
  assert w.update(90, True, False) is None
  assert w.update(140, False, True) is None
  assert w.update(151, False, True) is None
  assert w.update(179, False, False) is None
  assert w.update(180, False, True) is None
  assert w.update(210, False, True) == 'parked'
