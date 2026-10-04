"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import platform


from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import (CRUISE_LAG, GENTLE_DECEL, R, TARGET_OFFSET,
                                                                                           SmartCruiseControlMap, gentle_braking_target)
from openpilot.common.test import OpenpilotTestCase

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState


def east_of_origin(d: float) -> float:
  """Longitude (deg) of the point d meters east of (0, 0)."""
  return (d / R) * (180.0 / math.pi)


def envelope(tv: float, d: float, v_ego: float) -> float:
  return math.sqrt(tv ** 2 + 2 * GENTLE_DECEL * max(d - tv * TARGET_OFFSET - v_ego * CRUISE_LAG, 0.))


class TestSmartCruiseControlMap(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.reset_params()
    self.scc_m = SmartCruiseControlMap()

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

    # TODO-SP: mock data from gpsLocation
    self.params.put("LastGPSPosition", "{}", block=True)
    self.params.put("MapTargetVelocities", "{}", block=True)

  def test_initial_state(self):
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active
    assert self.scc_m.output_v_target == V_CRUISE_UNSET
    assert self.scc_m.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlMap", False, block=True)
    self.scc_m.enabled = self.params.get_bool("SmartCruiseControlMap")

    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(False, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.enabled

  def test_moderate_curve(self):
    # Regression: `... / 2 * a` parsed as `(.../2)*a` instead of `.../(2*a)`,
    # making max_d ~11x too small so the moderate-curve branch never tripped.
    # v_ego=25, a_ego=0, tv=24: fixed max_d≈45m vs buggy ≈4m at a 40m waypoint.
    waypoint_lon_deg = (40.0 / R) * (180.0 / math.pi)
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": 0.0, "longitude": 0.0}), block=True)
    self.mem_params.put("MapTargetVelocities",
                        json.dumps([{"latitude": 0.0, "longitude": waypoint_lon_deg, "velocity": 24.0}]), block=True)

    self.scc_m.update(True, False, 25.0, 0.0, 30.0)

    self.assertAlmostEqual(self.scc_m.v_target, 24.0, delta=24.0 * 1e-6)

  def curve_ahead(self, points: list[tuple[float, float]]) -> None:
    """mapd's target velocities at (distance east of the car in m, velocity in m/s), with the car at (0, 0)."""
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": 0.0, "longitude": 0.0}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps([{"latitude": 0.0, "longitude": east_of_origin(d), "velocity": v} for d, v in points]),
                        block=True)

  def test_gentle_braking_target(self):
    points = [{"latitude": 0.0, "longitude": 0.0, "velocity": v} for v in (25.0, 20.0, 0.0)]
    # the point that needs the most braking binds, and mapd's zero-velocity entries are skipped
    assert gentle_braking_target(points, [300., 600., 5.], 29.) == min(envelope(25., 300., 29.), envelope(20., 600., 29.))
    assert gentle_braking_target(points[2:], [5.], 29.) == 0.

  def test_gentle_braking_starts_braking_earlier(self):
    # 400 m from a 22 m/s point at 29 m/s: past the gentle envelope, but well outside the stock -1.2 m/s^2 braking distance
    self.curve_ahead([(400., 22.)])
    for _ in range(3):
      self.scc_m.update(True, False, 29., 0., 31.)
    assert self.scc_m.state == MapState.enabled

    self.params.put_bool("MapCurveGentleBraking", True, block=True)
    scc_m = SmartCruiseControlMap()
    for _ in range(3):
      scc_m.update(True, False, 29., 0., 31.)
    assert scc_m.state == MapState.turning
    self.assertAlmostEqual(scc_m.output_v_target, envelope(22., 400., 29.), delta=1e-3)
    assert scc_m.output_v_target < 29.

  def test_gentle_braking_limits_pickup_to_the_next_point(self):
    # below the next point's velocity, the target still caps the speed at what the point allows
    self.params.put_bool("MapCurveGentleBraking", True, block=True)
    self.curve_ahead([(100., 22.)])
    scc_m = SmartCruiseControlMap()
    for _ in range(3):
      scc_m.update(True, False, 20., 0., 31.)
    assert scc_m.state == MapState.turning
    self.assertAlmostEqual(scc_m.output_v_target, envelope(22., 100., 20.), delta=1e-3)
    assert 22. < scc_m.output_v_target < 31.

  def test_lat_accel_scales_target_velocities(self):
    self.curve_ahead([(40., 24.)])
    for lat_accel, scale in ((2.5, math.sqrt(2.5 / 2.0)), (5.0, math.sqrt(3.0 / 2.0))):  # bounded to 3.0
      self.params.put("MapCurveLatAccel", lat_accel, block=True)
      scc_m = SmartCruiseControlMap()
      scc_m.update(True, False, 40.0, 0.0, 45.0)
      self.assertAlmostEqual(scc_m.v_target, 24.0 * scale, delta=1e-6)

  # TODO-SP: mock data from modelV2 to test other states
