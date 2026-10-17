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
import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import (CRUISE_LAG, GENTLE_DECEL, MAPD_MERGE_VELOCITY, R,
                                                                                           SMOOTH_BRAKE_START, SMOOTH_FIT_AVERAGE, SMOOTH_FIT_LENGTH,
                                                                                           SMOOTH_FIT_STEP, SMOOTH_HOLD_MARGIN, SMOOTH_HOLD_TIME,
                                                                                           SMOOTH_MERGE_DIST, SMOOTH_MERGE_WEIGHT, TARGET_OFFSET,
                                                                                           TO_DEGREES, SmartCruiseControlMap, fitted_curvatures,
                                                                                           gentle_braking_target, merge_nodes, path_from_points,
                                                                                           project_on_path, smooth_slowing_target, solve_banded_spd,
                                                                                           stitch_nodes)
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
    for lat_accel, scale in ((2.5, math.sqrt(2.5 / 2.0)), (5.0, math.sqrt(3.5 / 2.0))):  # bounded to 3.5
      self.params.put("MapCurveLatAccel", lat_accel, block=True)
      scc_m = SmartCruiseControlMap()
      scc_m.update(True, False, 40.0, 0.0, 45.0)
      self.assertAlmostEqual(scc_m.v_target, 24.0 * scale, delta=1e-6)

  # TODO-SP: mock data from modelV2 to test other states


def arc_points(radius: float, length: float, straight: float = 0., step: float = 20.) -> list[dict]:
  """mapd-style points (lat, lon, velocity) along `straight` m due east from (0, 0), then a left arc of `radius`."""
  pts = []
  for d in np.arange(0., straight + length + 1e-6, step):
    if d <= straight:
      x, y = d, 0.
    else:
      a = (d - straight) / radius
      x, y = straight + radius * math.sin(a), radius * (1. - math.cos(a))
    pts.append({"latitude": y / R * TO_DEGREES, "longitude": x / R * TO_DEGREES, "velocity": 30.})
  return pts


def xy(points):
  return (np.array([p["longitude"] for p in points]) / TO_DEGREES * R, np.array([p["latitude"] for p in points]) / TO_DEGREES * R)


def dense_fitted_curvatures(x, y, merge):
  """fitted_curvatures with its normal equations built and solved as dense matrices, as it first shipped."""
  s = np.concatenate([[0.], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
  n = len(s)
  m = max(math.ceil(s[-1] / SMOOTH_FIT_STEP) + 1, 5)
  h = s[-1] / (m - 1)
  j = np.minimum((s / h).astype(int), m - 2)
  u = s / h - j
  interp = np.zeros((n, m))
  interp[np.arange(n), j] = 1. - u
  interp[np.arange(n), j + 1] = u
  third = np.diff(np.eye(m), n=3, axis=0) / h ** 3
  ds = np.diff(s)
  w = np.concatenate([[ds[0]], ds[:-1] + ds[1:], [ds[-1]]]) / 2.
  if merge.any():
    w[(np.abs(s[:, None] - s[merge][None, :]) <= SMOOTH_MERGE_DIST).any(axis=1)] *= SMOOTH_MERGE_WEIGHT
  lhs = interp.T @ (w[:, None] * interp) + SMOOTH_FIT_LENGTH ** 6 * h * (third.T @ third)
  path = np.linalg.solve(lhs, interp.T @ (w[:, None] * np.stack([x, y], axis=1)))
  s_grid = np.linspace(0., s[-1], m)
  d1 = np.gradient(path, s_grid, axis=0)
  d2 = np.gradient(d1, s_grid, axis=0)
  curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(np.hypot(d1[:, 0], d1[:, 1]), 1e-6) ** 3
  heading = np.concatenate([[0.], np.cumsum((curvature[1:] + curvature[:-1]) / 2. * h)])
  lo = np.clip(s - SMOOTH_FIT_AVERAGE / 2., 0., s[-1])
  hi = np.clip(s + SMOOTH_FIT_AVERAGE / 2., 0., s[-1])
  return (np.interp(hi, s_grid, heading) - np.interp(lo, s_grid, heading)) / np.maximum(hi - lo, SMOOTH_FIT_AVERAGE / 2.)


class TestSmoothSlowingGeometry(OpenpilotTestCase):
  def test_arc_curvature_is_its_inverse_radius(self):
    x, y = xy(arc_points(250., 600.))
    _, k = fitted_curvatures(x, y, np.zeros(len(x), dtype=bool))
    np.testing.assert_allclose(k[2:-2], 1. / 250., rtol=0.01)

  def test_sideways_node_barely_bends_the_path(self):
    # a straight road with one node 3 m to the side: 3-node curvature reads about 15 /km there
    x, y = np.arange(0., 400., 20.), np.zeros(20)
    y[10] = 3.
    _, k = fitted_curvatures(x, y, np.zeros(len(x), dtype=bool))
    assert np.abs(k).max() < 0.001

  def test_arc_after_a_straight_is_entered_through_a_spiral(self):
    # no spiral on the map: the path's curvature builds over about 80 m and overshoots the arc's a little
    x, y = xy(arc_points(200., 600., straight=300.))
    s, k = fitted_curvatures(x, y, np.zeros(len(x), dtype=bool))
    assert abs(k[s < 250.]).max() < 0.1 / 200. and abs(k[(s > 300.) & (s < 330.)]).max() < 0.9 / 200.
    np.testing.assert_allclose(k[(s > 360.) & (s < 840.)], 1. / 200., rtol=0.06)

  def test_merge_jump_is_crossed_gently(self):
    # a straight road whose centerline moves 8 m sideways at a merge node, onto the wider road's
    x = np.arange(0., 600., 25.)
    y = np.where(np.arange(len(x)) >= 12, 8., 0.)
    merge = np.zeros(len(x), dtype=bool)
    _, k = fitted_curvatures(x, y, merge)
    merge[12] = True
    _, k_merge = fitted_curvatures(x, y, merge)
    assert np.abs(k).max() > 0.002 and np.abs(k_merge).max() < 0.0015

  def test_banded_solve_matches_a_dense_solve(self):
    rng = np.random.default_rng(0)
    n, p = 60, 3
    band = np.zeros((p + 1, n))
    band[1:] = rng.uniform(-1., 1., (p, n))
    band[0] = 2. * p + 1.  # diagonally dominant, so positive definite
    a = np.diag(band[0])
    for d in range(1, p + 1):
      a += np.diag(band[d, :n - d], d) + np.diag(band[d, :n - d], -d)
    rhs = rng.uniform(-1., 1., (n, 2))
    np.testing.assert_allclose(solve_banded_spd(band, rhs), np.linalg.solve(a, rhs), rtol=1e-10, atol=1e-12)

  def test_long_path_matches_the_dense_fit(self):
    # mapd's freeway lists run to 4.5 km (a 900-point grid); the banded equations give the dense fit's curvature
    s = np.linspace(0., 4400., 51)
    x, y = s, 200. * np.sin(s / 700.)
    merge = np.zeros(len(s), dtype=bool)
    merge[20] = True
    _, k = fitted_curvatures(x, y, merge)
    np.testing.assert_allclose(k, dense_fitted_curvatures(x, y, merge), rtol=1e-6, atol=1e-9)

  def test_merge_nodes_come_from_mapds_velocity(self):
    points = arc_points(250., 200.)
    points[3]["velocity"] = MAPD_MERGE_VELOCITY
    merges = merge_nodes(points)
    assert merges == {(points[3]["latitude"], points[3]["longitude"])}
    nodes = [(p["latitude"], p["longitude"]) for p in points]
    _, _, merge, _ = path_from_points(nodes + nodes[-1:], merges)  # a repeated node is dropped
    assert merge.tolist() == [i == 3 for i in range(len(points))]

  def test_projection_extends_behind_the_first_node(self):
    x, y = np.array([0., 50., 100.]), np.zeros(3)
    assert project_on_path(x, y, 60., 2.) == (60., 2.)
    s, off = project_on_path(x, y, -120., 1.)
    assert s == -120. and off == 1.

  def test_stitch_keeps_the_previous_nodes_behind(self):
    old = arc_points(250., 300.)
    new = old[5:] + arc_points(250., 100.)[:2]
    nodes = stitch_nodes([(p["latitude"], p["longitude"]) for p in old], new)
    assert nodes[:5] == [(p["latitude"], p["longitude"]) for p in old[:5]] and len(nodes) == len(old) + 2
    # mapd's zero-velocity entries carry no node; an unrelated list isn't stitched
    assert stitch_nodes([(1., 1.)], [{"latitude": 0., "longitude": 0., "velocity": 0.}] + new) == \
      [(p["latitude"], p["longitude"]) for p in new]


V_SET = 40.  # m/s, a set speed above every node in these tests


class TestSmoothSlowingTarget(OpenpilotTestCase):
  def test_brakes_at_the_deceleration_the_node_needs(self):
    s_nodes, v_nodes = np.array([300.]), np.array([25.])
    # (31^2 - 25^2) / 2 / (300 - 25) = 0.611 m/s^2
    v, braking = smooth_slowing_target(s_nodes, v_nodes, 0., 31., V_SET, False)
    assert braking and abs((31. - v) - (31. ** 2 - 25. ** 2) / (2. * 275.)) < 1e-9

  def test_waits_until_the_need_reaches_the_start(self):
    # 0.15 m/s^2 doesn't start braking yet, and the node is past the hold's horizon: the envelope caps pickups
    s_nodes, v_nodes = np.array([600.]), np.array([28.])
    v, braking = smooth_slowing_target(s_nodes, v_nodes, 0., 31., V_SET, False)
    assert not braking and v > 31.
    assert abs(v - math.sqrt(28. ** 2 + 2. * SMOOTH_BRAKE_START * (600. - 28. * TARGET_OFFSET))) < 1e-9

  def test_envelope_caps_the_target_near_a_node(self):
    # 5 m past its margin, 0.3 m/s over: too little to start braking, but more than braking at the start rate allows
    v, braking = smooth_slowing_target(np.array([30.]), np.array([25.]), 0., 25.3, V_SET, False)
    assert not braking and abs(v - math.sqrt(25. ** 2 + 2. * SMOOTH_BRAKE_START * 5.)) < 1e-9

  def test_a_late_node_is_braked_for_over_at_least_a_second(self):
    # 5 m past its margin: braking distance counts as one second of travel, not 5 m
    v, braking = smooth_slowing_target(np.array([30.]), np.array([25.]), 0., 26., V_SET, False)
    assert braking and abs((26. - v) - (26. ** 2 - 25. ** 2) / (2. * 26.)) < 1e-9

  def test_node_within_its_margin_holds_its_speed(self):
    v, _ = smooth_slowing_target(np.array([20.]), np.array([25.]), 0., 26., V_SET, False)
    assert v == 25.

  def test_no_pickup_through_a_curve(self):
    # slower than the curve's next nodes by less than the margin: hold, never brake
    s_nodes, v_nodes = np.array([30., 60., 90.]), np.array([26., 25.5, 26.])
    v, braking = smooth_slowing_target(s_nodes, v_nodes, 0., 25., V_SET, False)
    assert not braking and v == 25.
    v, _ = smooth_slowing_target(s_nodes, v_nodes + 3., 0., 25., V_SET, False)
    assert v == 25.5 + 3. - SMOOTH_HOLD_MARGIN

  def test_no_pickup_before_a_slower_node_further_on(self):
    # the next nodes allow 29 m/s, but one 300 m (12 s) ahead allows 26: hold instead of picking up and braking again
    s_nodes, v_nodes = np.array([60., 300.]), np.array([29., 26.])
    v, braking = smooth_slowing_target(s_nodes, v_nodes, 0., 25.5, V_SET, False)
    assert not braking and v == 25.5
    v, _ = smooth_slowing_target(s_nodes[:1], v_nodes[:1], 0., 25.5, V_SET, False)
    assert v == 29. - SMOOTH_HOLD_MARGIN
    # past SMOOTH_HOLD_TIME of travel it doesn't hold the car back
    far = np.array([60., 25.5 * SMOOTH_HOLD_TIME + 50.])
    v, _ = smooth_slowing_target(far, v_nodes, 0., 25.5, V_SET, False)
    assert v == 29. - SMOOTH_HOLD_MARGIN

  def test_a_node_the_set_speed_clears_doesnt_hold(self):
    # set speed 29 m/s, a gentle curve ahead allows 29.5: pick up to the set speed, not to 28.5
    s_nodes, v_nodes = np.array([300.]), np.array([29.5])
    v, _ = smooth_slowing_target(s_nodes, v_nodes, 0., 28., 29., False)
    assert v == 29.5
    # the margin shrinks as the node's speed nears the set speed: half of it for a node 0.5 m/s under
    v, _ = smooth_slowing_target(s_nodes, v_nodes - 1., 0., 27., 29., False)
    assert v == 28.5 - 0.5

  def test_braking_goes_on_for_a_node_that_needs_little(self):
    # 0.03 m/s^2 doesn't start braking, but keeps it going: a slower node listed during braking joins the same braking
    s_nodes, v_nodes = np.array([437.]), np.array([24.5])
    v, braking = smooth_slowing_target(s_nodes, v_nodes, 0., 25., V_SET, True)
    assert braking and abs((25. - v) - 0.03) < 1e-6
    assert not smooth_slowing_target(s_nodes, v_nodes, 0., 25., V_SET, False)[1]

  def test_nothing_binds_without_nodes_ahead(self):
    assert smooth_slowing_target(np.array([10.]), np.array([20.]), 50., 30., V_SET, True) == (0., False)


class TestSmoothSlowingController(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.params.put_bool("SmartCruiseControlMap", True, block=True)
    self.params.put_bool("MapCurveSmoothSlowing", True, block=True)
    self.params.put("MapCurveLatAccel", 2.5, block=True)

  def set_map(self, points, east=1.):
    # (0, 0) itself reads as no position fix
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": 0.0, "longitude": east_of_origin(east)}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps(points), block=True)

  def test_slows_for_the_arc_ahead_at_the_lateral_accel(self):
    self.set_map(arc_points(200., 600., straight=300.))
    scc_m = SmartCruiseControlMap()
    for _ in range(3):
      scc_m.update(True, False, 31., 0., 33.)
    assert scc_m.state == MapState.turning and scc_m.smooth_braking
    # the arc, 299 m ahead, needs sqrt(2.5 * 200) = 22.4 m/s from about 60 m into it (the fitted path overshoots a little)
    self.assertAlmostEqual(math.sqrt(2.5 / np.abs(scc_m.path_curvatures).max()), math.sqrt(2.5 * 200.), delta=0.7)
    need = [(31. ** 2 - 2.5 * 200.) / (2. * (d - math.sqrt(2.5 * 200.) * TARGET_OFFSET)) for d in (299., 359.)]
    assert need[1] < 31. - scc_m.output_v_target < need[0]

  def test_distances_fall_between_position_fixes(self):
    self.set_map(arc_points(200., 600., straight=500.))
    scc_m = SmartCruiseControlMap()
    targets = []
    for _ in range(int(1. / DT_MDL)):
      scc_m.update(True, False, 33., 0., 33.)
      targets.append(scc_m.v_target)
    # the same position fix all second, yet the arc gets 1.65 m closer each tick and the braking target falls
    assert all(b < a for a, b in zip(targets[1:], targets[2:], strict=False))

  def test_merge_nodes_stay_marked_after_mapd_moves_on(self):
    points = arc_points(200., 600., straight=300.)
    points[6]["velocity"] = MAPD_MERGE_VELOCITY
    self.set_map(points)
    scc_m = SmartCruiseControlMap()
    scc_m.update(True, False, 31., 0., 33.)
    merge_node = (points[6]["latitude"], points[6]["longitude"])
    assert scc_m.path_merges == {merge_node}
    # mapd's next list starts past the merge node; the node is kept from the previous list, and so is its mark
    self.set_map([dict(p, velocity=30.) for p in points[8:]])
    scc_m.update(True, False, 31., 0., 33.)
    assert merge_node in scc_m.path_nodes and scc_m.path_merges == {merge_node}

  def test_ignores_a_path_the_car_is_not_on(self):
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": 100. / R * TO_DEGREES, "longitude": 0.}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps(arc_points(200., 600., straight=300.)), block=True)
    scc_m = SmartCruiseControlMap()
    for _ in range(3):
      scc_m.update(True, False, 31., 0., 33.)
    assert scc_m.state == MapState.enabled and scc_m.v_target == 0.

  def test_off_keeps_mapds_velocities(self):
    self.params.put_bool("MapCurveSmoothSlowing", False, block=True)
    self.curve = arc_points(200., 600., straight=300.)
    self.set_map(self.curve)
    scc_m = SmartCruiseControlMap()
    scc_m.update(True, False, 31., 0., 33.)
    assert not scc_m.smooth_slowing and len(scc_m.path_x) == 0
