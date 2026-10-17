"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import cast

from openpilot.cereal import custom, messaging
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import MockDec, build_sm
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import GENTLE_PICKUP_JERK, GENTLE_PICKUP_MAX_ACCEL, \
  GENTLE_PICKUP_RELEASE_JERK

EPS = 1e-6


def build(v_ego: float, v_cruise_kph: float):
  CP = structs.CarParams()
  CP.steerRatio = 15.0
  CP.wheelbase = 2.7
  CP.longitudinalActuatorDelay = 0.2
  planner = LongitudinalPlanner(CP, custom.CarParamsSP.new_message().as_reader(), init_v=v_ego)
  planner.dec = cast(DynamicExperimentalController, MockDec(False, 'acc'))

  sm = build_sm(False)
  car_state = messaging.new_message('carState')
  car_state.carState.vEgo = v_ego
  car_state.carState.vCruise = v_cruise_kph
  car_state.carState.vCruiseCluster = v_cruise_kph
  sm['carState'] = car_state.carState.as_reader()
  return planner, sm


def targets(planner, sm, ticks: int) -> list[float]:
  out = []
  for _ in range(ticks):
    planner.update(sm)
    out.append(float(planner.output_a_target))
  return out


class TestGentlePickup(OpenpilotTestCase):
  def test_off_by_default(self):
    # 30 m/s with the set speed at 130 km/h: the cruise limit, about 0.73 m/s^2 here, comes through
    planner, sm = build(30., 130.)
    assert not planner.gentle_pickup
    assert max(targets(planner, sm, 60)) > 0.5

  def test_highway_acceleration_ramps_to_the_limit(self):
    Params().put_bool("GentleHighwayPickup", True)
    planner, sm = build(30., 130.)
    a = targets(planner, sm, 60)
    assert max(a) <= GENTLE_PICKUP_MAX_ACCEL[-1] + EPS
    assert all(b - prev <= GENTLE_PICKUP_JERK[-1] * DT_MDL + EPS for prev, b in zip(a, a[1:], strict=False))
    assert a[-1] > GENTLE_PICKUP_MAX_ACCEL[-1] - 0.01

  def test_city_speed_unchanged(self):
    planner_off, sm = build(10., 60.)
    off = targets(planner_off, sm, 40)
    Params().put_bool("GentleHighwayPickup", True)
    planner_on, sm = build(10., 60.)
    assert planner_on.gentle_pickup
    assert targets(planner_on, sm, 40) == off

  def test_braking_passes_through(self):
    Params().put_bool("GentleHighwayPickup", True)
    planner, _ = build(30., 130.)
    assert planner.limit_pickup(30., -1.5, 0.2, DT_MDL) == -1.5
    assert planner.limit_pickup(30., 0.1, -0.8, DT_MDL) == GENTLE_PICKUP_JERK[-1] * DT_MDL

  def test_higher_acceleration_comes_down_gradually(self):
    # after a gas override the previous target can be above the limit
    Params().put_bool("GentleHighwayPickup", True)
    planner, _ = build(30., 130.)
    assert abs(planner.limit_pickup(30., 0.9, 0.8, DT_MDL) - (0.8 - GENTLE_PICKUP_RELEASE_JERK * DT_MDL)) < EPS

  def test_setting_is_reread_about_once_a_second(self):
    planner, sm = build(30., 130.)
    planner.update(sm)
    Params().put_bool("GentleHighwayPickup", True)
    targets(planner, sm, int(1. / DT_MDL) - 1)
    assert not planner.gentle_pickup
    planner.update(sm)
    assert planner.gentle_pickup
