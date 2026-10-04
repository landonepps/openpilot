"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_UNSET, IMPERIAL_INCREMENT
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

# Honda set speeds from route 171 (first engagement of the drive) and route 175 (SET at 32 mph after a 75 mph set speed)
SET_65_MPH = 104.6
SET_75_MPH = 120.7
SET_33_MPH = 53.1
SET_67_MPH = 107.8


def close(v_cruise_kph: float, kph: float) -> bool:
  # the car's set speed is a float32
  return abs(v_cruise_kph - kph) < 0.01


def car_state(set_kph: float, engaged: bool, buttons=()) -> car.CarState:
  CS = car.CarState(cruiseState={"available": True, "enabled": engaged, "speed": set_kph * CV.KPH_TO_MS,
                                 "speedCluster": set_kph * CV.KPH_TO_MS})
  CS.buttonEvents = [ButtonEvent(type=b, pressed=p) for b, p in buttons]
  return CS


class TestICBMEngageSetSpeed(OpenpilotTestCase):
  """With ICBM on, openpilot keeps its own set speed once engaged. It must start from the car's set speed for this
  engagement, which Honda reports 10-60 ms after its cruise engages, and a +/- tap 0.12-0.16 s after its release."""

  def setup_method(self):
    self.helper = VCruiseHelper(car.CarParams(pcmCruise=True), custom.CarParamsSP(pcmCruiseSpeed=False))

  def step(self, set_kph: float, engaged: bool, buttons=(), frames: int = 1) -> float:
    for _ in range(frames):
      self.helper.update_v_cruise(car_state(set_kph, engaged, buttons), enabled=engaged, is_metric=False)
    return self.helper.v_cruise_kph

  def press_set(self, set_kph: float) -> None:
    self.step(set_kph, False, [(ButtonType.decelCruise, True)])
    self.step(set_kph, False, frames=10)
    self.step(set_kph, False, [(ButtonType.decelCruise, False)])
    self.step(set_kph, False, frames=10)

  def test_first_engagement_waits_for_the_car(self):
    self.press_set(0.)
    assert self.step(0., True, frames=3) == V_CRUISE_UNSET
    assert close(self.step(SET_65_MPH, True, frames=100), SET_65_MPH)

  def test_set_after_a_previous_set_speed(self):
    self.press_set(SET_75_MPH)
    self.step(SET_75_MPH, True, frames=5)
    assert close(self.step(SET_33_MPH, True, frames=100), SET_33_MPH)

  def test_tap_during_the_wait(self):
    self.press_set(SET_65_MPH)
    self.step(SET_65_MPH, True, frames=20)
    self.step(SET_65_MPH, True, [(ButtonType.accelCruise, True)])
    self.step(SET_65_MPH, True, frames=7)
    self.step(SET_65_MPH, True, [(ButtonType.accelCruise, False)])
    self.step(SET_65_MPH, True, frames=12)
    assert close(self.step(SET_67_MPH, True, frames=100), SET_67_MPH)

  def test_takes_over_after_settling(self):
    self.press_set(0.)
    self.step(0., True, frames=2)
    self.step(SET_65_MPH, True, frames=100)
    # ICBM's own presses change the car's set speed; openpilot keeps the driver's
    assert close(self.step(SET_75_MPH, True, frames=10), SET_65_MPH)
    # the driver's presses change openpilot's
    self.step(SET_75_MPH, True, [(ButtonType.accelCruise, True)])
    assert close(self.step(SET_75_MPH, True, [(ButtonType.accelCruise, False)]), SET_65_MPH + IMPERIAL_INCREMENT)

  def test_never_takes_over_an_unset_speed(self):
    self.press_set(0.)
    assert self.step(0., True, frames=300) == V_CRUISE_UNSET
    assert close(self.step(SET_65_MPH, True, frames=100), SET_65_MPH)


class TestICBMUnsetSetSpeed(OpenpilotTestCase):
  """ICBM presses nothing while openpilot has no set speed: the planner caps an unset one at V_CRUISE_MAX (90 mph)."""

  def setup_method(self):
    self.icbm = IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True), custom.CarParamsSP(pcmCruiseSpeed=False))
    self.CC = car.CarControl(enabled=True)

  def buttons_sent(self, v_cruise_kph: float, cluster_kph: float, target_kph: float, seconds: float = 1.) -> set:
    CS = car_state(cluster_kph, True)
    CS.vCruise = v_cruise_kph
    LP_SP = custom.LongitudinalPlanSP.new_message(vTarget=target_kph * CV.KPH_TO_MS)
    sent = set()
    for _ in range(int(seconds / DT_CTRL)):
      self.icbm.run(CS, self.CC, LP_SP, is_metric=False)
      sent.add(self.icbm.cruise_button)
    return sent

  def test_unset_set_speed(self):
    assert self.buttons_sent(V_CRUISE_UNSET, SET_65_MPH, 145.) == {SendButtonState.none}

  def test_set_speed_above_the_dash(self):
    assert SendButtonState.increase in self.buttons_sent(SET_75_MPH, SET_65_MPH, SET_75_MPH)
