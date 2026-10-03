"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_gentle_pickup import build

Personality = log.LongitudinalPersonality


def run(personality) -> tuple[float, list]:
  """The following time the MPC planned with and the personalities the cost weights were set for."""
  planner, sm = build(30., 130.)
  selfdrive_state = messaging.new_message('selfdriveState')
  selfdrive_state.selfdriveState.enabled = True
  selfdrive_state.selfdriveState.personality = personality
  sm['selfdriveState'] = selfdrive_state.selfdriveState.as_reader()
  weights = []
  set_weights = planner.mpc.set_weights

  def spy(prev_accel_constraint=True, personality=Personality.standard):
    weights.append(personality)
    return set_weights(prev_accel_constraint, personality=personality)
  planner.mpc.set_weights = spy
  planner.update(sm)
  return float(planner.mpc.params[0, 4]), weights


class TestPersonalityGapOnly(OpenpilotTestCase):
  def test_off_by_default(self):
    for personality in (Personality.relaxed, Personality.standard, Personality.aggressive):
      assert run(personality) == (get_T_FOLLOW(personality), [personality])

  def test_personalities_set_only_the_gap(self):
    Params().put_bool("PersonalityGapOnly", True)
    gaps = {personality: run(personality) for personality in (Personality.relaxed, Personality.standard, Personality.aggressive)}
    assert gaps[Personality.relaxed] == (get_T_FOLLOW(Personality.relaxed), [Personality.standard])
    assert gaps[Personality.standard] == (get_T_FOLLOW(Personality.aggressive), [Personality.standard])
    # stock ACC's gap: about 1.0 s of distance per speed at highway speed, counting the MPC's 6 m standstill distance
    t_follow, weights = gaps[Personality.aggressive]
    assert weights == [Personality.standard]
    assert abs((t_follow * 29. + 6.) / 29. - 1.0) < 0.02
