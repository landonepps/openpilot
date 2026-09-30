import numpy as np

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (LEAD_T_IDXS_MODEL, MIN_X_LEAD_FACTOR, T_DIFFS, T_IDXS,
                                                                            LongitudinalMpc, model_lead_trajectory)


def radar_lead(d_rel=40., v_lead=25., a_lead=0., model_prob=0.9):
  lead = log.RadarState.LeadData.new_message()
  lead.present = True
  lead.dRel, lead.vLead, lead.aLeadK, lead.aLeadTau, lead.modelProb = d_rel, v_lead, a_lead, 1.5, model_prob
  return lead.as_reader()


def model_lead(v=(26., 25., 24., 24., 24., 24.), a0=-0.5, prob=0.9):
  lead = log.ModelDataV2.LeadDataV3.new_message()
  lead.prob = prob
  lead.x = [float(40. + 25. * t) for t in LEAD_T_IDXS_MODEL]
  lead.v = list(v)
  lead.a = [a0] + [0.] * (len(LEAD_T_IDXS_MODEL) - 1)
  return lead.as_reader()


def model_only_speeds(v_lead, ml):
  return np.clip(v_lead + np.interp(T_IDXS, LEAD_T_IDXS_MODEL, np.array(ml.v) - ml.v[0]), 0., 1e8)


class TestModelLeadTrajectory(OpenpilotTestCase):
  def test_speed_change_comes_from_model(self):
    ml = model_lead(a0=-0.5)
    xv = model_lead_trajectory(radar_lead(a_lead=-0.5), ml, 25.)
    np.testing.assert_allclose(xv[:, 1], model_only_speeds(25., ml))
    np.testing.assert_allclose(xv[:, 0], 40. + np.cumsum(T_DIFFS * xv[:, 1]))
    assert xv[0, 0] == 40. and xv[0, 1] == 25.

  def test_radar_braking_beyond_model_is_added_and_decays(self):
    ml = model_lead(a0=-0.5)
    xv = model_lead_trajectory(radar_lead(a_lead=-3.0), ml, 25.)
    extra = xv[:, 1] - model_only_speeds(25., ml)
    expected = np.cumsum(T_DIFFS * -2.5 * np.exp(-_LEAD_ACCEL_TAU * T_IDXS**2 / 2.))
    np.testing.assert_allclose(extra, expected)
    assert extra[0] == 0. and np.all(np.diff(extra) <= 0.)
    # the residual fades: about -2 m/s over the horizon, not -2.5 m/s^2 sustained (-25 m/s)
    assert -2.2 < extra[-1] < -1.8

  def test_radar_acceleration_beyond_model_is_ignored(self):
    ml = model_lead(a0=0.)
    xv = model_lead_trajectory(radar_lead(a_lead=2.0), ml, 25.)
    np.testing.assert_allclose(xv[:, 1], model_only_speeds(25., ml))

  def test_unusable_model_prediction_returns_none(self):
    assert model_lead_trajectory(radar_lead(), model_lead(prob=0.4), 25.) is None
    # radard's low-speed override publishes radar tracks without a model match (modelProb 0)
    assert model_lead_trajectory(radar_lead(model_prob=0.), model_lead(), 25.) is None
    short = log.ModelDataV2.LeadDataV3.new_message(prob=0.9, v=[25., 25.], a=[0., 0.])
    assert model_lead_trajectory(radar_lead(), short.as_reader(), 25.) is None

  def test_close_lead_is_lifted_to_brakeable_distance(self):
    xv = model_lead_trajectory(radar_lead(d_rel=5., v_lead=10.), model_lead(), 25.)
    min_x = MIN_X_LEAD_FACTOR * (25. + 10.) * (25. - 10.) / (-ACCEL_MIN * 2)
    assert xv[0, 0] == min_x

  def test_process_lead_falls_back_without_model(self):
    mpc = LongitudinalMpc()
    mpc.x0[1] = 25.
    lead = radar_lead(a_lead=-1.0)
    legacy = mpc.process_lead(lead)
    np.testing.assert_array_equal(mpc.process_lead(lead, None), legacy)
    np.testing.assert_array_equal(mpc.process_lead(lead, model_lead(prob=0.2)), legacy)
    assert not np.array_equal(mpc.process_lead(lead, model_lead()), legacy)
