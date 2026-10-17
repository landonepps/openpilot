import numpy as np

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.radard import NEW_TRACK_HOLD_S, V_REL_STD_NOMINAL, KalmanParams, RadarD, Track

KP = KalmanParams(DT_MDL)
HOLD_CNT = round(NEW_TRACK_HOLD_S / DT_MDL)
# a new track's reported lead speed settling over its first ~0.35 s at constant true speed, as on route 19d at 659.3 s
BIRTH = [15.1] * 3 + [16.5, 18.0, 19.5, 20.4, 20.7] + [20.7] * 30
NOISY = list(20. + np.cumsum(np.random.default_rng(1).normal(0., 0.3, 400)))


def track_outputs(series, v_rel_std, weighted=True, hold_cnt=0):
  track = Track(1, series[0], KP, hold_cnt, weighted)
  out = []
  for v_lead in series:
    track.update(100., 0., v_lead - 25., v_lead, v_rel_std)
    out.append((track.vLeadK, track.aLeadK))
  return np.array(out)


class TestUncertaintyFilter(OpenpilotTestCase):
  def test_fixed_gain_is_the_steady_state_at_nominal_noise(self):
    (p00, p01), _ = KP.P
    s = p00 + KP.R
    np.testing.assert_allclose([(p00 + DT_MDL * p01) / s, p01 / s], [KP.K[0][0], KP.K[1][0]], rtol=1e-7)

  def test_nominal_or_unreported_noise_filters_as_today(self):
    fixed = track_outputs(NOISY, None, weighted=False)
    for v_rel_std in (None, 0., V_REL_STD_NOMINAL):
      np.testing.assert_allclose(track_outputs(NOISY, v_rel_std), fixed, atol=1e-6)

  def test_noisy_readings_count_less(self):
    assert track_outputs(BIRTH, None)[:, 1].max() > 4.
    # a Bosch C track reading velocity uncertainty 100 (about 15 m/s)
    assert track_outputs(BIRTH, 15.)[:, 1].max() < 0.5
    noisy_a = [np.abs(track_outputs(NOISY, std)[:, 1]).max() for std in (V_REL_STD_NOMINAL, 1., 3.)]
    assert noisy_a[0] > noisy_a[1] > noisy_a[2]

  def test_new_track_hold_then_weighted_filter(self):
    out = track_outputs(BIRTH, 15., hold_cnt=HOLD_CNT)
    np.testing.assert_array_equal(out[:HOLD_CNT], np.array([BIRTH[:HOLD_CNT], [0.] * HOLD_CNT]).T)
    assert np.abs(out[:, 1]).max() < 0.1

  def test_setting_is_off_by_default_and_reread_about_once_a_second(self):
    rd = RadarD(structs.CarParams(), structs.CarParamsSP())
    rd.read_params()
    assert not rd.uncertainty_filter
    Params().put_bool("RadarUncertaintyFilter", True, block=True)
    rd.frame = 1
    rd.read_params()
    assert not rd.uncertainty_filter
    rd.frame = int(1. / DT_MDL)
    rd.read_params()
    assert rd.uncertainty_filter
