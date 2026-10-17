from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.radard import NEW_TRACK_HOLD_S, KalmanParams, RadarD, Track

HOLD_CNT = round(NEW_TRACK_HOLD_S / DT_MDL)
# a new track's reported lead speed settling over its first ~0.35 s at constant true speed, as on route 19d at 659.3 s
BIRTH = [15.1] * 3 + [16.5, 18.0, 19.5, 20.4, 20.7] + [20.7] * 30


def track_outputs(hold_cnt):
  track = Track(1, BIRTH[0], KalmanParams(DT_MDL), hold_cnt)
  out = []
  for v_lead in BIRTH:
    track.update(100., 0., v_lead - 25., v_lead)
    out.append((track.vLeadK, track.aLeadK, track.aLeadTau.x))
  return out


class TestNewTrackHold(OpenpilotTestCase):
  def test_filter_seeded_at_birth_reads_settling_as_acceleration(self):
    assert max(a for _, a, _ in track_outputs(0)) > 4.

  def test_hold_publishes_measured_speed_without_acceleration(self):
    out = track_outputs(HOLD_CNT)
    for (v_k, a_k, tau), v_lead in zip(out[:HOLD_CNT], BIRTH, strict=False):
      assert v_k == v_lead and a_k == 0. and tau == 1.5
    # the filter starts from the last held measurement, which has settled here
    assert max(abs(a) for _, a, _ in out) < 0.5

  def test_setting_is_off_by_default_and_reread_about_once_a_second(self):
    rd = RadarD(structs.CarParams(), structs.CarParamsSP())
    rd.read_params()
    assert rd.new_track_hold_cnt == 0
    Params().put_bool("RadarNewTrackHold", True, block=True)
    rd.frame = 1
    rd.read_params()
    assert rd.new_track_hold_cnt == 0
    rd.frame = int(1. / DT_MDL)
    rd.read_params()
    assert rd.new_track_hold_cnt == HOLD_CNT
