import math
from typing import cast

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.radard import FAR_TRACK_BLEND_S, FAR_TRACK_FADE_S, RADAR_TO_CAMERA, RadarD

CAPPED_STD = math.exp(11)  # the model's lead xStd and vStd on most confident ticks of recent models
V_EGO = 33.3


class FakeSubMaster(dict):
  def __init__(self):
    super().__init__()
    self.seen = {'modelV2': True}
    self.recv_frame = {'carState': 0}
    self.logMonoTime = {'modelV2': 0}

  def all_checks(self):
    return True


def sm_for(lead_x, lead_v, lead_a=0.):
  sm = FakeSubMaster()
  cs = messaging.new_message('carState')
  cs.carState.vEgo = V_EGO
  sm['carState'] = cs.as_reader().carState
  msg = messaging.new_message('modelV2')
  md = msg.modelV2
  md.velocity.x = [V_EGO] * 33
  leads = md.init('leadsV3', 3)
  for i, lead in enumerate(leads):
    lead.prob = 0.9 if i == 0 else 0.
    lead.x, lead.y, lead.v, lead.a = [lead_x] * 6, [0.] * 6, [lead_v] * 6, [lead_a] * 6
    lead.xStd, lead.yStd, lead.vStd = [CAPPED_STD] * 6, [0.5] * 6, [CAPPED_STD] * 6
  sm['modelV2'] = msg.as_reader().modelV2
  return sm


def leads(enabled, d_rel, v_radar, v_camera, seconds, a_camera=0., gap_at=None):
  """leadOne on each tick while the radar reports one track at d_rel going v_radar and the camera sees the same car going
  v_camera. gap_at: the tick the radar point is missing, which makes radard reseed the track."""
  Params().put_bool("RadarFarTrackSpeedBlend", enabled, block=True)
  rd = RadarD(structs.CarParams(), structs.CarParamsSP())
  sm = sm_for(d_rel + RADAR_TO_CAMERA, v_camera, a_camera)
  point = structs.RadarData.new_message(points=[{'trackId': 1, 'dRel': d_rel, 'yRel': 0., 'vRel': v_radar - V_EGO}]).to_bytes()
  empty = structs.RadarData.new_message(points=[]).to_bytes()
  out = []
  for i in range(round(seconds / DT_MDL)):
    with structs.RadarData.from_bytes(empty if i == gap_at else point) as reader:
      rd.update(cast(messaging.SubMaster, sm), reader)
    out.append(rd.radar_state.leadOne)
  return out


def at(lead_list, seconds):
  return lead_list[round(seconds / DT_MDL) - 1]


class TestFarTrackSpeedBlend(OpenpilotTestCase):
  def test_far_new_track_averages_the_two_speeds_then_fades_to_the_radar(self):
    # route 1db at 2023.65 s: the radar's new track 90 m ahead read 25 m/s, the camera about 32.5, the lead did about 31
    out = leads(True, 90., 25., 32.5, 3.)
    for lead in out[:round(FAR_TRACK_BLEND_S / DT_MDL) - 1]:
      assert lead.radar and abs(lead.vLead - 28.75) < 1e-3 and abs(lead.vLeadK - 28.75) < 0.1
      assert abs(lead.vRel - (28.75 - V_EGO)) < 1e-3 and abs(lead.dRel - 90.) < 1e-3
    mid = at(out, (FAR_TRACK_BLEND_S + FAR_TRACK_FADE_S) / 2)
    assert 25. < mid.vLead < 28.75
    assert abs(at(out, FAR_TRACK_FADE_S + DT_MDL).vLead - 25.) < 1e-3

  def test_off_by_default(self):
    assert all(abs(lead.vLead - 25.) < 1e-3 for lead in leads(False, 90., 25., 32.5, 1.))

  def test_near_new_track_unchanged(self):
    assert all(abs(lead.vLead - 25.) < 1e-3 for lead in leads(True, 40., 25., 32.5, 1.))

  def test_slow_or_stopped_traffic_keeps_the_radar_speed(self):
    assert all(abs(lead.vLead - 15.) < 1e-3 for lead in leads(True, 120., 15., 25., 1.))  # under half the ego's speed
    assert all(abs(lead.vLead - 22.) < 1e-3 for lead in leads(True, 120., 22., 28., 1.))  # closing at 11.3 m/s

  def test_acceleration_fades_in_and_camera_braking_passes_through(self):
    assert all(lead.aLeadK == 0. for lead in leads(True, 90., 25., 32.5, 1.))
    assert all(abs(lead.aLeadK + 1.2) < 1e-3 for lead in leads(True, 90., 25., 32.5, 1., a_camera=-1.2))

  def test_a_reseeded_track_is_blended_again(self):
    # the uncertainty gate drops a far track for a frame and republishes it, and radard starts it as a new track
    out = leads(True, 90., 35., 30., 4., gap_at=round(3. / DT_MDL))
    assert abs(at(out, 2.9).vLead - 35.) < 1e-3
    assert abs(at(out, 3.2).vLead - 32.5) < 1e-3

  def test_setting_is_off_by_default_and_reread_about_once_a_second(self):
    rd = RadarD(structs.CarParams(), structs.CarParamsSP())
    rd.read_params()
    assert not rd.far_blend
    Params().put_bool("RadarFarTrackSpeedBlend", True, block=True)
    rd.frame = 1
    rd.read_params()
    assert not rd.far_blend
    rd.frame = int(1. / DT_MDL)
    rd.read_params()
    assert rd.far_blend
