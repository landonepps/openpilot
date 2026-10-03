import math
from typing import cast

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.radard import MATCH_LATERAL_M, RADAR_TO_CAMERA, RadarD

CAPPED_STD = math.exp(11)  # the model's lead xStd and vStd on most confident ticks of recent models


class FakeSubMaster(dict):
  def __init__(self):
    super().__init__()
    self.seen = {'modelV2': True}
    self.recv_frame = {'carState': 0}
    self.logMonoTime = {'modelV2': 0}

  def all_checks(self):
    return True


def lead_one(enabled, v_ego, lead_x, lead_y, lead_v, points, ticks=3):
  """leadOne after a few ticks of a camera lead at (lead_x, lead_y) going lead_v, with the model's capped stds, and the
  radar points (trackId, dRel, yRel, vRel)."""
  Params().put_bool("RadarLateralMatch", enabled, block=True)
  rd = RadarD(structs.CarParams(), structs.CarParamsSP())
  sm = FakeSubMaster()
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego
  sm['carState'] = cs.as_reader().carState
  msg = messaging.new_message('modelV2')
  md = msg.modelV2
  md.velocity.x = [v_ego] * 33
  leads = md.init('leadsV3', 3)
  for i, lead in enumerate(leads):
    lead.prob = 0.9 if i == 0 else 0.
    lead.x, lead.y, lead.v, lead.a = [lead_x] * 6, [lead_y] * 6, [lead_v] * 6, [0.] * 6
    lead.xStd, lead.yStd, lead.vStd = [CAPPED_STD] * 6, [0.5] * 6, [CAPPED_STD] * 6
  sm['modelV2'] = msg.as_reader().modelV2
  rr = structs.RadarData.new_message(points=[{'trackId': i, 'dRel': d, 'yRel': y, 'vRel': v} for i, d, y, v in points]).to_bytes()
  for _ in range(ticks):
    with structs.RadarData.from_bytes(rr) as reader:
      rd.update(cast(messaging.SubMaster, sm), reader)
  return rd.radar_state.leadOne


class TestLateralMatch(OpenpilotTestCase):
  def test_next_lane_car_is_not_the_lead(self):
    # route 1c0 at 602 s: the radar has lost the truck the camera sees 54 m ahead in the lane, and a car in the next lane
    # 4 m to the side at 44 m passes the range and speed checks
    args = (30., 54. + RADAR_TO_CAMERA, 0., 27., [(1, 44., 3.5, 0.)])
    lead = lead_one(False, *args)
    assert lead.present and lead.radar and lead.radarTrackId == 1
    lead = lead_one(True, *args)
    assert lead.present and not lead.radar and abs(lead.dRel - 54.) < 1e-3

  def test_far_in_path_truck_does_not_hide_the_lead(self):
    # route 171 stopped in traffic: the lead 7 m ahead and 1.2 m to the side (outside the low-speed override's 1 m), and a
    # truck 40 m out nearer the camera lead's lateral position, which wins the match and fails the range check
    args = (0., 7. + RADAR_TO_CAMERA, -1.0, 0., [(1, 7., 1.2, 0.), (2, 40., 0.9, 0.)])
    lead = lead_one(False, *args)
    assert lead.present and not lead.radar
    lead = lead_one(True, *args)
    assert lead.present and lead.radar and lead.radarTrackId == 1

  def test_limit_is_measured_from_the_camera_lead(self):
    # the model's y is positive to the right and yRel to the left: a camera lead 2 m to the right matches a track 2 m to
    # the right, not one 2 m to the left
    lead = lead_one(True, 30., 50. + RADAR_TO_CAMERA, 2., 30., [(1, 50., -2., 0.)])
    assert lead.radar and lead.radarTrackId == 1
    lead = lead_one(True, 30., 50. + RADAR_TO_CAMERA, 2., 30., [(1, 50., 2., 0.)])
    assert lead.present and not lead.radar
    lead = lead_one(True, 30., 50. + RADAR_TO_CAMERA, 0., 30., [(1, 50., MATCH_LATERAL_M - 0.1, 0.)])
    assert lead.radar and lead.radarTrackId == 1

  def test_setting_is_off_by_default_and_reread_about_once_a_second(self):
    rd = RadarD(structs.CarParams(), structs.CarParamsSP())
    rd.read_params()
    assert rd.match_lateral is None
    Params().put_bool("RadarLateralMatch", True, block=True)
    rd.frame = 1
    rd.read_params()
    assert rd.match_lateral is None
    rd.frame = int(1. / DT_MDL)
    rd.read_params()
    assert rd.match_lateral == MATCH_LATERAL_M
