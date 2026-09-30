import numpy as np
from collections.abc import Callable
from typing import cast

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.radard import FAINT_LEAD_SPAN_S, RadarD

V_EGO = 15.
ONE_SECOND = int(round(FAINT_LEAD_SPAN_S / DT_MDL))


class FakeSubMaster(dict):
  def __init__(self):
    super().__init__()
    self.seen = {'modelV2': True}
    self.recv_frame = {'carState': 0}
    self.logMonoTime = {'modelV2': 0}

  def all_checks(self):
    return True


def model(prob, lead_x):
  msg = messaging.new_message('modelV2')
  md = msg.modelV2
  md.position.x = np.linspace(0., 150., 33).tolist()
  md.position.y = [0.] * 33
  md.velocity.x = [V_EGO] * 33
  leads = md.init('leadsV3', 3)
  for i, lead in enumerate(leads):
    lead.prob = prob if i == 0 else 0.
    lead.x, lead.y, lead.v, lead.a = [lead_x] * 6, [0.] * 6, [0.] * 6, [0.] * 6
    lead.xStd, lead.yStd, lead.vStd = [2.] * 6, [0.5] * 6, [1.] * 6
  return msg.as_reader().modelV2


def drive(ticks, prob: float | Callable[[int], float] = 0.4, d0=60., y_rel=0., v_rel=None, d_rel=None, enabled=True):
  """A car ahead, closing at the ego's speed unless v_rel/d_rel (functions of the tick) say otherwise. leadOne per tick."""
  Params().put_bool("RadarConfirmedFaintLead", enabled, block=True)
  rd = RadarD(structs.CarParams(), structs.CarParamsSP())
  sm = FakeSubMaster()
  cs = messaging.new_message('carState')
  cs.carState.vEgo = V_EGO
  sm['carState'] = cs.as_reader().carState
  v_rel = v_rel or (lambda k: -V_EGO)
  d_rel = d_rel or (lambda k: d0 - V_EGO * DT_MDL * k)
  leads = []
  for k in range(ticks):
    p = prob if isinstance(prob, (int, float)) else prob(k)
    sm['modelV2'] = model(p, d_rel(k) + 1.52)
    rr = structs.RadarData.new_message(points=[{'trackId': 1, 'dRel': d_rel(k), 'yRel': y_rel, 'vRel': v_rel(k)}])
    with structs.RadarData.from_bytes(rr.to_bytes()) as reader:
      rd.update(cast(messaging.SubMaster, sm), reader)
    leads.append(rd.radar_state.leadOne)
  return leads


class TestFaintLead(OpenpilotTestCase):
  def test_confirmed_faint_lead_after_a_second_on_the_path(self):
    leads = drive(ONE_SECOND + 10)
    assert not any(lead.present for lead in leads[:ONE_SECOND - 1])
    taken = [lead for lead in leads if lead.present]
    assert len(taken) >= 10 and all(lead.radar and lead.radarTrackId == 1 for lead in taken)
    # at constant speed: the acceleration of a lead the camera isn't confident of isn't confirmed either
    assert all(lead.aLeadK == 0. and lead.aLeadTau == 1.5 and abs(lead.modelProb - 0.4) < 1e-6 for lead in taken)

  def test_off_by_default(self):
    rd = RadarD(structs.CarParams(), structs.CarParamsSP())
    rd.read_params()
    assert not rd.faint_lead
    assert not any(lead.present for lead in drive(ONE_SECOND + 10, enabled=False))

  def test_not_below_the_floor_or_off_the_path(self):
    assert not any(lead.present for lead in drive(ONE_SECOND + 10, prob=0.25))
    assert not any(lead.present for lead in drive(ONE_SECOND + 10, y_rel=2.5))

  def test_not_a_closing_speed_the_range_doesnt_show(self):
    # a bridge return blended into the track: vRel says closing at 10 m/s while the range holds (each reading a little
    # different, so each is a fresh one)
    assert not any(lead.present for lead in drive(ONE_SECOND + 10, d_rel=lambda k: 60. + 0.01 * (k % 2), v_rel=lambda k: -10.))

  def test_not_a_lead_slowing_faster_than_1_g(self):
    # range and Doppler agree on an object whose speed falls 13 m/s in a second
    v_rel = lambda k: -5. - 13. * DT_MDL * k  # noqa: E731
    d_rel = lambda k: 120. - sum(-v_rel(i) * DT_MDL for i in range(k))  # noqa: E731
    assert not any(lead.present for lead in drive(ONE_SECOND + 10, d_rel=d_rel, v_rel=v_rel))

  def test_held_down_to_the_hold_probability(self):
    prob = lambda k: 0.4 if k < ONE_SECOND + 5 else (0.25 if k < ONE_SECOND + 15 else 0.1)  # noqa: E731
    leads = drive(ONE_SECOND + 25, prob=prob)
    assert all(lead.present for lead in leads[ONE_SECOND + 1:ONE_SECOND + 15])
    assert not any(lead.present for lead in leads[ONE_SECOND + 20:])

  def test_setting_is_reread_about_once_a_second(self):
    Params().put_bool("RadarConfirmedFaintLead", False, block=True)
    rd = RadarD(structs.CarParams(), structs.CarParamsSP())
    rd.read_params()
    Params().put_bool("RadarConfirmedFaintLead", True, block=True)
    rd.frame = 1
    rd.read_params()
    assert not rd.faint_lead
    rd.frame = int(1. / DT_MDL)
    rd.read_params()
    assert rd.faint_lead
