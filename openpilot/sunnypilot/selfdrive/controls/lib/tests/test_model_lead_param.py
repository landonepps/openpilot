"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import build_planner, build_sm


class TestModelLeadParam(OpenpilotTestCase):
  def _passed_model_leads(self, planner, sm, ticks=1):
    seen = []
    update = planner.mpc.update

    def spy(radarstate, personality, model_leads=None, **kwargs):
      seen.append(model_leads)
      return update(radarstate, personality=personality, model_leads=model_leads, **kwargs)
    planner.mpc.update = spy
    for _ in range(ticks):
      planner.update(sm)
    return seen

  def test_off_by_default(self):
    planner = build_planner(False, 'acc')
    assert all(leads is None for leads in self._passed_model_leads(planner, build_sm(False), ticks=3))

  def test_enabled_passes_model_leads(self):
    Params().put_bool("ModelLeadTrajectory", True)
    planner = build_planner(False, 'acc')
    sm = build_sm(False)
    seen = self._passed_model_leads(planner, sm)
    assert seen[-1] is not None and len(seen[-1]) == len(sm['modelV2'].leadsV3)

  def test_setting_is_reread_about_once_a_second(self):
    planner = build_planner(False, 'acc')
    sm = build_sm(False)
    planner.update(sm)  # reads the setting (off) on its first frame
    Params().put_bool("ModelLeadTrajectory", True)
    ticks = int(1. / DT_MDL)
    seen = self._passed_model_leads(planner, sm, ticks=ticks)
    assert all(leads is None for leads in seen[:-1]) and seen[-1] is not None
