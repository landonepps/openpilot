from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.honda.bosch_c_radar_live import configure
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.honda.tests.test_bosch_c_radar import FAR, SETTLED, objects_bank
from opendbc.car.honda.tests.test_bosch_c_radar_live import params
from openpilot.sunnypilot.selfdrive.car.bosch_c_live_params import update_bosch_c_live_params


def setting(value):
  return SimpleNamespace(get_bool=lambda key: key == "HondaBoschCUncertaintyGate" and value)


def test_gate_follows_setting_while_running():
  p, sp = params()
  configure(p, sp, True)
  ri = RadarInterface(p, sp)
  clock = [0]
  ri.bosch_c.clock = lambda: clock[0]
  ids = []
  for counter, enabled in enumerate((False, True, False)):
    update_bosch_c_live_params(ri, setting(enabled))
    stamp = 1_000_000_000 + counter * 66_000_000
    clock[0] = stamp + 1_000_000
    result = ri.update([(stamp, objects_bank(counter, {0: {'wire': 1, 'x': FAR, 'uncertainty': 40}, 1: SETTLED}))])
    ids.append({pt.trackId for pt in result.points})
  assert ids == [{1, 2}, {2}, {1, 2}]


def test_other_radars_are_untouched():
  p, sp = params()
  ri = RadarInterface(p, sp)
  assert ri.bosch_c is None
  update_bosch_c_live_params(ri, setting(True))
  update_bosch_c_live_params(SimpleNamespace(), setting(True))
  assert not sp.flags and isinstance(sp, structs.CarParamsSP)
