from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.honda.bosch_c_radar import velocity_std
from opendbc.car.honda.bosch_c_radar_live import configure
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.honda.tests.test_bosch_c_radar import FAR, SETTLED, objects_bank
from opendbc.car.honda.tests.test_bosch_c_radar_live import params
from openpilot.sunnypilot.selfdrive.car.radar_tracks_sp import radar_tracks_sp


def test_bosch_c_publishes_each_points_speed_std():
  p, sp = params()
  configure(p, sp, True)
  ri = RadarInterface(p, sp)
  ri.bosch_c.clock = lambda: 1_001_000_000
  RD = ri.update([(1_000_000_000, objects_bank(0, {0: {'wire': 1, 'x': FAR, 'uncertainty': 40}, 1: SETTLED}))])
  msg = radar_tracks_sp(ri, RD, True)
  assert msg.valid
  points = {pt.trackId: pt.vRelStd for pt in msg.radarTracksSP.points}
  assert points.keys() == {pt.trackId for pt in RD.points} == {1, 2}
  assert abs(points[1] - velocity_std(40)) < 1e-6 and abs(points[2] - velocity_std(SETTLED['uncertainty'])) < 1e-6


def test_other_radars_publish_nothing():
  p, sp = params()
  ri = RadarInterface(p, sp)
  assert ri.bosch_c is None
  assert radar_tracks_sp(ri, structs.RadarData.new_message(), True) is None
  assert radar_tracks_sp(SimpleNamespace(), structs.RadarData.new_message(), True) is None
