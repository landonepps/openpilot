"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from opendbc.car import structs


def radar_tracks_sp(RI, RD: structs.RadarDataT, valid: bool):
  """radarTracksSP for the points in RD, or None when the radar reports no per-track extras.

  Only the Bosch C radar does: each track's speed uncertainty, which radard's RadarUncertaintyFilter uses.
  """
  bosch_c = getattr(RI, 'bosch_c', None)
  if bosch_c is None:
    return None
  stds = bosch_c.velocity_stds()
  msg = messaging.new_message('radarTracksSP', valid=valid)
  points = msg.radarTracksSP.init('points', len(RD.points))
  for point, pt in zip(points, RD.points, strict=True):
    point.trackId = pt.trackId
    point.vRelStd = stds.get(pt.trackId, 0.)
  return msg
