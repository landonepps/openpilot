#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log, custom
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

# RadarNewTrackHold: a new track's reported speed can take a few sweeps to settle. On the Bosch C radar a new far track
# can read 5-6 m/s off for its first ~0.3 s while its range stays flat, and a filter seeded from it reports that as
# several m/s^2 of lead acceleration when the track becomes the lead. With the setting on, a track publishes its
# measured speed with zero acceleration for this long, and the filter starts from there.
NEW_TRACK_HOLD_S = 0.5

# RadarUncertaintyFilter: where the radar reports a speed uncertainty for each reading (radarTracksSP; only the Bosch C
# radar does), each track's filter weights the reading by it. Tracks whose readings stay at or below this noise keep
# the fixed-gain behavior. On Bosch C those are settled tracks (velocity uncertainty 6 or less); new and far tracks can
# read several times higher.
V_REL_STD_NOMINAL = 0.37  # m/s

# RadarConfirmedFaintLead: at speed radard takes a lead only above 0.5 camera lead probability, so a car the radar has
# before the camera is confident of doesn't reach the planner. With the setting on, a camera lead down to
# FAINT_LEAD_PROB is taken when its radar match confirms it: over the last FAINT_LEAD_SPAN_S of fresh readings it stayed
# on the model's path, closed at FAINT_LEAD_MIN_CLOSING or more, its range confirmed its speed and its speed changed no
# faster than 1 g. It goes to the planner at constant speed; its acceleration isn't confirmed. Once taken it is kept
# above FAINT_LEAD_HOLD_PROB while the path and range checks hold. On Bosch C replays it takes a stopped car in town a
# few tenths of a second before the camera does, and it holds almost every car it takes (bosch-c-research
# docs/radar-only-leads.md).
FAINT_LEAD_PROB, FAINT_LEAD_HOLD_PROB = 0.3, 0.2
FAINT_LEAD_SPAN_S, FAINT_LEAD_MIN_READINGS = 1.0, 10
FAINT_LEAD_MAX_GAP_S = 0.15  # a tick this long after the last fresh reading restarts the run on the path
# every reading within FAINT_LEAD_PATH_WIDTH_M of the path, the median within FAINT_LEAD_MEDIAN_WIDTH_M and
# FAINT_LEAD_CORE_FRAC of them within FAINT_LEAD_CORE_WIDTH_M: the model's path far out on a curve is a few tenths off
FAINT_LEAD_PATH_WIDTH_M, FAINT_LEAD_MEDIAN_WIDTH_M, FAINT_LEAD_CORE_WIDTH_M, FAINT_LEAD_CORE_FRAC = 2.0, 0.8, 1.0, 0.7
FAINT_LEAD_MIN_CLOSING = 2.0  # m/s
# the range fits a line within FAINT_LEAD_MAX_RANGE_RMS_M whose slope closes no more than max(FAINT_LEAD_RATE_TOL,
# FAINT_LEAD_RATE_TOL_FRAC |vRel|) slower than the mean or the newest vRel. One-sided: the phantoms it is for report
# closing speeds the range doesn't show, and a fast closer's range slope trails its vRel by a few m/s.
FAINT_LEAD_MAX_RANGE_RMS_M, FAINT_LEAD_RATE_TOL, FAINT_LEAD_RATE_TOL_FRAC = 1.0, 2.5, 0.3
FAINT_LEAD_MAX_ACCEL = 10.  # m/s^2
FAINT_LEAD_MAX_D_REL = 120.  # m

# RadarLateralMatch: radard matches the camera lead to its most probable track and only then checks that track's range
# and speed. The probability is scaled by the model lead's stds, and recent models report xStd and vStd at their cap on
# most confident ticks, so the most probable track is the one laterally closest to the camera lead, at any range: a car
# in the next lane at a range that passes, or a far in-path truck that fails the range check while the real lead is
# close. With the setting on, radard takes the most probable of the tracks that pass the range and speed checks and are
# within MATCH_LATERAL_M of the camera lead's lateral position, and uses the camera-only lead if there are none. On
# Bosch C, matches within 60 m thin out to a minimum 2.5-3.5 m from the camera lead and rise again at the next lane.
MATCH_LATERAL_M = 2.5

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]

    # K is the steady-state gain of the predictor-form filter with these noises (at the tabulated dts), and P its
    # predicted covariance, where WeightedKF1D starts
    self.Q = [[10., 0.], [0., 100.]]
    self.R = 1e3
    p00 = p01 = p11 = 0.
    for _ in range(1000):
      s = p00 + self.R
      k0, k1 = (p00 + dt * p01) / s, p01 / s
      p00, p01, p11 = (p00 + 2 * dt * p01 + dt * dt * p11 + self.Q[0][0] - k0 * k0 * s,
                       p01 + dt * p11 + self.Q[0][1] - k0 * k1 * s,
                       p11 + self.Q[1][1] - k1 * k1 * s)
    self.P = [[p00, p01], [p01, p11]]


class WeightedKF1D(KF1D):
  """KF1D in its time-varying form: each reading's measurement noise follows the speed uncertainty the radar reports
  for it (RadarUncertaintyFilter). KF1D's fixed gain is this filter's steady state at the nominal noise. Readings at or
  below V_REL_STD_NOMINAL, or without a reported uncertainty, use that nominal noise; noisier readings count less."""
  def __init__(self, x0, kalman_params: KalmanParams):
    super().__init__(x0, kalman_params.A, kalman_params.C, kalman_params.K)
    self.dt = kalman_params.A[0][1]
    self.Q, self.R = kalman_params.Q, kalman_params.R
    (self.p00, self.p01), (_, self.p11) = kalman_params.P

  def update(self, meas, v_std: float | None = None):
    r = self.R * max((v_std or 0.) / V_REL_STD_NOMINAL, 1.) ** 2
    dt, p00, p01, p11 = self.dt, self.p00, self.p01, self.p11
    s = p00 + r
    k0, k1 = (p00 + dt * p01) / s, p01 / s
    innovation = meas - self.x0_0
    self.x0_0, self.x1_0 = self.x0_0 + dt * self.x1_0 + k0 * innovation, self.x1_0 + k1 * innovation
    self.p00 = p00 + 2 * dt * p01 + dt * dt * p11 + self.Q[0][0] - k0 * k0 * s
    self.p01 = p01 + dt * p11 + self.Q[0][1] - k0 * k1 * s
    self.p11 = p11 + self.Q[1][1] - k1 * k1 * s
    return [self.x0_0, self.x1_0]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams, hold_cnt: int = 0, weighted: bool = False):
    self.identifier = identifier
    self.cnt = 0
    self.hold_cnt = hold_cnt  # updates that publish zero acceleration before the filter starts (NEW_TRACK_HOLD_S)
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    if weighted:  # RadarUncertaintyFilter
      self.kf = WeightedKF1D([[v_lead], [0.0]], kalman_params)
    else:
      self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    # RadarConfirmedFaintLead: this run of fresh readings on the model's path, as (t, dRel, vRel, path offset, vLead)
    self.path_readings: deque[tuple[float, float, float, float, float]] = deque(maxlen=64)
    self.last_reading: tuple[float, float, float] | None = None
    self.last_reading_t = 0.0

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, v_rel_std: float | None = None):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead

    if self.cnt < self.hold_cnt:
      # new track: its speed may still be settling, so no acceleration yet; the filter starts from the last of these
      self.kf.set_x([[self.vLead], [0.0]])
      self.vLeadK = float(self.vLead)
      self.aLeadK = 0.0
      self.aLeadTau.x = _LEAD_ACCEL_TAU
      self.cnt += 1
      return

    # computed velocity and accelerations
    if self.cnt > 0:
      if isinstance(self.kf, WeightedKF1D):
        self.kf.update(self.vLead, v_rel_std)
      else:
        self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def update_path(self, t: float, path_x: np.ndarray, path_y: np.ndarray):
    """RadarConfirmedFaintLead: add this tick's reading to the run on the path if it is a new one."""
    reading = (self.dRel, self.yRel, self.vRel)
    if reading == self.last_reading:
      if t - self.last_reading_t > FAINT_LEAD_MAX_GAP_S + 1e-6:
        self.path_readings.clear()
      return
    self.last_reading, self.last_reading_t = reading, t
    offset = math.nan
    if len(path_x) and 1. < self.dRel <= path_x[-1]:
      offset = self.yRel + float(np.interp(self.dRel, path_x, path_y))  # yRel is positive left, the path's y positive right
    if abs(offset) <= FAINT_LEAD_PATH_WIDTH_M:
      self.path_readings.append((t, self.dRel, self.vRel, offset, self.vLead))
    else:
      self.path_readings.clear()

  def confirms_faint_lead(self, held: bool) -> bool:
    """RadarConfirmedFaintLead: whether this track's readings confirm it as the lead. A held lead needs only the range
    and acceleration checks on its unbroken run."""
    if not self.path_readings or not self.dRel <= FAINT_LEAD_MAX_D_REL:
      return False
    t_last = self.path_readings[-1][0]
    window = [r for r in self.path_readings if r[0] >= t_last - FAINT_LEAD_SPAN_S - 1e-6]
    if len(window) < FAINT_LEAD_MIN_READINGS or self.path_readings[0][0] > t_last - FAINT_LEAD_SPAN_S + 1e-6:
      return False
    ts = np.array([r[0] for r in window]) - t_last
    d_rels = np.array([r[1] for r in window])
    v_mean, v_last = float(np.mean([r[2] for r in window])), window[-1][2]
    slope, intercept = np.polyfit(ts, d_rels, 1)
    rms = float(np.sqrt(np.mean((d_rels - (slope * ts + intercept)) ** 2)))
    if rms > FAINT_LEAD_MAX_RANGE_RMS_M or any(slope - v > max(FAINT_LEAD_RATE_TOL, FAINT_LEAD_RATE_TOL_FRAC * abs(v)) for v in (v_mean, v_last)):
      return False
    if abs(np.polyfit(ts, np.array([r[4] for r in window]), 1)[0]) > FAINT_LEAD_MAX_ACCEL:
      return False
    if held:
      return True
    offsets = np.abs(np.array([r[3] for r in window]))
    if np.median(offsets) > FAINT_LEAD_MEDIAN_WIDTH_M or np.mean(offsets <= FAINT_LEAD_CORE_WIDTH_M) < FAINT_LEAD_CORE_FRAC:
      return False
    return v_mean <= -FAINT_LEAD_MIN_CLOSING

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track],
                          max_lateral: float | None = None):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  def sane(c):
    # stationary radar points can be false positives
    dist_sane = abs(c.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
    vel_sane = (abs(c.vRel + v_ego - lead.v[0]) < 10) or (v_ego + c.vRel > 3)
    return dist_sane and vel_sane

  if max_lateral is not None:  # RadarLateralMatch
    candidates = [c for c in tracks.values() if sane(c) and abs(c.yRel + lead.y[0]) <= max_lateral]
    return max(candidates, key=prob) if candidates else None

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  if sane(track):
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, lead_prob: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
             low_speed_override: bool = True, max_lateral: float | None = None) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks, max_lateral=max_lateral)
  else:
    track = None

  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any],
                    lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    self.CP = CP
    self.CP_SP = CP_SP

    self.current_time = 0.0
    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    self.params = Params()
    self.frame = 0
    self.new_track_hold_cnt = 0
    self.uncertainty_filter = False
    self.faint_lead = False
    self.match_lateral: float | None = None  # RadarLateralMatch
    self.faint_lead_track: int | None = None  # the track held as a faint lead last tick
    self.faint_lead_paths_stale = False  # runs on the path stop while the setting is off

  def read_params(self) -> None:
    # about once a second. The new-track hold and the uncertainty filter apply to tracks created after a change; the faint
    # lead and the lateral match check apply at once
    if self.frame % int(1. / DT_MDL) == 0:
      self.new_track_hold_cnt = round(NEW_TRACK_HOLD_S / DT_MDL) if self.params.get_bool("RadarNewTrackHold") else 0
      self.uncertainty_filter = self.params.get_bool("RadarUncertaintyFilter")
      self.faint_lead = self.params.get_bool("RadarConfirmedFaintLead")
      self.match_lateral = MATCH_LATERAL_M if self.params.get_bool("RadarLateralMatch") else None

  def update(self, sm: messaging.SubMaster, rr: car.RadarData, rr_sp: capnp._DynamicStructReader | None = None):
    self.read_params()
    self.frame += 1
    self.ready = sm.seen['modelV2']

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}
    # each reading's speed uncertainty, where the radar reports one
    v_rel_stds = {pt.trackId: pt.vRelStd for pt in rr_sp.points} if rr_sp is not None else {}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params, self.new_track_hold_cnt, self.uncertainty_filter)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, v_rel_stds.get(ids))

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, self.lead_prob_filters[0].x,
                          self.CP, self.CP_SP, low_speed_override=True, max_lateral=self.match_lateral)
      if self.faint_lead:
        position = sm['modelV2'].position
        path_x, path_y = np.asarray(position.x), np.asarray(position.y)
        for track in self.tracks.values():
          if self.faint_lead_paths_stale:
            track.path_readings.clear()
          track.update_path(self.frame * DT_MDL, path_x, path_y)
        self.faint_lead_paths_stale = False
        lead_one = self.get_faint_lead(lead_one, leads_v3[0])
      else:
        self.faint_lead_track = None
        self.faint_lead_paths_stale = True
      self.radar_state.leadOne = lead_one
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, self.lead_prob_filters[1].x,
                                          self.CP, self.CP_SP, low_speed_override=False, max_lateral=self.match_lateral)

  def get_faint_lead(self, lead_dict: dict[str, Any], lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
    """RadarConfirmedFaintLead: the camera lead's radar match when the camera isn't confident of it but the radar
    confirms it; lead_dict otherwise."""
    held, self.faint_lead_track = self.faint_lead_track, None
    lead_prob = self.lead_prob_filters[0].x
    if lead_dict['present'] or not self.ready or not self.tracks or self.v_ego < V_EGO_STATIONARY or \
       not FAINT_LEAD_HOLD_PROB < lead_prob <= .5:
      return lead_dict
    track = match_vision_to_track(self.v_ego, lead_msg, self.tracks, max_lateral=self.match_lateral)
    if track is None or not (lead_prob > FAINT_LEAD_PROB or track.identifier == held) or \
       not track.confirms_faint_lead(held=track.identifier == held):
      return lead_dict
    self.faint_lead_track = track.identifier
    lead = track.get_RadarState(lead_prob)
    lead['aLeadK'], lead['aLeadTau'] = 0.0, _LEAD_ACCEL_TAU
    return get_custom_yrel(self.CP, self.CP_SP, lead, lead_msg)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  cloudlog.info("radard is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got CarParamsSP")

  # *** setup messaging
  # radarTracksSP comes only from radars that report per-track extras
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks', 'radarTracksSP'], poll='modelV2',
                           ignore_alive=['radarTracksSP'], ignore_avg_freq=['radarTracksSP'], ignore_valid=['radarTracksSP'])
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'], sm['radarTracksSP'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
