import json
import math
import platform

import numpy as np

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.navd.helpers import coordinate_from_param, Coordinate
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState

ACTIVE_STATES = (MapState.turning, )
ENABLED_STATES = (MapState.enabled, MapState.overriding, *ACTIVE_STATES)

R = 6373000.0  # approximate radius of earth in meters
TO_RADIANS = math.pi / 180
TO_DEGREES = 180 / math.pi
TARGET_JERK = -0.6  # m/s^3 There's some jounce limits that are not consistent so we're fudging this some
TARGET_ACCEL = -1.2  # m/s^2 should match up with the long planner limit
TARGET_OFFSET = 1.0  # seconds - This controls how soon before the curve you reach the target velocity. It also helps
                     # reach the target velocity when inaccuracies in the distance modeling logic would cause overshoot.
                     # The value is multiplied against the target velocity to determine the additional distance. This is
                     # done to keep the distance calculations consistent but results in the offset actually being less
                     # time than specified depending on how much of a speed differential there is between v_ego and the
                     # target velocity.
MAPD_TARGET_LAT_ACCEL = 2.0  # m/s^2, the lateral acceleration mapd computes MapTargetVelocities for
MAPD_MERGE_VELOCITY = math.sqrt(MAPD_TARGET_LAT_ACCEL / 0.0015)  # m/s, mapd's velocity at nodes near a merge or split
MAP_CURVE_LAT_ACCEL_RANGE = (1.0, 3.5)  # m/s^2, bounds for MapCurveLatAccel

# MapCurveGentleBraking: instead of a curve point's own velocity once it is within TARGET_ACCEL braking distance, the
# target is the highest speed from which every point ahead is reached at its velocity braking at GENTLE_DECEL. The
# planner brakes by the speed gap (get_cruise_accel, gain 1/s), so the car trails a falling target by about CRUISE_LAG
# of travel, and each point's braking starts that much earlier. Between points the same target limits the pickup to
# what the next point allows. On 13 logged drives through one freeway connector (closed-loop replay), this braked at
# -0.5 m/s^2 from when mapd first saw the curve, against -1.2 m/s^2 later, for the same speed through the curve.
GENTLE_DECEL = 0.5  # m/s^2
CRUISE_LAG = 1.0  # s


# MapCurveSmoothSlowing: mapd's velocities come from the unsigned curvature of each three consecutive OSM nodes, so a
# node placed a few meters sideways reads as a tight curve: on the CA-85 S to I-280 S connector one such node set the
# 51 mph floor where the car's path allows about 60 mph, and on the I-280 N to CA-85 N ramp one asked for 35 mph on a
# curve the car took at 55. With the setting on, the curvature comes from a smooth path fitted to mapd's nodes (see
# fitted_curvatures) instead of from each three nodes, so a misplaced node bends the path a little over a long stretch
# rather than sharply at one node, and a real curve keeps its curvature. A node's speed is
# sqrt(MapCurveLatAccel / curvature). Where a way gains lanes (a ramp merging) or a two-way road becomes one-way, its
# centerline jumps sideways onto the next way's, which the car's lane doesn't do; mapd sets about 15 m around those
# nodes to curvature 0.0015, which marks them in its velocities, and the fit gives nodes within SMOOTH_MERGE_DIST of them
# almost no weight (on the I-280 N to CA-85 N merge an 8 m jump otherwise read as a 4.4 /km bend, 2.9 with it).
# Checked against the car's own path curvature at 11,170 distinct node passes above 20 m/s (bosch-c-research
# docs/drives-1ea-1f2-braking-and-curves.md), this asked for less than 0.85x a speed the car took comfortably at 3 of
# them, against mapd's 66 and 83 for a 100 m heading-change window, and in curves its estimated speed over the car's
# was 0.943/1.019/1.125 at p10/p50/p90 (mapd 0.903/1.017/1.184).
# mapd starts each list two nodes into the way the car is on, so the previous list's nodes behind the new first node are
# kept: without the path behind them, the first nodes' curvature jumps when a new list arrives.
# The car's distance along the path is projected from LastGPSPosition, which the live map process writes once per
# second, and dead-reckoned from the car's speed in between, so the distances to the nodes fall smoothly instead of in
# 30 m steps at highway speed. Braking follows the constant deceleration each node needs, (v^2 - v_node^2) / 2d,
# from when the hardest of them reaches SMOOTH_BRAKE_START; a curve mapd sees late is braked for at what it needs, up
# to the planner's -1.2 m/s^2, rather than at -1.2 at once. Braking goes on while any node ahead still needs more than
# SMOOTH_BRAKE_STOP: mapd lists about 500 m, so a curve's slowest nodes can show up only once braking has started (on the
# CA-85 S to I-280 S connector 2.5-4.5 s in, about a mph slower), and they are met in the same braking instead of with a
# short second one. Until braking starts the same envelope caps pickups, and the speed never rises past the slowest node
# of the next SMOOTH_HOLD_TIME of travel less SMOOTH_HOLD_MARGIN, so the car doesn't pick up between two tight parts of
# a ramp only to brake again. mapd's list can reach several km on a long way, so the hold has its own horizon: holding
# for a curve 2.6 km ahead kept the car 1 mph under the set speed for 90 s.
SMOOTH_FIT_LENGTH = 22.  # m
SMOOTH_FIT_STEP = 5.  # m
SMOOTH_FIT_AVERAGE = 30.  # m
SMOOTH_MERGE_DIST = 30.  # m
SMOOTH_MERGE_WEIGHT = 0.01
SMOOTH_BRAKE_START = 0.5  # m/s^2
SMOOTH_BRAKE_STOP = 0.02  # m/s^2, braking continues while a node still needs more than this
SMOOTH_MIN_BRAKE_TIME = 1.0  # s; a node's braking distance counts as at least this much travel
SMOOTH_HOLD_TIME = 15.0  # s of travel
SMOOTH_HOLD_MARGIN = 1.0  # m/s
SMOOTH_STITCH_NODES = 10
SMOOTH_MAX_PATH_OFFSET = 30.  # m; farther from mapd's path than this, the car isn't on it and the map is ignored
SMOOTH_MIN_CURVATURE = 1e-4  # 1/m


def local_xy(lat, lon, origin: tuple[float, float]):
  """Meters east and north of origin (lat, lon in degrees); flat-earth, for paths of a few km."""
  return ((lon - origin[1]) * TO_RADIANS * R * math.cos(origin[0] * TO_RADIANS), (lat - origin[0]) * TO_RADIANS * R)


def stitch_nodes(previous: list[tuple[float, float]], points) -> list[tuple[float, float]]:
  """mapd's nodes (latitude, longitude), without its zero-velocity (no curvature) entries, behind which go the previous
  list's nodes up to the new first node when the previous list had it. mapd starts each list two nodes into the way the
  car is on, so the first nodes of a new list would otherwise lose the path behind them from their curvature window."""
  nodes = [(p["latitude"], p["longitude"]) for p in points if p["velocity"] > 0]
  if nodes and nodes[0] in previous:
    i = previous.index(nodes[0])
    nodes = previous[max(i - SMOOTH_STITCH_NODES, 0):i] + nodes
  return nodes


def merge_nodes(points) -> set[tuple[float, float]]:
  """mapd's nodes (latitude, longitude) near a merge or split."""
  return {(p["latitude"], p["longitude"]) for p in points if abs(p["velocity"] - MAPD_MERGE_VELOCITY) < 1e-3}


def path_from_points(nodes: list[tuple[float, float]], merges: set[tuple[float, float]]) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[float, float]]:
  """The node path in local meters of its first node, without repeated nodes, which of its nodes are in `merges`, and
  that origin."""
  if len(nodes) < 2:
    return np.zeros(0), np.zeros(0), np.zeros(0, dtype=bool), (0., 0.)
  lat, lon = np.array(nodes).T
  origin = (float(lat[0]), float(lon[0]))
  x, y = local_xy(lat, lon, origin)
  keep = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > 0.5])
  return x[keep], y[keep], np.array([node in merges for node in nodes])[keep], origin


def solve_banded_spd(band: np.ndarray, rhs: np.ndarray) -> np.ndarray:
  """The solution z of A z = rhs (rows of rhs are A's rows) for a symmetric positive definite A given by its diagonal
  and upper diagonals, band[d, i] = A[i, i + d], by a banded Cholesky factorization: O(n p^2) for p upper diagonals
  instead of a dense solve's O(n^3)."""
  p, n = band.shape[0] - 1, band.shape[1]
  a = band.tolist()
  low = [[0.] * n for _ in range(p + 1)]  # low[d][i] = L[i, i - d] of A = L L^T
  for i in range(n):
    for d in range(min(i, p), 0, -1):
      k = i - d
      v = a[d][k]
      for e in range(d + 1, min(i, p) + 1):
        v -= low[e][i] * low[e - d][k]
      low[d][i] = v / low[0][k]
    v = a[0][i]
    for e in range(1, min(i, p) + 1):
      v -= low[e][i] ** 2
    low[0][i] = math.sqrt(v)
  out = np.empty((n, rhs.shape[1]))
  for c in range(rhs.shape[1]):
    z = rhs[:, c].tolist()
    for i in range(n):
      v = z[i]
      for e in range(1, min(i, p) + 1):
        v -= low[e][i] * z[i - e]
      z[i] = v / low[0][i]
    for i in range(n - 1, -1, -1):
      v = z[i]
      for e in range(1, min(n - 1 - i, p) + 1):
        v -= low[e][i + e] * z[i + e]
      z[i] = v / low[0][i]
    out[:, c] = z
  return out


def fitted_curvatures(x: np.ndarray, y: np.ndarray, merge: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Arc length of each node and the signed curvature there of a smooth path fitted to the nodes, averaged over
  SMOOTH_FIT_AVERAGE around the node. The path, positions every SMOOTH_FIT_STEP of the node path's arc length, minimizes
  the squared distance to each node, weighted by its share of the path's length, plus SMOOTH_FIT_LENGTH^6 times the
  integral of its squared third derivative (how fast the curvature changes; roads are built from lines, arcs and the
  spirals between them, so a constant curve costs almost nothing). A node a few meters sideways is then mostly missed
  instead of followed through a sharp bend. Nodes within SMOOTH_MERGE_DIST of a `merge` node get SMOOTH_MERGE_WEIGHT of
  their weight. Needs at least 3 nodes."""
  s = np.concatenate([[0.], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
  m = max(math.ceil(s[-1] / SMOOTH_FIT_STEP) + 1, 5)
  h = s[-1] / (m - 1)
  j = np.minimum((s / h).astype(int), m - 2)  # the path's position at each node is linear between grid points j and j + 1
  u = s / h - j
  ds = np.diff(s)
  w = np.concatenate([[ds[0]], ds[:-1] + ds[1:], [ds[-1]]]) / 2.
  if merge.any():
    w[(np.abs(s[:, None] - s[merge][None, :]) <= SMOOTH_MERGE_DIST).any(axis=1)] *= SMOOTH_MERGE_WEIGHT
  # the normal equations are banded (a node touches two grid points, a third difference four), so they are built as
  # diagonals and solved in O(m). A dense solve is O(m^3): on the comma a 4.4 km list (m = 881) took 0.7 s, which held
  # up plannerd and radard (same core) long enough for "Communication Issue Between Processes" (routes 1f4-1f9)
  band = np.zeros((4, m))  # band[d, i] = lhs[i, i + d]
  np.add.at(band[0], j, w * (1. - u) ** 2)
  np.add.at(band[0], j + 1, w * u ** 2)
  np.add.at(band[1], j, w * u * (1. - u))
  third = (-1., 3., -3., 1.)  # each third difference, times h^3, over four consecutive grid points
  for a in range(4):
    for b in range(a, 4):
      band[b - a, a:m - 3 + a] += SMOOTH_FIT_LENGTH ** 6 * h * third[a] * third[b] / h ** 6
  rhs = np.zeros((m, 2))
  xy = np.stack([x, y], axis=1)
  np.add.at(rhs, j, (w * (1. - u))[:, None] * xy)
  np.add.at(rhs, j + 1, (w * u)[:, None] * xy)
  path = solve_banded_spd(band, rhs)
  s_grid = np.linspace(0., s[-1], m)
  d1 = np.gradient(path, s_grid, axis=0)
  d2 = np.gradient(d1, s_grid, axis=0)
  curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(np.hypot(d1[:, 0], d1[:, 1]), 1e-6) ** 3
  heading = np.concatenate([[0.], np.cumsum((curvature[1:] + curvature[:-1]) / 2. * h)])
  lo = np.clip(s - SMOOTH_FIT_AVERAGE / 2., 0., s[-1])
  hi = np.clip(s + SMOOTH_FIT_AVERAGE / 2., 0., s[-1])
  return s, (np.interp(hi, s_grid, heading) - np.interp(lo, s_grid, heading)) / np.maximum(hi - lo, SMOOTH_FIT_AVERAGE / 2.)


def project_on_path(x: np.ndarray, y: np.ndarray, px: float, py: float) -> tuple[float, float]:
  """Arc length of the point on the path nearest (px, py), and its distance from the path. mapd's list leaves out the
  first two nodes of the way the car is on, which can put its first node 150 m or more ahead, so the first segment
  extends backwards: a car behind the path's start gets a negative arc length."""
  dx, dy = np.diff(x), np.diff(y)
  seg_len2 = np.maximum(dx ** 2 + dy ** 2, 1e-6)
  u = np.clip(((px - x[:-1]) * dx + (py - y[:-1]) * dy) / seg_len2, 0., 1.)
  u[0] = min(((px - x[0]) * dx[0] + (py - y[0]) * dy[0]) / seg_len2[0], 1.)
  dist = np.hypot(x[:-1] + u * dx - px, y[:-1] + u * dy - py)
  i = int(np.argmin(dist))
  s = np.concatenate([[0.], np.cumsum(np.sqrt(seg_len2))])
  return float(s[i] + u[i] * math.sqrt(seg_len2[i])), float(dist[i])


def smooth_slowing_target(s_nodes: np.ndarray, v_nodes: np.ndarray, s_car: float, v_ego: float, v_cruise: float,
                          braking: bool) -> tuple[float, bool]:
  """MapCurveSmoothSlowing's target speed and braking state, given each node's arc length and speed and the car's arc
  length. Nodes within TARGET_OFFSET of travel ahead hold their own speed. While braking, the target is the car's speed
  less the deceleration the hardest node needs (the planner accelerates by the speed gap, gain 1/s); otherwise it is the
  highest speed from which every node ahead is reached braking at SMOOTH_BRAKE_START. Either way it stays at or below
  the car's speed while a node of the next SMOOTH_HOLD_TIME of travel is less than SMOOTH_HOLD_MARGIN faster, the margin
  shrinking to nothing as that node's speed nears the set speed v_cruise. 0 when no node binds."""
  v_hold = v_envelope = math.inf
  a_need = 0.
  for s_i, v_i in zip(s_nodes, v_nodes, strict=True):
    d = s_i - s_car
    if d <= 0.:
      continue
    d -= v_i * TARGET_OFFSET
    if d <= 0.:
      v_hold = min(v_hold, v_i)
    else:
      if v_i < v_ego:
        a_need = max(a_need, (v_ego ** 2 - v_i ** 2) / (2. * max(d, v_ego * SMOOTH_MIN_BRAKE_TIME)))
      v_envelope = min(v_envelope, math.sqrt(v_i ** 2 + 2. * SMOOTH_BRAKE_START * d))
  braking = a_need >= SMOOTH_BRAKE_START or (braking and a_need > SMOOTH_BRAKE_STOP)
  v_target = min(v_hold, v_ego - a_need * CRUISE_LAG if braking else v_envelope)
  # no pickup past the slowest node of the next SMOOTH_HOLD_TIME of travel (less SMOOTH_HOLD_MARGIN), so the speed holds
  # through a curve and between the tight parts of a ramp instead of rising and falling with its nodes; this never brakes
  # the car. The margin shrinks as that node's speed nears the set speed, so a node the car can take at the set speed
  # doesn't hold it below it
  v_ahead = min((v_i for s_i, v_i in zip(s_nodes, v_nodes, strict=True) if 0. < s_i - s_car <= v_ego * SMOOTH_HOLD_TIME), default=math.inf)
  margin = min(SMOOTH_HOLD_MARGIN, max(v_cruise - v_ahead, 0.))
  v_target = min(v_target, max(v_ego, v_ahead - margin))
  return (0. if math.isinf(v_target) else max(v_target, 0.)), braking


def gentle_braking_target(points, distances, v_ego: float) -> float:
  """MapCurveGentleBraking's target: the highest speed from which each point ahead is reached at its velocity braking at
  GENTLE_DECEL, keeping the TARGET_OFFSET margin. 0 when no point has a velocity."""
  v_target = math.inf
  for target_velocity, d in zip(points, distances, strict=True):
    tv = target_velocity["velocity"]
    if tv <= 0:
      continue
    braking_distance = max(d - tv * TARGET_OFFSET - v_ego * CRUISE_LAG, 0.)
    v_target = min(v_target, math.sqrt(tv ** 2 + 2 * GENTLE_DECEL * braking_distance))
  return 0. if math.isinf(v_target) else v_target


def velocities_from_param(param: str, params: Params):
  if params is None:
    params = Params()

  json_str = params.get(param)
  if json_str is None:
    return None

  velocities = json.loads(json_str)

  return velocities


def calculate_accel(t, target_jerk, a_ego):
  return a_ego + target_jerk * t


def calculate_velocity(t, target_jerk, a_ego, v_ego):
  return v_ego + a_ego * t + target_jerk/2 * (t ** 2)


def calculate_distance(t, target_jerk, a_ego, v_ego):
  return t * v_ego + a_ego/2 * (t ** 2) + target_jerk/6 * (t ** 3)


# points should be in radians
# output is meters
def distance_to_point(ax, ay, bx, by):
  a = math.sin((bx-ax)/2)*math.sin((bx-ax)/2) + math.cos(ax) * math.cos(bx)*math.sin((by-ay)/2)*math.sin((by-ay)/2)
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

  return R * c  # in meters


class SmartCruiseControlMap:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.enabled = self.params.get_bool("SmartCruiseControlMap")
    self.gentle_braking = self.params.get_bool("MapCurveGentleBraking")
    self.smooth_slowing = self.params.get_bool("MapCurveSmoothSlowing")
    self.lat_accel = self.read_lat_accel()
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.state = MapState.disabled
    self.v_cruise = 0
    self.target_lat = 0.0
    self.target_lon = 0.0
    self.frame = -1

    self.last_position = coordinate_from_param("LastGPSPosition", self.mem_params) or Coordinate(0.0, 0.0)
    self.target_velocities = velocities_from_param("MapTargetVelocities", self.mem_params) or []

    # MapCurveSmoothSlowing state: mapd's node path, the car's travel since the last position fix, and braking
    self.path_source: list | None = None
    self.path_nodes: list[tuple[float, float]] = []
    self.path_merges: set[tuple[float, float]] = set()
    self.path_x, self.path_y, self.path_origin = np.zeros(0), np.zeros(0), (0., 0.)
    self.path_s, self.path_curvatures = np.zeros(0), np.zeros(0)
    self.odometer = 0.
    self.fix_coord: tuple[float, float] | None = None
    self.odometer_at_fix = 0.
    self.smooth_braking = False

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V)

    return V_CRUISE_UNSET

  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def read_lat_accel(self) -> float:
    lo, hi = MAP_CURVE_LAT_ACCEL_RANGE
    return min(max(float(self.params.get("MapCurveLatAccel", return_default=True)), lo), hi)

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlMap")
      self.gentle_braking = self.params.get_bool("MapCurveGentleBraking")
      self.smooth_slowing = self.params.get_bool("MapCurveSmoothSlowing")
      self.lat_accel = self.read_lat_accel()

  def smooth_slowing_update(self, lat: float, lon: float) -> float:
    """MapCurveSmoothSlowing's target for the car at (lat, lon), the latest position fix; see smooth_slowing_target."""
    if (lat, lon) != self.fix_coord:
      self.fix_coord = (lat, lon)
      self.odometer_at_fix = self.odometer
    if self.target_velocities != self.path_source:
      self.path_source = self.target_velocities
      self.path_nodes = stitch_nodes(self.path_nodes, self.target_velocities)
      self.path_merges = (self.path_merges & set(self.path_nodes)) | merge_nodes(self.target_velocities)
      self.path_x, self.path_y, merge, self.path_origin = path_from_points(self.path_nodes, self.path_merges)
      if len(self.path_x) >= 3:
        self.path_s, self.path_curvatures = fitted_curvatures(self.path_x, self.path_y, merge)
    if len(self.path_x) < 3 or (lat == 0. and lon == 0.):
      self.smooth_braking = False
      return 0.

    s_fix, offset = project_on_path(self.path_x, self.path_y, *local_xy(lat, lon, self.path_origin))
    # behind the path's start the road may curve away from the extended first segment
    if offset > SMOOTH_MAX_PATH_OFFSET + 0.2 * max(-s_fix, 0.):
      self.smooth_braking = False
      return 0.
    v_nodes = np.sqrt(self.lat_accel / np.maximum(np.abs(self.path_curvatures), SMOOTH_MIN_CURVATURE))
    s_car = s_fix + self.odometer - self.odometer_at_fix
    v_target, self.smooth_braking = smooth_slowing_target(self.path_s, v_nodes, s_car, self.v_ego, self.v_cruise, self.smooth_braking)
    return v_target

  def update_calculations(self) -> None:
    self.last_position = coordinate_from_param("LastGPSPosition", self.mem_params) or Coordinate(0.0, 0.0)
    lat = self.last_position.latitude
    lon = self.last_position.longitude

    self.target_velocities = velocities_from_param("MapTargetVelocities", self.mem_params) or []
    if self.smooth_slowing:
      self.v_target = self.smooth_slowing_update(lat, lon)
      return
    # MapCurveLatAccel: mapd's velocities are for MAPD_TARGET_LAT_ACCEL; v scales with the square root
    scale = math.sqrt(self.lat_accel / MAPD_TARGET_LAT_ACCEL)
    if scale != 1.:
      self.target_velocities = [dict(tv, velocity=tv["velocity"] * scale) for tv in self.target_velocities]

    if self.last_position is None or self.target_velocities is None:
      return

    min_dist = 1000
    min_idx = 0
    distances = []

    # find our location in the path
    for i in range(len(self.target_velocities)):
      target_velocity = self.target_velocities[i]
      tlat = target_velocity["latitude"]
      tlon = target_velocity["longitude"]
      d = distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, tlat * TO_RADIANS, tlon * TO_RADIANS)
      distances.append(d)
      if d < min_dist:
        min_dist = d
        min_idx = i

    # only look at values from our current position forward
    forward_points = self.target_velocities[min_idx:]
    forward_distances = distances[min_idx:]

    if self.gentle_braking:
      self.v_target = gentle_braking_target(forward_points, forward_distances, self.v_ego)
      return

    # find velocities that we are within the distance we need to adjust for
    valid_velocities = []
    for i in range(len(forward_points)):
      target_velocity = forward_points[i]
      tlat = target_velocity["latitude"]
      tlon = target_velocity["longitude"]
      tv = target_velocity["velocity"]
      if tv > self.v_ego:
        continue

      d = forward_distances[i]

      a_diff = (self.a_ego - TARGET_ACCEL)
      accel_t = abs(a_diff / TARGET_JERK)
      min_accel_v = calculate_velocity(accel_t, TARGET_JERK, self.a_ego, self.v_ego)

      max_d = 0
      if tv > min_accel_v:
        # calculate time needed based on target jerk
        a = 0.5 * TARGET_JERK
        b = self.a_ego
        c = self.v_ego - tv
        t_a = -1 * ((b**2 - 4 * a * c) ** 0.5 + b) / (2 * a)
        t_b = ((b**2 - 4 * a * c) ** 0.5 - b) / (2 * a)
        if not isinstance(t_a, complex) and t_a > 0:
          t = t_a
        else:
          t = t_b
        if isinstance(t, complex):
          continue

        max_d = max_d + calculate_distance(t, TARGET_JERK, self.a_ego, self.v_ego)
      else:
        t = accel_t
        max_d = calculate_distance(t, TARGET_JERK, self.a_ego, self.v_ego)

        # calculate additional time needed based on target accel
        t = abs((min_accel_v - tv) / TARGET_ACCEL)
        max_d += calculate_distance(t, 0, TARGET_ACCEL, min_accel_v)

      if d < max_d + tv * TARGET_OFFSET:
        valid_velocities.append((float(tv), tlat, tlon))

    # Find the smallest velocity we need to adjust for
    min_v = 100.0
    target_lat = 0.0
    target_lon = 0.0
    for tv, lat, lon in valid_velocities:
      if tv < min_v:
        min_v = tv
        target_lat = lat
        target_lon = lon

    if self.v_target < min_v and not (self.target_lat == 0 and self.target_lon == 0):
      for i in range(len(forward_points)):
        target_velocity = forward_points[i]
        tlat = target_velocity["latitude"]
        tlon = target_velocity["longitude"]
        tv = target_velocity["velocity"]
        if tv > self.v_ego:
          continue

        if tlat == self.target_lat and tlon == self.target_lon and tv == self.v_target:
          return

      # not found so let's reset
      self.v_target = 0.0
      self.target_lat = 0.0
      self.target_lon = 0.0

    self.v_target = min_v
    self.target_lat = target_lat
    self.target_lon = target_lon

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, TURNING
    if self.state != MapState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = MapState.disabled
      elif self.long_override:
        self.state = MapState.overriding

      else:
        # ENABLED
        if self.state == MapState.enabled:
          if self.v_cruise > self.v_target != 0:
            self.state = MapState.turning

        # TURNING
        elif self.state == MapState.turning:
          if self.v_cruise <= self.v_target or self.v_target == 0:
            self.state = MapState.enabled

        # OVERRIDING
        elif self.state == MapState.overriding:
          if not self.long_override:
            if self.v_cruise > self.v_target != 0:
              self.state = MapState.turning
            else:
              self.state = MapState.enabled

    # DISABLED
    elif self.state == MapState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = MapState.overriding
        else:
          self.state = MapState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update(self, long_enabled: bool, long_override: bool, v_ego, a_ego, v_cruise) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise = v_cruise
    self.odometer += v_ego * DT_MDL

    self.update_params()
    self.update_calculations()

    self.is_enabled, self.is_active = self._update_state_machine()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
