"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.cereal import messaging, custom, log
from opendbc.car import structs
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
LongitudinalPersonality = log.LongitudinalPersonality

# GentleHighwayPickup: at highway speed the plan accelerates up to the cruise limit, 0.8 m/s^2 at 25 m/s falling to 0.6 at
# 40 m/s. The cruise target sits at that limit while a lead holds the plan down, so after a brake the plan goes straight
# to it once the lead stops holding it down. On the CR-V's pickups after a brake above 31 m/s, openpilot peaked at a
# median 0.74 m/s^2 against 0.33 for stock ACC (bosch-c-research docs/drive-1db-brake-then-accelerate.md). With the
# setting on, the plan's acceleration builds up from zero at no more than GENTLE_PICKUP_JERK and stops at
# GENTLE_PICKUP_MAX_ACCEL, whether it is returning to the set speed after a brake or climbing to a new one. Both blend in
# between the two speeds in GENTLE_PICKUP_BP, so city driving and launches are unchanged. Braking is never limited.
GENTLE_PICKUP_BP = [15., 25.]  # m/s
GENTLE_PICKUP_MAX_ACCEL = [ACCEL_MAX, 0.3]  # m/s^2
GENTLE_PICKUP_JERK = [10., 0.25]  # m/s^3
# an acceleration above the limit, as after the driver lets go of the gas, comes down to it at this rate rather than at once
GENTLE_PICKUP_RELEASE_JERK = 1.0  # m/s^3

# PersonalityGapOnly: the personality (the distance button) sets only the following gap, and every personality keeps
# standard's jerk and acceleration-change costs. Relaxed keeps its own gap, standard takes aggressive's, and aggressive
# takes stock ACC's. Stock ACC on the CR-V settled a median 28 m behind a lead at 29 m/s, 0.98 s of distance per speed
# (bosch-c-research docs/stock-acc-far-leads.md). The MPC's gap is t_follow * v + 6 m, so 0.8 s gives the same distance
# at highway speed (29 m at 29 m/s) and a longer one in town (16 m at 13 m/s).
STOCK_GAP_T_FOLLOW = 0.8  # s


def get_gap_only_T_FOLLOW(personality) -> float:
  if personality == LongitudinalPersonality.aggressive:
    return STOCK_GAP_T_FOLLOW
  if personality == LongitudinalPersonality.standard:
    return get_T_FOLLOW(LongitudinalPersonality.aggressive)
  return get_T_FOLLOW(personality)


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.

    self.params = Params()
    self.frame = 0
    self.model_lead_trajectory = self.params.get_bool("ModelLeadTrajectory")
    self.gentle_pickup = self.params.get_bool("GentleHighwayPickup")
    self.gap_only = self.params.get_bool("PersonalityGapOnly")

  def read_params(self) -> None:
    if self.frame % int(1. / DT_MDL) == 0:
      self.model_lead_trajectory = self.params.get_bool("ModelLeadTrajectory")
      self.gentle_pickup = self.params.get_bool("GentleHighwayPickup")
      self.gap_only = self.params.get_bool("PersonalityGapOnly")

  def model_leads(self, sm: messaging.SubMaster):
    # The model's lead predictions, when the MPC should plan against them (long_mpc.model_lead_trajectory)
    return sm['modelV2'].leadsV3 if self.model_lead_trajectory else None

  def t_follow(self, sm: messaging.SubMaster) -> float | None:
    """The MPC's following time: PersonalityGapOnly's gap for the personality, or None for the personality's own."""
    if self.gap_only:
      return get_gap_only_T_FOLLOW(sm['selfdriveState'].personality)
    return None

  def weights_personality(self, sm: messaging.SubMaster):
    """The personality whose MPC cost weights to use: standard for every personality under PersonalityGapOnly."""
    if self.gap_only:
      return LongitudinalPersonality.standard
    return sm['selfdriveState'].personality

  def limit_pickup(self, v_ego: float, a_target: float, a_prev: float, dt: float) -> float:
    """GentleHighwayPickup: the plan's acceleration target with its rise and its ceiling limited at speed, given the
    previous tick's target. Lower targets, braking included, pass through."""
    if not self.gentle_pickup:
      return a_target
    max_accel = float(np.interp(v_ego, GENTLE_PICKUP_BP, GENTLE_PICKUP_MAX_ACCEL))
    jerk = float(np.interp(v_ego, GENTLE_PICKUP_BP, GENTLE_PICKUP_JERK))
    rise_limit = max(a_prev, 0.) + jerk * dt
    ceiling = max(max_accel, a_prev - GENTLE_PICKUP_RELEASE_JERK * dt)
    return min(a_target, rise_limit, ceiling)

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.read_params()
    self.frame += 1
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
