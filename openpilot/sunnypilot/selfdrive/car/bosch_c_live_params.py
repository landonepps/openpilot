"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


def update_bosch_c_live_params(RI, params) -> None:
  """Apply Bosch C radar settings that are safe to change while running.

  The uncertainty gate filters each bank independently, so it follows the
  setting live. HondaBoschCExperimentalRadar changes CarParams and still needs a
  restart. carParamsSP records the gate setting only as it was at startup.
  """
  bosch_c = getattr(RI, 'bosch_c', None)
  if bosch_c is not None:
    bosch_c.uncertainty_gate = params.get_bool("HondaBoschCUncertaintyGate")
