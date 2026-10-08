"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params


class ModelStateBase:
  def __init__(self):
    self.lat_delay = Params().get("LagdValueCache", return_default=True)
    # modeld writes these before each run(), for jetlink's joining model: the camera's dropped-frame share,
    # and whether openpilot or MADS is in control (the large model swaps in only while nothing is)
    self.frame_drop_ratio = 0.
    self.in_control = True
