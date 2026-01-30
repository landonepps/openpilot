"""
Torque Tune Debug Renderer for Comma 4 (Mici) UI

Displays learned torque tune values (friction, latAccelFactor, latAccelOffset,
calibration percentage) when ShowDebugInfo param is enabled.
"""
import time
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class TorqueTuneDebugRenderer(Widget):
  PARAM_CHECK_INTERVAL = 1.0  # Check ShowDebugInfo every 1 second
  FONT_SIZE = 28
  PADDING = 8
  BG_COLOR = rl.Color(0, 0, 0, 150)  # Semi-transparent black
  GREEN = rl.Color(0, 255, 0, 255)
  GRAY = rl.Color(160, 160, 160, 255)

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._show_debug = False
    self._last_param_check = 0.0

  def _update_state(self):
    # Periodically check the ShowDebugInfo param
    now = time.monotonic()
    if now - self._last_param_check > self.PARAM_CHECK_INTERVAL:
      self._show_debug = self._params.get_bool("ShowDebugInfo")
      self._last_param_check = now

  def _render(self, rect: rl.Rectangle):
    # Only render when debug mode is enabled and car is started
    if not self._show_debug or not ui_state.started:
      return

    font = gui_app.font(FontWeight.NORMAL)

    sm = ui_state.sm
    if not sm.valid['liveTorqueParameters']:
      return

    ltp = sm['liveTorqueParameters']
    friction = ltp.frictionCoefficientFiltered
    lat_accel_factor = ltp.latAccelFactorFiltered
    lat_accel_offset = ltp.latAccelOffsetFiltered
    live_valid = ltp.liveValid
    cal_percent = ltp.calPerc

    # Format display lines
    line1 = f"F:{friction:.3f} LAF:{lat_accel_factor:.3f}"
    line2 = f"OFF:{lat_accel_offset:.3f} CAL:{cal_percent}%"

    # Measure text to calculate background size
    line1_size = measure_text_cached(font, line1, self.FONT_SIZE, 0)
    line2_size = measure_text_cached(font, line2, self.FONT_SIZE, 0)
    max_width = max(line1_size.x, line2_size.x)
    total_height = line1_size.y + line2_size.y + self.PADDING

    # Position in top-left, just below FPS counter (around y=40)
    x = rect.x + 16
    y = rect.y + 40

    # Draw semi-transparent background
    bg_rect = rl.Rectangle(
      x - self.PADDING,
      y - self.PADDING,
      max_width + self.PADDING * 2,
      total_height + self.PADDING * 2
    )
    rl.draw_rectangle_rounded(bg_rect, 0.2, 10, self.BG_COLOR)

    # Choose color based on live validity
    text_color = self.GREEN if live_valid else self.GRAY

    # Draw text lines
    rl.draw_text_ex(font, line1, rl.Vector2(x, y), self.FONT_SIZE, 0, text_color)
    rl.draw_text_ex(font, line2, rl.Vector2(x, y + line1_size.y + 4), self.FONT_SIZE, 0, text_color)
