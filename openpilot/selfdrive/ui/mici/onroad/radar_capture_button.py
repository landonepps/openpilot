"""Compact comma four capture control; drawing and touch routing need no IPC."""
import pyray as rl

WIDTH = 124
HEIGHT = 42
BOTTOM_MARGIN = 14
FONT_SIZE = 15


class RadarCaptureButton:
  def __init__(self, controller, on_toggle):
    self.controller = controller
    self.on_toggle = on_toggle
    self.rect = rl.Rectangle(0, 0, 0, 0)
    self.visible = False
    self.owns_touch = False
    self.consumed_touch = False
    self.cancelled = False
    self.start_pos = (0., 0.)

  @property
  def interacting(self):
    return self.owns_touch or self.consumed_touch

  def hide(self):
    self.visible = False
    self.owns_touch = self.consumed_touch = False

  def blocks_scrolling(self, events):
    return self.visible and (self.owns_touch or any(
      e.slot == 0 and e.left_pressed and rl.check_collision_point_rec(e.pos, self.rect) for e in events))

  def handle_events(self, events, touch_valid):
    self.consumed_touch = False
    for event in events:
      if event.slot != 0:
        continue
      inside = rl.check_collision_point_rec(event.pos, self.rect)
      if event.left_pressed:
        self.owns_touch = self.visible and touch_valid and inside
        self.cancelled = False
        self.start_pos = (event.pos.x, event.pos.y)
      if not self.owns_touch:
        continue
      self.consumed_touch = True
      dx, dy = event.pos.x - self.start_pos[0], event.pos.y - self.start_pos[1]
      if not touch_valid or not inside or dx * dx + dy * dy > 100:
        self.cancelled = True
      if event.left_released:
        self.owns_touch = False
        if not self.cancelled:
          self.on_toggle()

  def render(self, content_rect, *, visible, font, measure, events, touch_valid):
    if not visible:
      self.visible = False
      self.handle_events(events, False)
      return
    self.visible = True
    self.rect = rl.Rectangle(content_rect.x + (content_rect.width - WIDTH) / 2,
                             content_rect.y + content_rect.height - HEIGHT - BOTTOM_MARGIN, WIDTH, HEIGHT)
    self.handle_events(events, touch_valid)
    alpha = 175 if self.owns_touch else 115
    rl.draw_rectangle_rounded(self.rect, 1., 12, rl.Color(16, 22, 28, alpha))
    rl.draw_rectangle_rounded_lines_ex(self.rect, 1., 12, 1., rl.Color(255, 255, 255, 85))
    color = rl.Color(230, 235, 240, 230)
    if self.controller.state == 'recording':
      color = rl.Color(255, 95, 95, 255)
    elif self.controller.state in ('starting', 'waiting', 'stopping'):
      color = rl.Color(255, 195, 80, 255)
    elif self.controller.state == 'saved':
      color = rl.Color(95, 225, 170, 255)
    elif self.controller.state == 'error':
      color = rl.Color(255, 125, 95, 255)
    label = self.controller.label
    size = measure(font, label, FONT_SIZE)
    # Center the indicator + text as one group in the small touch target.
    x = self.rect.x + (self.rect.width - size.x - 16) / 2
    y = self.rect.y + self.rect.height / 2
    rl.draw_circle_v(rl.Vector2(x + 4, y), 4, color)
    rl.draw_text_ex(font, label, rl.Vector2(x + 16, y - size.y / 2), FONT_SIZE, 0, rl.Color(255, 255, 255, 240))
