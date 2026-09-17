"""Passive radar bird's-eye preview. Coordinates remain calibration hypotheses."""
import pyray as rl

WIDTH, HEIGHT = 160, 126
RANGE_M, HALF_WIDTH_M = 100., 10.
BADGE_WIDTH, BADGE_HEIGHT = 216, 78


def preview_objects(preview):
  tracks = preview.get('tracks', []) if preview else []
  visible = [t for t in tracks if 0 < t['x'] <= RANGE_M and abs(t['y']) <= HALF_WIDTH_M]
  central = [t for t in visible if t['candidate'] and abs(t['y']) <= 2.]
  return visible, min(central, key=lambda t: t['x'], default=None), len(tracks)


def render_center_badge(content_rect, preview, selected, *, font, measure):
  # Fixed HUD position, not a camera projection or a claim of ACC lead lock.
  # Keep clear of the upper-left speed display and the radar map below.
  rect = rl.Rectangle(content_rect.x + content_rect.width - BADGE_WIDTH - 10,
                      content_rect.y + 12, BADGE_WIDTH, BADGE_HEIGHT)
  if preview is None:
    accent = rl.Color(255, 195, 80, 240)
    background = rl.Color(38, 32, 22, 220)
    label, detail = 'WAITING', 'No fresh radar data'
  elif selected is None:
    accent = rl.Color(185, 195, 205, 220)
    background = rl.Color(20, 25, 30, 200)
    label, detail = 'NONE', 'No center track'
  else:
    accent = rl.Color(95, 240, 240, 255)
    background = rl.Color(8, 65, 70, 235)
    label, detail = f"YES  ~{selected['x']:.1f} m", f"Track #{selected['track_id']} | estimate"
  rl.draw_rectangle_rounded(rect, .18, 8, background)
  rl.draw_rectangle_rounded_lines_ex(rect, .18, 8, 2., accent)
  for text, offset, size in (('CENTER CANDIDATE', 6, 14), (label, 27, 24), (detail, 59, 11)):
    width = measure(font, text, size).x
    fitted = size * min(1., (BADGE_WIDTH - 20) / max(1., width))
    x = rect.x + (BADGE_WIDTH - measure(font, text, fitted).x) / 2
    rl.draw_text_ex(font, text, rl.Vector2(x, rect.y + offset), fitted, 0, accent)


def render_radar_overlay(content_rect, controller, *, visible, font, measure):
  if not visible or not controller.active:
    return
  rect = rl.Rectangle(content_rect.x + content_rect.width - WIDTH - 10,
                      content_rect.y + content_rect.height - HEIGHT - 14, WIDTH, HEIGHT)
  rl.draw_rectangle_rounded(rect, .12, 8, rl.Color(16, 22, 28, 160))
  white = rl.Color(240, 245, 250, 240)
  cyan = rl.Color(95, 225, 230, 255)
  amber = rl.Color(255, 195, 80, 255)

  def text(label, x, y, width, size=12, color=white):
    measured = measure(font, label, size)
    fitted = size * min(1., width / max(1., measured.x))
    rl.draw_text_ex(font, label, rl.Vector2(rect.x + x, rect.y + y), fitted, 0, color)

  preview = controller.preview
  tracks, selected, total = preview_objects(preview)
  text(f'Radar ~ {len(tracks)}/{total}', 8, 5, WIDTH - 16, 14)
  if preview is None:
    text('No fresh preview', 8, 40, WIDTH - 16, color=amber)
  else:
    # Linear plan view: 100 m ahead at top, +/-10 m across, car at bottom.
    x0, y0, w, h = rect.x + 8, rect.y + 29, 60, 64
    grid = rl.Color(210, 225, 235, 75)
    for fraction in (0., .5, 1.):
      rl.draw_line_v(rl.Vector2(x0, y0 + h * fraction), rl.Vector2(x0 + w, y0 + h * fraction), grid)
    rl.draw_line_v(rl.Vector2(x0 + w / 2, y0), rl.Vector2(x0 + w / 2, y0 + h), grid)
    rl.draw_rectangle(int(x0 + w / 2 - 3), int(y0 + h), 6, 4, white)
    for track in tracks:
      point = rl.Vector2(x0 + w * (.5 - track['y'] / (2 * HALF_WIDTH_M)), y0 + h * (1 - track['x'] / RANGE_M))
      if track['candidate']:
        rl.draw_circle_v(point, 2.5, cyan)
      else:
        rl.draw_circle_lines_v(point, 3., amber)
      if selected is track:
        rl.draw_circle_lines_v(point, 5., white)
    if selected:
      for row, label in enumerate((f"#{selected['track_id']}", f"~{selected['x']:.1f} m",
                                   f"{selected['v']:+.1f} m/s", f"age {selected['age_s']:.0f}s")):
        text(label, 78, 27 + row * 17, WIDTH - 86)
    else:
      text('No central', 78, 42, WIDTH - 86)
      text('candidate', 78, 59, WIDTH - 86)
  text(controller.marker_notice or '100m / +/-10m', 8, 106, WIDTH - 16)
  render_center_badge(content_rect, preview, selected, font=font, measure=measure)
