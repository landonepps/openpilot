from types import SimpleNamespace

import pyray as rl
import pytest

from openpilot.selfdrive.ui.mici.onroad.radar_debug_overlay import preview_objects, render_center_badge, render_radar_overlay


def track(ident, x, y=0, candidate=True):
  return {'track_id': ident, 'x': x, 'y': y, 'v': 0., 'candidate': candidate, 'age_s': 3.}


def test_map_shows_guard_rejects_but_does_not_select_them_as_central_candidate():
  tracks = [track(1, 10, candidate=False), track(2, 20, y=4), track(3, 30), track(4, 101), track(5, 15, y=-11)]
  visible, selected, count = preview_objects({'tracks': tracks})
  assert [t['track_id'] for t in visible] == [1, 2, 3]
  assert selected['track_id'] == 3 and count == 5
  assert preview_objects(None) == ([], None, 0)


def test_overlay_hides_with_alert_or_inactive_capture_and_clears_stale_points(monkeypatch):
  calls = []
  for name in ('draw_rectangle_rounded', 'draw_rectangle_rounded_lines_ex', 'draw_text_ex', 'draw_line_v',
               'draw_rectangle', 'draw_circle_v', 'draw_circle_lines_v'):
    monkeypatch.setattr(rl, name, lambda *args, name=name: calls.append((name, args)))
  controller = SimpleNamespace(active=True, preview={'tracks': [track(1, 30)]}, marker_notice='Mark saved')
  rect = rl.Rectangle(0, 0, 476, 240)

  def render(visible):
    render_radar_overlay(rect, controller, visible=visible, font=None, measure=lambda *args: rl.Vector2(70, 12))

  render(False)
  assert not calls
  controller.active = False
  render(True)
  assert not calls
  controller.active = True
  render(True)
  panel = calls[0][1][0]
  assert panel.x > 300 and panel.x + panel.width < 476  # Clear of capture button and side panel.
  assert any(name == 'draw_circle_v' for name, _ in calls)
  assert any(args[1] == 'Mark saved' for name, args in calls if name == 'draw_text_ex')
  calls.clear()
  controller.preview = None
  render(True)
  assert not any(name.startswith('draw_circle') for name, _ in calls)
  assert any(args[1] == 'No fresh preview' for name, args in calls if name == 'draw_text_ex')
  labels = [args[1] for name, args in calls if name == 'draw_text_ex']
  assert 'WAITING' in labels and not any(label.startswith('YES') for label in labels)


@pytest.mark.parametrize('preview,expected', [
  ({'tracks': [track(7, 3.2), track(8, 8)]}, 'YES  ~3.2 m'),
  ({'tracks': [track(7, 3.2, y=2.1)]}, 'NONE'),
  ({'tracks': [track(7, 3.2, candidate=False)]}, 'NONE'),
  ({'tracks': []}, 'NONE'),
  (None, 'WAITING'),
])
def test_center_badge_distinguishes_candidate_absence_and_stale_data(monkeypatch, preview, expected):
  labels = []
  rectangles = []
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', lambda rect, *args: rectangles.append(rect))
  monkeypatch.setattr(rl, 'draw_rectangle_rounded_lines_ex', lambda *args: None)
  monkeypatch.setattr(rl, 'draw_text_ex', lambda font, text, *args: labels.append(text))
  _, selected, _ = preview_objects(preview)
  render_center_badge(rl.Rectangle(0, 0, 476, 240), preview, selected,
                      font=None, measure=lambda font, text, size: rl.Vector2(len(text) * size * .55, size))
  assert labels[:2] == ['CENTER CANDIDATE', expected]
  assert ('Track #7 | estimate' in labels) == (selected is not None)
  rect = rectangles[0]
  assert rect.x > 162 and rect.x + rect.width < 476  # Upper-left speed / side panel remain clear.
  assert rect.y + rect.height < 100  # Radar map starts below the badge.
