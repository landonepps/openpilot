import ast
from pathlib import Path
from types import SimpleNamespace

import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.radar_capture_button import RadarCaptureButton, WIDTH, HEIGHT


def event(x, y, *, pressed=False, released=False, slot=0):
  return SimpleNamespace(pos=rl.Vector2(x, y), slot=slot, left_pressed=pressed, left_released=released, left_down=not released)


def button():
  clicks = []
  obj = RadarCaptureButton(SimpleNamespace(state='idle', label='Radar log'), lambda: clicks.append(True))
  obj.visible = True
  obj.rect = rl.Rectangle(176, 184, WIDTH, HEIGHT)
  return obj, clicks


def test_tap_captures_press_and_release_for_parent_gestures():
  obj, clicks = button()
  down, up = event(205, 203, pressed=True), event(205, 203, released=True)
  assert obj.blocks_scrolling([down])
  obj.handle_events([down], True)
  assert obj.interacting and obj.owns_touch and not clicks
  obj.handle_events([up], True)
  assert clicks == [True] and obj.interacting and not obj.owns_touch
  obj.handle_events([], True)
  assert not obj.interacting


def test_drag_cancel_does_not_trigger_capture_or_parent_release():
  obj, clicks = button()
  obj.handle_events([event(205, 203, pressed=True)], True)
  obj.handle_events([event(235, 203)], True)
  obj.handle_events([event(205, 203, released=True)], True)
  assert not clicks and obj.interacting


def test_touch_started_outside_and_secondary_touch_do_not_trigger():
  obj, clicks = button()
  obj.handle_events([event(20, 20, pressed=True), event(205, 203, released=True)], True)
  obj.handle_events([event(205, 203, pressed=True, slot=1), event(205, 203, released=True, slot=1)], True)
  assert not clicks


def test_alert_hiding_button_cancels_gesture_but_keeps_release_claimed():
  obj, clicks = button()
  obj.handle_events([event(205, 203, pressed=True)], True)
  obj.render(rl.Rectangle(0, 0, 476, 240), visible=False, font=None, measure=None,
             events=[event(205, 203, released=True)], touch_valid=True)
  assert not clicks and obj.interacting and not obj.visible


def test_actual_parent_release_method_does_not_navigate_after_capture_tap():
  # Load the actual two routing methods without camera/IPC startup. This tests
  # the production parent guard together with the real button touch handler.
  path = Path(__file__).parents[1] / 'mici/onroad/augmented_road_view.py'
  tree = ast.parse(path.read_text())
  source = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'AugmentedRoadView')
  methods = [n for n in source.body if isinstance(n, ast.FunctionDef) and n.name in ('_handle_mouse_release', 'is_swiping_left')]

  class Parent:
    def _handle_mouse_release(self, pos):
      self.navigated = True

  namespace = {'Parent': Parent, 'gui_app': SimpleNamespace(mouse_events=[])}
  wrapper = ast.ClassDef(name='AugmentedRoadView', bases=[ast.Name(id='Parent', ctx=ast.Load())], keywords=[], body=methods, decorator_list=[])
  future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
  exec(compile(ast.fix_missing_locations(ast.Module(body=[future, wrapper], type_ignores=[])), str(path), 'exec'), namespace)
  parent = namespace['AugmentedRoadView']()
  parent.navigated = False
  parent._radar_button, clicks = button()
  parent._bookmark_icon = SimpleNamespace(interacting=lambda: False, is_swiping_left=lambda: False)
  down, up = event(205, 203, pressed=True), event(205, 203, released=True)
  namespace['gui_app'].mouse_events = [down]
  assert parent.is_swiping_left()  # Scroller is blocked even before button render.
  parent._radar_button.handle_events([down, up], True)
  parent._handle_mouse_release(up.pos)
  assert clicks == [True] and not parent.navigated
  parent._radar_button.handle_events([], True)
  parent._handle_mouse_release(rl.Vector2(30, 30))
  assert parent.navigated
