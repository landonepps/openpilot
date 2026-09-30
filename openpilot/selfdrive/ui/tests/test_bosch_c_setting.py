"""Exercise the real setting callback without starting GLFW or native IPC."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car.honda.bosch_c_radar_live import supported
from opendbc.car.honda.tests.test_bosch_c_radar_live import params


def setting():
  path = Path(__file__).parents[1] / 'mici/layouts/settings/developer.py'
  tree = ast.parse(path.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DeveloperLayoutMici')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_on_bosch_c_enabled')
  values = {'HondaBoschCExperimentalRadar': False, 'AlphaLongitudinalEnabled': True}
  writes, restarts, pages, checked = [], [], [], []

  def put(key, value, **_):
    writes.append((key, value))
    values[key] = value

  state = SimpleNamespace(CP=params()[0], engaged=False, is_release=False, offroad=True,
                          params=SimpleNamespace(get_bool=values.get, put_bool=put))
  state.is_offroad = lambda: state.offroad
  widgets = []
  namespace = {
    'ui_state': state, 'bosch_c_supported': supported,
    'restart_needed_callback': lambda: restarts.append(True),
    'gui_app': SimpleNamespace(push_widget=pages.append, texture=lambda *_: None),
    'NavScroller': lambda: SimpleNamespace(_scroller=SimpleNamespace(add_widgets=widgets.extend), dismiss=lambda cb: cb()),
    'GreyBigButton': lambda *_: None,
    'BigConfirmationCircleButton': lambda title, icon, cb: cb,
  }
  wrapper = ast.ClassDef(name='Setting', bases=[], keywords=[], body=[method], decorator_list=[])
  exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[])), str(path), 'exec'), namespace)
  obj = namespace['Setting']()
  obj._bosch_c_toggle = SimpleNamespace(set_checked=checked.append)
  return obj, state, values, writes, restarts, widgets


def test_enable_requires_confirmation_and_changes_only_radar_setting():
  obj, state, values, writes, restarts, widgets = setting()
  obj._on_bosch_c_enabled(True)
  assert not writes and not restarts
  widgets[-1]()
  assert writes == [('HondaBoschCExperimentalRadar', True)] and restarts == [True]
  assert values['AlphaLongitudinalEnabled']


@pytest.mark.parametrize('change', ['onroad', 'engaged', 'release', 'no_params', 'stock_longitudinal'])
def test_confirmation_rechecks_conditions(change):
  obj, state, values, writes, restarts, widgets = setting()
  obj._on_bosch_c_enabled(True)
  if change == 'onroad':
    state.offroad = False
  elif change == 'engaged':
    state.engaged = True
  elif change == 'release':
    state.is_release = True
  elif change == 'no_params':
    state.CP = None
  else:
    state.CP.openpilotLongitudinalControl = False
  widgets[-1]()
  assert not writes and not restarts


def test_disable_remains_available_without_supported_carparams():
  obj, state, values, writes, restarts, widgets = setting()
  values['HondaBoschCExperimentalRadar'] = True
  state.CP = None
  obj._on_bosch_c_enabled(False)
  assert writes == [('HondaBoschCExperimentalRadar', False)] and restarts == [True]
  assert values['AlphaLongitudinalEnabled']


LIVE_SETTINGS = [
  ('_on_bosch_c_gate_enabled', '_bosch_c_gate_toggle', 'HondaBoschCUncertaintyGate'),
  ('_on_model_lead_trajectory', '_model_lead_toggle', 'ModelLeadTrajectory'),
  ('_on_new_track_hold', '_new_track_hold_toggle', 'RadarNewTrackHold'),
  ('_on_uncertainty_filter', '_uncertainty_filter_toggle', 'RadarUncertaintyFilter'),
  ('_on_faint_lead', '_faint_lead_toggle', 'RadarConfirmedFaintLead'),
]


def live_setting(method_name, toggle, key):
  path = Path(__file__).parents[1] / 'mici/layouts/settings/developer.py'
  tree = ast.parse(path.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DeveloperLayoutMici')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == method_name)
  values = {key: False}
  writes, restarts, checked = [], [], []

  def put(key, value, **_):
    writes.append((key, value))
    values[key] = value

  state = SimpleNamespace(engaged=False, is_release=False, offroad=False, params=SimpleNamespace(get_bool=values.get, put_bool=put))
  state.is_offroad = lambda: state.offroad
  namespace = {'ui_state': state, 'restart_needed_callback': lambda: restarts.append(True)}
  wrapper = ast.ClassDef(name='Setting', bases=[], keywords=[], body=[method], decorator_list=[])
  exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[])), str(path), 'exec'), namespace)
  obj = namespace['Setting']()
  setattr(obj, toggle, SimpleNamespace(set_checked=checked.append))
  return getattr(obj, method_name), state, writes, restarts, checked


@pytest.mark.parametrize('method,toggle,key', LIVE_SETTINGS)
def test_live_setting_toggles_onroad_while_disengaged_without_restart(method, toggle, key):
  callback, state, writes, restarts, checked = live_setting(method, toggle, key)
  callback(True)
  callback(False)
  assert writes == [(key, True), (key, False)]
  assert checked == [True, False] and not restarts


@pytest.mark.parametrize('change', ['engaged', 'release'])
@pytest.mark.parametrize('method,toggle,key', LIVE_SETTINGS)
def test_live_setting_refuses_while_engaged_or_on_release(method, toggle, key, change):
  callback, state, writes, restarts, checked = live_setting(method, toggle, key)
  setattr(state, 'engaged' if change == 'engaged' else 'is_release', True)
  callback(True)
  assert not writes and checked == [False] and not restarts
