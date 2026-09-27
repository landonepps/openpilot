"""Synthetic coexistence evidence and bounded startup tests; no vehicle required."""
import json

import pytest
from openpilot.cereal import log
from opendbc.car.honda.bosch_c_radar import OBJECT_IDS
from opendbc.car.honda.tests.test_bosch_c_radar import bank, frame
from opendbc.car.honda.tests.test_bosch_c_radar import cp as base_cp
from openpilot.tools.car_porting.bosch_c_coexistence_analyze import DISABLE, REQUEST, analyze


def cp():
  result = base_cp()
  result.safetyConfigs = [{}]
  return result


def fixture(path, *, mode=True, stop=False, empty=False, receipt=True, factory_continues=False,
            pre=True, stale=False, truncated=False, delayed=False, wrong_bus=False, background=True):
  params = cp()
  params.openpilotLongitudinalControl = mode
  e = log.Event.new_message(logMonoTime=1, valid=True)
  e.carParams = params
  rows = [{'kind': 'session'}, {'kind': 'event', 'service': 'carParams', 'timestamp_ns': 1,
                               'valid': True, 'event_hex': e.to_bytes().hex()}]
  counter = 0
  for i in range(181):
    ns = i * 50_000_000
    frames = []
    if background:
      frames.extend([{'address': 0x100, 'src': b, 'payload': '00'} for b in (0, 1)])
    if (pre and i <= 20) or factory_continues:
      frames.append({'address': 0x1DF, 'src': 0, 'payload': '00'})
    if i == 20:
      frames.append({'address': REQUEST, 'src': 129 if wrong_bus else 128 if receipt else 192, 'payload': DISABLE.hex()})
    if (pre or i > 20) and (not stop or i <= 20) and (not delayed or i > 25) and not (stale and 80 < i < 90):
      b = [frame(a, counter % 256) for a in OBJECT_IDS] if empty else bank(counter % 256)
      frames.extend({'address': c.address, 'src': c.src, 'payload': c.dat.hex()} for c in b)
      counter += 1
    rows.append({'kind': 'event', 'service': 'can', 'timestamp_ns': ns, 'frames': frames})
    rows.append({'kind': 'health', 'timestamp_ns': ns, 'valid': not stop or i <= 24,
                 'alive': True, 'frequency_ok': True})
  if not truncated:
    rows.append({'kind': 'end', 'reason': 'duration', 'shadow_error': None})
  path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
  return path


@pytest.mark.parametrize('empty', [False, True])
def test_survival_including_empty_slots(tmp_path, empty):
  result = analyze(fixture(tmp_path/'capture', empty=empty))
  assert result['outcome'] == 'object_stream_survives_with_usable_updates', result
  assert result['post_banks'] > 40


def test_stream_stop_requires_successful_silencing_and_bus_coverage(tmp_path):
  result = analyze(fixture(tmp_path/'capture', stop=True))
  assert result['outcome'] == 'object_stream_stops_at_or_after_silencing', result


@pytest.mark.parametrize('options', [{'mode': False}, {'receipt': False}, {'factory_continues': True},
  {'pre': False}, {'stale': True}, {'truncated': True}, {'delayed': True}, {'wrong_bus': True}, {'background': False}])
def test_missing_evidence_never_passes(tmp_path, options):
  result = analyze(fixture(tmp_path/'capture', **options))
  assert result['outcome'] == 'inconclusive', result
  assert result['missing_evidence']


def test_partial_json_is_inconclusive(tmp_path):
  path = fixture(tmp_path/'capture')
  with path.open('a') as f:
    f.write('{"partial":')
  assert analyze(path)['outcome'] == 'inconclusive'


@pytest.mark.parametrize('failure', ['size_limit', 'negative_response', 'health'])
def test_explicit_failure_evidence_is_inconclusive(tmp_path, failure):
  path = fixture(tmp_path/'capture')
  rows = [json.loads(line) for line in path.read_text().splitlines()]
  if failure == 'size_limit':
    rows[-1]['reason'] = 'size_limit'
  elif failure == 'health':
    for row in rows:
      if row['kind'] == 'health' and row['timestamp_ns'] > 4_000_000_000:
        row['frequency_ok'] = False
  else:
    rows.insert(-1, {'kind': 'event', 'service': 'can', 'timestamp_ns': 9_000_000_000,
                    'frames': [{'address': 0x18DAF1B0, 'src': 0, 'payload': '037f2822'}]})
  path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
  result = analyze(path)
  assert result['outcome'] == 'inconclusive' and result['missing_evidence']


@pytest.mark.parametrize('scenario', ['not_armed', 'ready', 'readiness_timeout', 'launch_failed'])
def test_one_shot_boot_launcher_is_bounded_and_never_rearms(tmp_path, scenario):
  from types import SimpleNamespace

  from openpilot.tools.car_porting.bosch_c_coexistence_boot import launch_armed
  now = [0.]
  calls = []
  if scenario != 'not_armed':
    (tmp_path/'arm-next-startup').touch()

  def popen(args, **kwargs):
    calls.append(args)
    if scenario == 'launch_failed':
      raise OSError('synthetic failure')
    if scenario == 'ready':
      ready = args[args.index('--ready-file')+1]
      from pathlib import Path
      Path(ready).write_text(json.dumps({'pid': 123, 'output': args[args.index('--output')+1]}))
    assert kwargs['start_new_session'] and not kwargs.get('shell', False)
    return SimpleNamespace(pid=123, poll=lambda: None)

  result = launch_armed(tmp_path, popen=popen, clock=lambda: now[0],
                        sleep=lambda dt: now.__setitem__(0, now[0]+dt), timeout=.2)
  assert result['state'] == scenario
  assert now[0] <= .25
  assert len(calls) == (scenario != 'not_armed')
  assert not (tmp_path/'arm-next-startup').exists()
  assert launch_armed(tmp_path, popen=popen)['state'] == 'not_armed'


@pytest.mark.parametrize('empty', [False, True])
def test_coexistence_is_separate_from_failed_runtime_scheduling(tmp_path, empty):
  path = fixture(tmp_path/'capture', empty=empty)
  rows = [json.loads(line) for line in path.read_text().splitlines()]
  rows = [r for r in rows if not (r['kind'] == 'health' and 4_000_000_000 < r['timestamp_ns'] < 5_000_000_000)]
  path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
  result = analyze(path)
  assert result['outcome'] == 'inconclusive'
  assert result['coexistence_outcome'] == 'object_stream_survives_with_coherent_updates'
  assert not result['runtime_health_passed']
  assert result['max_shadow_health_gap_s'] == 1
  assert (result['post_active_object_observations'] == 0) == empty


def test_coexistence_cannot_pass_without_factory_silencing(tmp_path):
  result = analyze(fixture(tmp_path/'capture', factory_continues=True))
  assert result['coexistence_outcome'] == 'inconclusive'
  assert result['coexistence_missing_evidence']
