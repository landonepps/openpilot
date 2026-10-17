import io
import json
import pytest

from opendbc.car import structs
from opendbc.car.honda.bosch_c_radar import BoschCRadarInterface
from opendbc.car.honda.tests.test_bosch_c_radar import CALIBRATION, bank, cp
from openpilot.tools.car_porting.bosch_c_radar_logger import diagnostic_record, radar_preview, run_capture, write_record


def test_record_keeps_raw_payloads_and_leaves_carparams_unchanged():
  params = cp()
  before = params.to_dict()
  adapter = BoschCRadarInterface(params, structs.CarParamsSP(), calibration=CALIBRATION)
  packets = [(0, bank())]
  record = diagnostic_record(adapter, packets)
  assert len(record['radar_data']['points']) == 1
  assert len(record['raw_frames']) == 16
  assert record['raw_frames'][0]['payload'] == packets[0][1][0].dat.hex()
  assert params.to_dict() == before
  json.dumps(record, allow_nan=False)


def test_capture_loop_writes_header_and_stream_with_bounded_duration():
  adapter = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION)
  ticks = iter([0., .1, .2, 2.])
  packets = iter([[(0, bank())], [(60_000_000, bank(1))]])
  output = io.BytesIO()
  result = run_capture(adapter, lambda: next(packets), output, duration_s=1., max_bytes=100_000, clock=lambda: next(ticks))
  records = [json.loads(line) for line in output.getvalue().splitlines()]
  assert result['reason'] == 'duration' and result['updates'] == 2
  assert result['bytes'] == len(output.getvalue())
  assert [r['kind'] for r in records] == ['session', 'update', 'update']
  assert records[0]['calibration']['x_reference_offset'] == CALIBRATION.x_reference_offset


def test_size_limit_never_writes_partial_json():
  output = io.BytesIO()
  count = write_record(output, {'kind': 'test'}, 0, 100)
  before = output.getvalue()
  assert write_record(output, {'big': 'a' * 100}, count, 100) is None
  assert output.getvalue() == before
  assert json.loads(before)['kind'] == 'test'


def test_late_delivery_after_timeout_retains_raw_without_reviving_track():
  adapter = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION, clock=lambda: 300_000_000)
  diagnostic_record(adapter, [(0, bank())])
  timeout = diagnostic_record(adapter, [])
  assert timeout['radar_data']['errors']['radarUnavailableTemporary']
  late = diagnostic_record(adapter, [(250_000_000, bank(1))])
  assert late['late_packets_not_decoded'] == 1 and len(late['raw_frames']) == 16
  assert not adapter.decoder.tracks


def test_graceful_stop_reports_finished_and_preserves_complete_records():
  adapter = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION)
  output = io.BytesIO()
  stopped = [False]
  progress = []

  def receive():
    stopped[0] = True
    return [(0, bank())]

  result = run_capture(adapter, receive, output, duration_s=10, max_bytes=100_000, clock=lambda: 0.,
                       should_stop=lambda: stopped[0], progress=progress.append)
  assert result['state'] == 'finished' and result['reason'] == 'stopped'
  assert result['banks'] == 1 and result['updates'] == 1
  assert [r['state'] for r in progress] == ['waiting', 'recording', 'finished']
  assert len(output.getvalue().splitlines()) == 2


def test_progress_returns_to_waiting_when_radar_stream_expires():
  adapter = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION, clock=lambda: 300_000_000)
  output = io.BytesIO()
  progress = []
  calls = [0]

  def receive():
    calls[0] += 1
    return [(0, bank())] if calls[0] == 1 else []

  result = run_capture(adapter, receive, output, duration_s=10, max_bytes=100_000, clock=lambda: 0.,
                       should_stop=lambda: calls[0] >= 2, progress=progress.append)
  assert [r['state'] for r in progress] == ['waiting', 'recording', 'waiting', 'finished']
  assert result['banks'] == 1


def test_preview_includes_stationary_and_guard_rejected_objects():
  adapter = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION)
  diagnostic_record(adapter, [(0, bank(velocity=1540))])
  preview = radar_preview(adapter)
  assert preview['tracks'][0]['v'] == 0
  assert preview['tracks'][0]['candidate']
  assert preview['tracks'][0]['x'] == CALIBRATION.convert(adapter.decoder.tracks[1].raw)[0]
  # Negative candidate distance is still diagnostic data, not silently dropped.
  diagnostic_record(adapter, [(60_000_000, bank(1, x=4096))])
  preview = radar_preview(adapter)
  assert len(preview['tracks']) == 1 and not preview['tracks'][0]['candidate']
  assert preview['tracks'][0]['age_s'] == pytest.approx(.06)
  assert preview['last_bank_ns'] == 60_000_000


def test_preview_is_empty_before_first_bank_and_after_expiry():
  adapter = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION, clock=lambda: 300_000_000)
  assert radar_preview(adapter)['tracks'] == []
  diagnostic_record(adapter, [(0, bank())])
  diagnostic_record(adapter, [])
  assert radar_preview(adapter)['tracks'] == []
