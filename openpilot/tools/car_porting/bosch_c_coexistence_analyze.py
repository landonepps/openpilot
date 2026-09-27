"""Conservative coexistence verdict from passive JSONL, never from TX alone."""
import argparse
import hashlib
import json
from pathlib import Path

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.bosch_c_radar import BoschCRadarInterface, CandidateCalibration, OBJECT_IDS
from opendbc.car.honda.hondacan import CanBus
from openpilot.tools.car_porting.bosch_c_radar_capture import CALIBRATION_PATH

REQUEST = 0x18DAB0F1
RESPONSE = 0x18DAF1B0
DISABLE = bytes.fromhex('03288303')
ACC_CONTROL = 0x1DF


def records(path):
  with Path(path).open() as f:
    for line in f:
      try:
        yield json.loads(line)
      except (ValueError, UnicodeError):
        yield {'kind': 'damaged'}
        return


def analyze(path):
  # Import the normal shared schema, never independently capnp.load it.
  from openpilot.cereal import log
  cp = None
  params = []
  footer = None
  damaged = False
  for r in records(path):
    damaged |= r['kind'] == 'damaged'
    if r['kind'] == 'end':
      footer = r
    if r.get('service') == 'carParams':
      with log.Event.from_bytes(bytes.fromhex(r['event_hex'])) as e:
        params.append((e.carParams.carFingerprint, e.carParams.openpilotLongitudinalControl, r['valid']))
        cp = e.carParams.as_builder()
  missing = []
  with Path(path).open('rb') as source:
    capture_hash = hashlib.file_digest(source, 'sha256').hexdigest()
  result = {'capture_sha256': capture_hash,
            'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'outcome': 'inconclusive', 'coexistence_outcome': 'inconclusive', 'missing_evidence': missing, 'criteria':
            'At least 5 s after disable, factory RX absent, continuous receive buses, coherent fresh banks and shadow health.'}
  if damaged or footer is None or footer.get('reason') not in ('duration', 'stopped'):
    missing.append('Complete capture with normal footer; truncated/size-limited data cannot prove absence')
  if cp is None or any(p != ('HONDA_CRV_6G', True, True) for p in params) or cp.brand != 'honda':
    missing.append('Current, consistent streamed HONDA_CRV_6G CarParams with openpilotLongitudinalControl=True')
    return result
  buses = CanBus(cp)
  if not 0 <= buses.pt < 128 or not 0 <= buses.radar < 128:
    missing.append('Valid CarParams safetyConfigs for receive-bus mapping')
    return result
  adapter = BoschCRadarInterface(cp, structs.CarParamsSP(), bus=buses.radar,
    calibration=CandidateCalibration(**json.loads(CALIBRATION_PATH.read_text())['parameters']))
  receipts, attempts, responses, factory, objects, banks, health = [], [], [], [], [], [], []
  active_banks = []
  background = {buses.pt: [], buses.radar: []}
  late = 0
  invalid_can = 0
  last = 0
  for r in records(path):
    ns = r.get('timestamp_ns', 0)
    if r['kind'] == 'health':
      health.append(r)
    if r.get('service') not in ('can', 'sendcan'):
      continue
    invalid_can += r['service'] == 'can' and not r.get('valid', True)
    frames = []
    for f in r['frames']:
      address, src, data = f['address'], f['src'], bytes.fromhex(f['payload'])
      if address == REQUEST and data.startswith(DISABLE):
        if r['service'] == 'sendcan' and src == buses.pt:
          attempts.append(ns)
        if r['service'] == 'can' and src == buses.pt + 128:
          receipts.append(ns)
      if r['service'] != 'can':
        continue
      frames.append(CanData(address, data, src))
      if src == buses.pt and address == RESPONSE:
        responses.append({'timestamp_ns': ns, 'payload': data.hex()})
      if src == buses.pt and address == ACC_CONTROL:
        factory.append(ns)
      if src == buses.radar and address in OBJECT_IDS:
        objects.append(ns)
      if src in background and address not in (*OBJECT_IDS, ACC_CONTROL, REQUEST, RESPONSE):
        background[src].append(ns)
    if r['service'] == 'can':
      last = max(last, ns)
      if adapter.decoder.now_ns is not None and ns < adapter.decoder.now_ns:
        late += 1
        continue
      before = adapter.decoder.counters['accepted_banks']
      rr = adapter.update([(ns, frames)])
      if adapter.decoder.counters['accepted_banks'] > before:
        banks.append((ns, bool(rr is not None and not rr.errors.canError)))
        active_banks.append((ns, len(adapter.decoder.tracks), len(rr.points) if rr is not None else 0))
  result.update(receive_buses={'powertrain': buses.pt, 'objects': buses.radar}, disable_attempts=attempts,
                disable_tx_receipts=receipts, ecu_responses=responses, decoder_counters=dict(adapter.decoder.counters),
                late_can_packets=late, shadow_error=None if footer is None else footer.get('shadow_error'))
  if invalid_can:
    missing.append('Invalid CAN events require investigation')
  if not receipts:
    missing.append('Panda returned TX receipt for existing 28 83 03 on the mapped powertrain bus')
    return result
  # Choose a receipt with a nearby factory-RX transition, not merely the last retry.
  candidates = [t for t in receipts if any(t-1e9 <= f <= t for f in factory) and
                not any(f > t+500_000_000 for f in factory)]
  t = candidates[0] if candidates else receipts[0]
  end = last
  start = t + 1_000_000_000
  if end - start < 5e9:
    missing.append('At least five seconds of post-transition receive CAN')
  pre_factory = [f for f in factory if t-1e9 <= f <= t]
  if len(pre_factory) < 10 or t-max(pre_factory, default=0) > 100_000_000 or max(pre_factory, default=t)-min(pre_factory, default=t) < 250_000_000:
    missing.append('Factory ACC_CONTROL receive baseline before disable')
  if any(f > t+500_000_000 for f in factory):
    missing.append('Sustained disappearance of factory ACC_CONTROL RX, excluding replacement TX receipts')
  if any(bytes.fromhex(r['payload']).startswith(bytes.fromhex('037f28')) for r in responses if r['timestamp_ns'] >= t):
    missing.append('CommunicationControl received a negative ECU response')
  pre_banks = [ns for ns, good in banks if good and t-1e9 <= ns <= t]
  if len(pre_banks) < 5 or t-max(pre_banks, default=0) > 200_000_000:
    missing.append('At least five coherent object banks immediately before disable')
  for bus, times in background.items():
    # Non-object/non-ACC traffic proves that bus reception itself did not stop.
    window = [start, *[ns for ns in times if start <= ns <= end], end]
    if max((b-a for a, b in zip(window, window[1:], strict=False)), default=10**10) > 1e9:
      missing.append(f'Continuous unrelated receive traffic on bus {bus}, maximum gap 1 s')
  post = [ns for ns, good in banks if good and start <= ns <= end]
  post_frames = [ns for ns in objects if start <= ns <= end]
  gaps = [b-a for a, b in zip([start, *post], [*post, end], strict=True)]
  relevant_health = [h for h in health if t+3e9 <= h['timestamp_ns'] <= end]
  bad_health = sum(not (h['valid'] and h['alive'] and h['frequency_ok'] and not h.get('stale_points', False)) for h in relevant_health)
  result.update(transition_ns=t, post_seconds=max(0, (end-start)/1e9), pre_banks=len(pre_banks),
                post_banks=len(post), post_object_frames=len(post_frames),
                post_banks_with_objects=sum(n > 0 for ns, n, _ in active_banks if start <= ns <= end),
                post_active_object_observations=sum(n for ns, n, _ in active_banks if start <= ns <= end),
                post_candidate_point_observations=sum(n for ns, _, n in active_banks if start <= ns <= end),
                last_factory_acc_rx_ns=max(factory, default=None),
                max_bank_gap_s=max(gaps, default=0)/1e9, health_samples=len(relevant_health), health_failures=bad_health,
                silencing_evidence='factory RX disappearance; suppressed positive UDS response is not required')
  # Keep transport coexistence separate from the stricter live-runtime verdict.
  # This does not waive any evidence or change the overall readiness outcome.
  coexistence_missing = list(missing)
  if post_frames and (len(post) < 40 or max(gaps) > 200_000_000):
    coexistence_missing.append('Continuous coherent post-disable banks with maximum gap 200 ms')
  if late:
    coexistence_missing.append('CAN ordering discontinuity requires investigation')
  result['coexistence_missing_evidence'] = coexistence_missing
  if not coexistence_missing:
    result['coexistence_outcome'] = ('object_stream_survives_with_coherent_updates' if post_frames else
                                     'object_stream_stops_at_or_after_silencing')
  health_times = [t+3e9, *[h['timestamp_ns'] for h in relevant_health], end]
  health_gap = max(b-a for a, b in zip(health_times, health_times[1:], strict=False))
  result['max_shadow_health_gap_s'] = health_gap / 1e9
  result['runtime_health_passed'] = bool(len(relevant_health) >= 30 and not bad_health and
                                        health_gap <= 250_000_000 and not (footer or {}).get('shadow_error'))
  if health_gap > 250_000_000:
    missing.append('Continuous live shadow scheduling after initialization, maximum gap 250 ms')
  if missing:
    return result
  if not post_frames:
    if len(relevant_health) < 30 or any(h['valid'] or h.get('points', 0) for h in relevant_health) or footer.get('shadow_error'):
      missing.append('Live shadow must report stale/invalid data after object-stream loss')
      return result
    result['outcome'] = 'object_stream_stops_at_or_after_silencing'
    return result
  if len(post) < 40 or max(gaps) > 200_000_000:
    missing.append('Continuous usable post-disable banks with maximum gap 200 ms')
  if (len(relevant_health) < 30 or bad_health or footer.get('shadow_error') or
      health_gap > 250_000_000):
    missing.append('Healthy serialized live shadow at 20 Hz after initialization, without scheduling gaps or shadow errors')
  if late:
    missing.append('CAN ordering discontinuity requires investigation')
  if not missing:
    result['outcome'] = 'object_stream_survives_with_usable_updates'
  return result


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('capture', type=Path)
  ap.add_argument('--output', type=Path)
  args = ap.parse_args()
  result = json.dumps(analyze(args.capture), indent=2)
  print(result)
  if args.output:
    args.output.write_text(result + '\n')


if __name__ == '__main__':
  main()
