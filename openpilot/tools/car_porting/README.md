# Experimental Honda Bosch C radar

This branch supports opt-in Bosch C radar input on the Honda CR-V 6G with Bosch CAN-FD and existing comma longitudinal control. It uses the normal Honda RadarInterface, card publication and RadarD fusion. The tested receive layout is radar bus 1. Fresh installations default to disabled; an existing `HondaBoschCExperimentalRadar` setting survives an ordinary update.

Coexistence, native parked publication and a first engaged road test have passed on a 2026 CR-V and comma four. The first drive included 138 seconds of active longitudinal control and speeds up to about 65 mph, without recorded radar/CAN/ACC faults. This is limited experimental validation, not production readiness or proof of collision avoidance.

## Install and control through the device UI

The fork is `landonepps/openpilot`, branch `sp-honda-dev-202608`. Its opendbc submodule points to `landonepps/opendbc`, branch `bosch-c-radar-diagnostics`, at the exact commit recorded by the parent repository. Publish the opendbc commit first, then the openpilot commit. A parent update with an unavailable submodule commit cannot install.

On a device already using this fork and branch:

1. Keep the car parked and the comma powered, connected to the internet, and in its **offroad** state. Park alone does not imply offroad; accessory power may still report ignition on. The Software update button is disabled while onroad.
2. Open **Settings → Software**, check for updates, then download the update. Confirm the target branch is `sp-honda-dev-202608`.
3. Use **install now** when ready. Let the normal updater and launcher install, rebuild and restart. Do not edit source files after downloading an update; the updater's modification check can prevent activation.
4. Check the new commit in Software. Existing alpha-longitudinal and experimental-radar settings remain as configured.

On comma four, **Settings → Developer → Bosch C radar** manages the opt-in. It is shown for the supported, already-enabled longitudinal configuration, or when the radar setting is already saved so it can be disabled. It is offroad-only. Enabling requires confirmation. Changes request the normal onroad cycle and take effect at car initialization. After enabling alpha longitudinal, complete its restart first so CarParams reflects that mode. The radar switch does not enable alpha longitudinal itself.

To roll back the radar input, turn off **Bosch C radar** while offroad and let the requested restart complete. Alpha longitudinal is unaffected. Restoring an older whole build also rolls back unrelated changes; retain the tested revision and any device-local work before updating.

## Full-trip diagnostics are automatic

The standard logger records the following in `/data/media/0/realdata/<route>--<segment>/rlog.zst` throughout normal driving. There is no ten-minute radar limit in these logs and no need to tap **Radar log**.

| Evidence | Logged services |
|---|---|
| Original payloads, bus IDs, object counters/CRC, factory RX versus transmit receipts | `can`, `sendcan` |
| Selected integration, longitudinal mode, safety configuration, firmware | `carParams`, `carParamsSP` |
| Actual radar publication validity, errors, object IDs and measurements | `radarTracks` |
| Fused leads, whether a lead uses radar, radar track ID | `radarState` |
| Motion, driver inputs, CAN and ACC faults | `carState` |
| Engagement, requested acceleration, planned acceleration and source | `selfdriveState`, `carControl`, `carOutput`, `longitudinalPlan` |
| Vision association, process health, device state, software provenance | `modelV2`, `managerState`, `deviceState`, route `initData` |

Events retain their boot-clock timestamps. Raw frames allow the decoder's integrity/cadence checks to be reconstructed offline; its internal counters are not a separate continuously published service. Empty radar points with valid messages are distinct from a missing or stale stream. No extra full-trip recorder or second radar decoder is started by this integration.

Preserve **full rlogs**, including segment 0 and every segment of the trip. Qlogs alone are insufficient: raw CAN is heavily decimated and radarTracks is not included there. The normal storage manager removes older routes when space is low; local recordings are not a permanent archive. Retrieve logs soon after a useful trip. Existing bookmarks help identify moments but do not guarantee indefinite retention. Note the approximate time and observed behavior after parking. Original logs and footage are private data and must not be committed to GitHub.

The first engaged test was audited from these same standard route logs. Longer trips can therefore be analyzed without running the temporary observer. Normal upload settings are unchanged; this change does not request automatic upload of full logs or private footage.

## Additional bounded captures

The comma four **Radar log** button remains available under either existing longitudinal configuration. It runs a receive-only logger in a separate low-priority process. A tap starts or stops it. **Stop / wait** means no usable bank yet; **Stop m:ss** means recording; **Saved**, **No radar**, **Log error** and **Low storage** report its result. Alerts hide the button. A capture stops after ten minutes, at 256 MiB, on leaving the supported onroad configuration, or when the UI exits. It requires at least 512 MiB free, excludes concurrent captures and never deletes old captures.

These extra files live in `/data/media/0/bosch_c_radar/` and are not automatically uploaded. Keep the `.jsonl`, `.status.json`, `.stderr.log` and optional `.markers.jsonl` with matching route logs. **Radar log** returning to idle does not disable radar input or stop normal route logging.

During this extra capture, the bird's-eye preview shows structurally accepted objects under the working calibration. Cyan points pass the empirical display guard; hollow amber points do not. The white ring and **CENTER CANDIDATE** badge select the nearest guard-passing object within 2 m of the centerline and 100 m ahead. This geometric selection is independent of the actual fused lead. The map uses positive-left lateral position, at most sixteen slots, and a 1.5-second status freshness limit. An empty map does not establish that the road is clear. The `~` label reflects the experimental preview; range, velocity and lateral use the accepted working conversions.

The usual bookmark gesture still creates a normal route bookmark. During a manual radar capture it also saves a timestamped preview snapshot asynchronously. The snapshot may be older than the gesture; use the event timestamp and full rlog for analysis.

CLI alternative, from the normal sunnypilot runtime:

```sh
python -m openpilot.tools.car_porting.bosch_c_radar_logger \
  --calibration openpilot/tools/car_porting/bosch_c_candidate_calibration.json \
  --output /data/media/0/bosch_c_radar/new-capture.jsonl \
  --bus 1 --duration 600 --max-mib 256
```

The output must be new. Both existing longitudinal modes are supported for passive capture. The logger never changes mode, sends CAN or publishes control input.

## Startup coexistence tooling

`bosch_c_coexistence.py` captures original receive traffic, transmit requests/receipts, both clock domains, source revisions/hashes, actual radar publications and a disk-only shadow adapter. To catch the existing silencing sequence, explicitly arm once while parked:

```sh
mkdir -p /data/media/0/bosch_c_radar
touch /data/media/0/bosch_c_radar/arm-next-startup
```

At the next launcher startup, the hook consumes the marker and starts a detached recorder before manager. It waits at most five seconds for readiness, then allows normal startup even on failure. With no marker it does nothing. Default capture is 180 seconds/128 MiB, with a shared capture lock and bounded subscription draining. The mici button shows **REC banks** or **REC wait** and is status-only during this run. Valid empty banks count as stream presence. Failure details remain in the `.launch.json` and stderr files.

```sh
python -m openpilot.tools.car_porting.bosch_c_coexistence_analyze \
  /data/media/0/bosch_c_radar/startup-CAPTURE.jsonl \
  --output /tmp/coexistence-verdict.json
```

The analyzer separates transport coexistence from shadow scheduling health. It requires consistent streamed comma-longitudinal CarParams, a returned disable receipt, pre-disable factory ACC and object-bank baselines, factory RX disappearance, ongoing unrelated traffic on the mapped buses, and coherent post-disable banks. A transmitted request alone is not success. Truncation, rejected TX, delayed startup, stale data or missing evidence produce an inconclusive result. A suppressed positive UDS response is not required. Shadow data never enters a live publisher.

`bosch_c_drive_observer.py` is also included for short, explicitly launched receive-only checks. It records decimated health summaries and route IDs; it stops after 10 minutes of movement by default, sustained parking, or its size limit. It is not an always-on service and does not replace full rlogs.

## Runtime contract and remaining limits

The accepted working conversions are `dRel = 0.05 * (rawX - 4096) - 4.296`, `vRel = 0.1 * (rawV - 1539)` and `yRel = 0.01 * signed13(rawY)` (positive left). The lateral scale changed from 1/128 to 0.01 m/count on 2026-09-28 after gyro, camera and azimuth-field checks agreed; replay showed no change in braking.

The decoder verifies sixteen-slot banks, address-dependent CRC, counters, phase and identity consistency. Duplicate complete banks cannot refresh tracks. The live wrapper uses Linux boot time, rejects old/future-clock receive batches and expires data after 200 ms. Lost data produces empty invalid publications through the existing path, with normal downstream health protections. Hardware radar-fault bits are not decoded.

RadarD fusion, lead selection, planner/controller behavior, panda safety and radar silencing are unchanged. At highway speed, normal radar lead matching still requires vision agreement. Setting radarUnavailable=False also affects the existing radar-aware Dynamic Experimental Control branch if the user enables that separate feature; this integration does not enable it.

Focused tests, without starting the native UI:

```sh
python -m pytest opendbc_repo/opendbc/car/honda/tests \
  openpilot/tools/car_porting/test_bosch_c_*.py \
  openpilot/selfdrive/ui/tests/test_bosch_c_setting.py -q
```

The prior live adapter passed native compilation and parked/engaged publication checks. The newly added settings callback is covered headlessly; its physical comma four rendering still needs verification after the normal update.
