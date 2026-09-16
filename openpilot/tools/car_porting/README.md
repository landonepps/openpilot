# Bosch C passive radar diagnostics

`bosch_c_radar_logger.py` collects local diagnostics for the experimental Honda Bosch C adapter. It subscribes to receive CAN while the normal Honda interface continues running. It does not publish `radarData`, send CAN, change parameters, or alter lead selection.

The adapter is in `opendbc/car/honda/bosch_c_radar.py` in the opendbc submodule. It requires `HONDA_CRV_6G` and explicit candidate calibration. It is not registered with Honda's production interface. Physical units and measurement-validity rules are still under investigation.

## Comma four driving-view button

On comma four, a translucent **Radar log** button appears at the bottom center of the driving view for `HONDA_CRV_6G` while Honda retains longitudinal control. It is implemented in the `mici` UI and is not shown on comma 3/3X or other vehicles. Alerts temporarily hide the button.

Tap once to start and again to stop. The button shows **Starting**, then **Stop / wait** until valid radar banks arrive. During capture it shows **Stop m:ss** with a red indicator. **Saved** confirms a finished capture containing radar banks. **No radar** means no banks were captured. **Log error** or **Low storage** can be tapped to retry; details remain in the capture's local status/stderr files.

No calibration transfer or SSH command is needed to trigger the UI capture once this code is installed. The button passes the bundled `bosch_c_candidate_calibration.json`, an exact copy of the frozen research hypothesis, to the logger. It does not enable production radar support.

Files are saved under `/data/media/0/bosch_c_radar/`, alongside the normal route directory. Each capture gets a unique UTC-based filename, a `.jsonl` data file, a `.status.json` progress/result file and a `.stderr.log` failure log. These files are local and are not automatically uploaded as route recordings. Retrieve them with the corresponding rlog after the drive.

The UI launches decoding and file writing in a separate low-priority process. Capture stops on a second tap, when leaving the supported onroad configuration, when the UI exits, after ten minutes or at 256 MiB. New captures require at least 512 MiB of free space. A per-directory lock prevents concurrent UI/CLI captures. Old captures are not deleted automatically.

Touch tests verify that tapping does not open the home screen or trigger the bookmark gesture. Drags cancel the tap. The control is 124×42 in the 536×240 comma four view. Full live/on-device UI testing is still pending; see the research report for local rendering limitations.

## Live radar preview and bookmarks

During a capture, comma four shows a translucent 160×126 bird's-eye panel at the lower right of the camera view. It hides for alerts, blind-spot warnings and while scrolling away. This is a separate diagnostic view of the candidate object list, independent of Honda's HUD lead selection. No camera match is required. It does not identify object classes such as car, wall or pedestrian.

- The car is at the bottom of the map. The top is 100 m ahead, the middle line is 50 m, and the horizontal span is 10 m either side, all under provisional calibration. Positive candidate lateral position is drawn left.
- `Radar ~ 3/5` means three objects fit within the map and five structurally accepted tracks exist in total. Objects outside the plot are counted, not clamped to its edges. The stream has at most sixteen object slots; this is not a map of all radar reflections.
- Cyan dots pass the existing empirical display guard. Hollow amber dots fail that guard but remain visible if they fit within the map. Neither color is a recovered confidence score or object classification.
- A white ring highlights the nearest guard-passing object within the map and within 2 m of the centerline. Its track ID, estimated forward distance, relative speed and continuous track age appear beside the map. This geometric selection is not an ACC lead decision. Negative relative speed means closing under the candidate velocity convention. Track age is not a confidence score.
- `~` marks unvalidated physical calibration. The shown forward distance uses the frozen research offset; it is not yet a verified bumper-to-object or laser distance.

Status is published at most twice per second. `No fresh preview` replaces dots when the worker has no fresh valid bank, stops recording, or its status is more than 1.5 seconds old. An empty map does not establish that the space is clear.

Use the existing leftward bookmark swipe to mark an interesting moment while capture is active. It still creates the normal route bookmark and also saves a record in the capture's `.markers.jsonl` file. `Mark saved` confirms the separate marker write; `Mark failed` means it failed. Marker writes run off the UI thread, with at most one pending. Each record includes UTC, a timestamp using the rlog clock domain, the capture filename, and the last displayed object summary with its source timestamps and freshness flag. The snapshot can predate the gesture by the status-update delay; use the gesture timestamp to locate the exact raw frames. Retrieve the marker file together with the capture and rlog. Captures remain manually started and retain their ten-minute/256 MiB limits.

A wall is a possible measurement target only if an exported track can be associated with a known reflecting surface. A wall may yield no stable object, multiple tracks, or a return from an edge/post instead of the flat face. While parked, check repeatability at multiple known distances and record the laser's reference position relative to the car. Keep the raw capture even when the displayed estimate seems wrong. Independent distance/offset validation is still required.

## Running a passive capture

Run from this checkout in an environment with the normal sunnypilot IPC runtime, during a stock Honda longitudinal-control session. Supply the research project's `artifacts/calibration.json` or the bundled `openpilot/tools/car_porting/bosch_c_candidate_calibration.json` explicitly. The tool rejects sessions with `openpilotLongitudinalControl` enabled.

```sh
python -m openpilot.tools.car_porting.bosch_c_radar_logger \
  --calibration /path/to/calibration.json \
  --output /path/to/new-bosch-c-diagnostics.jsonl \
  --bus 1 --duration 600 --max-mib 256
```

The output path must be new. The defaults stop capture after ten minutes or 256 MiB. Ctrl-C preserves completed records. The tool has been tested with injected receive streams; live IPC and on-car operation still need validation. No capture starts automatically; it requires a button tap or the CLI command.

The first JSONL record contains the platform, recorded ECU firmware, calibration, adapter source hash and capture limits. Update records contain:

- Candidate `RadarData` when the adapter emits an update.
- Raw fields for every structurally accepted track, including objects excluded by the empirical display guard.
- Track IDs, observation times, bank age and pending-slot count.
- Integrity, counter, rejected-bank and timeout-related counters.
- Original Bosch C payloads received by this subscriber, including malformed/corrupt messages.

A delayed packet older than the last timeout heartbeat remains in the raw diagnostic record but is not decoded. `late_packets_not_decoded` records that loss. Original rlogs remain the authoritative capture; diagnostic files do not contain all vehicle CAN traffic. Capture timestamps are monotonic nanoseconds, suitable for alignment with rlogs.

## Behavior and limits

The adapter requires all sixteen slots for a bank, verifies the address-dependent CRC, checks counter/phase progression, and maintains identity across slot changes. It rejects duplicate-slot, duplicate-ID and invalid-ID banks. Repeated completed banks cannot refresh tracks. Objects expire after 200 ms, and timed empty updates emit error heartbeats after stream loss. Hardware radar-fault bits are not decoded.

Only `trackId`, `dRel`, `yRel` and `vRel` enter candidate `RadarData`. Quality and the additional motion fields remain diagnostic data. The existing empirical display guard is preserved; no new quality cutoff is enabled. The normal Honda interface, `radarUnavailable`, radar-disable sequence, RadarD and downstream consumers remain unchanged.

Before production activation, validate physical calibration and object validity, run passive vehicle captures, and resolve retention of the object stream during sunnypilot longitudinal control.

## Fork baseline and validation

The diagnostics branch uses MVL's `sp-honda-dev-202608` baselines: sunnypilot `46db408ea71908ab2559b385a7aad9fc85220422` and opendbc `254d6f150b21b82da9666b1000f7b99507f15c9a`. The submodule is pinned to the candidate decoder in `landonepps/opendbc`, branch `bosch-c-radar-diagnostics`.

After integrating those baselines, 88 Honda adapter, capture and research tests and five comma four touch tests passed. Ruff also passed for the changed Python files. On-device UI and live CAN capture validation remain pending.

The live-preview and bookmark extension passes 100 combined tests, including stale-status clearing, stationary objects, display-guard rejects, asynchronous marker success/failure, and preservation of the normal route bookmark. A synthetic software rendering of the production drawing calls checked layout with the production font. This does not replace a device/GPU UI test.
