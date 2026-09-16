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
