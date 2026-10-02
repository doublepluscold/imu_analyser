# imuview: IMU live viewer, logger and replay

Shows the orientation of a sensor module as a 3D quad in [Rerun](https://rerun.io),
logs everything at full rate, and (stage 2) replays sessions. Orientation only for now;
the code is laid out so GPS position can be added later without restructuring.

## Status

| Stage | What | State |
|---|---|---|
| 1 | simulator (+ fake GNSS), reference parser, NMEA/UBX framers, Mahony/Madgwick, logging, Rerun view | done, tested on sim only |
| 2 | MTData2 parser (default), listen-only serial port, host time reconstruction, `live`, `replay`, `reparse`, `import csv/raw`, `calib record`, MTData2 simulator | done; parser checked against the vendor decoder on real dumps, live port not yet run here |
| 2b | `sniff`, `--recompute-fusion` | not started |
| 3 | gyro PSD, Allan deviation, session report | not started |
| C | calibration: `calib check/gyro/axes/accel/level/report/apply`, `framing` | check, gyro, axes, framing and the apply layer (`--calib`) done, checked on sim + legacy dumps; accel fit, level, report, apply not yet. Steps for the user: `docs/CALIBRATION.md` |

## Run

```bash
uv sync
uv run imu live                                # the module on /dev/ttyUSB0, LISTEN-ONLY
uv run imu calib record --label still --seconds 60   # passive recording, no viewer
uv run imu replay logs/<session> [--calib calib/imu_calib.json]
uv run imu reparse logs/<session> [--calib ...]      # raw.bin -> new session
uv run imu import csv data/legacy/still.csv --label still     # files of mtdata2_decoder.py
uv run imu import raw data/legacy/still.bin --label still --duration 13.3
uv run imu sim --protocol mtdata2 --corrupt 0.2   # synthetic MTData2 with damaged frames
uv run imu sim                                 # 3 IMUs at 1 kHz, opens the Rerun viewer
uv run imu sim --n-imu 1 --with-fake-gnss      # also 5 Hz fake GNSS fixes
uv run imu sim --seconds 30 --no-viewer --rrd  # headless, saves record.rrd in the session
uv run imu info logs/<session>                 # session stats
uv run pytest                                  # tests
```

Keys while running: `c` calibrate gyro bias (keep still for 3 s), `t` tare,
`r` stop/start recording (a new session each time), `q` quit.

Settings: copy `config.example.toml` to `config.toml` and pass `--config config.toml`.

The simulator is still for 5 s (time to press `c`), then rotates slowly about all axes
for 12 s, then is still again for 4 s, and repeats.

## Files

```
src/imuview/
  messages.py   ImuSample, GnssFix, GnssRaw, RawBlock, Command, Event, State
  frames.py     ALL frame math (quaternions, euler, board alignment, tare, geodetic <-> NED)
  protocol.py   framers (ref, NMEA, UBX), demux, reference parser, encoder
  protocol_mtdata2.py  Xsens MTData2 framer + parser (wraps vendor/mtdata2_decoder.py), encoder
  calibration.py  host calibration file and its application (a_cal = M (a - b), w - bias)
  sources.py    SimSource (ref + mtdata2), SerialSource (listen-only), MasterSerialSource (master ESP
                over USB), UdpSource, RawFileSource,
                BytesSource (raw dumps), MessageSource + csv_messages (CSV imports)
  fusion.py     Mahony, Madgwick, passthrough, gyro calibration
  pipeline.py   config, Router, Pipeline (same code for live, sim and replay)
  logger.py     session folder: raw.bin, parquet streams, meta.json
  viewer.py     Rerun view
  cli.py        command line and keyboard
tests/          test_protocol.py, test_fusion.py, test_pipeline.py
```

Data flow: `Source -> Demux -> Parser -> Calibration -> BoardAlignment -> Router -> Estimator -> State -> Recorder / Viewer`.

## The real device: MTData2, listen-only

The module emulates an Xsens MTi: `FA FF 36 LEN payload CS` at 115200 8N1, about 60 valid
frames/s, see `docs/module-context.md`. `vendor/mtdata2_decoder.py` (verified on the device)
does the block decoding; `protocol_mtdata2.py` wraps it.

**Nothing ever writes to the port.** The same USART hosts an OTA bootloader and a command
parser. `SerialSource` has no write method, opens the port with DTR/RTS low and no flow
control, exclusively; `tests/test_listen_only.py` checks that on a pty with pyserial's write
patched to fail, and greps `src/` for send calls.

## Time without a device clock

The stream has no timestamp and no packet counter, so every message has `time_source`:
`device` (a device clock, e.g. the ref protocol or MTData2 SampleTimeFine if it ever appears),
`host`, or `synthetic` (made up from byte positions, for raw dumps without timestamps).
Host time of a frame = chunk arrival time minus (bytes after the frame in the buffer) x 10 / baud,
so frames of one USB chunk get distinct times. Filters use device time if present, else host
time clamped to [0.5, 3] x the running median interval; bigger gaps go to the event log.
The Rerun `time` timeline is device time if present, else host time (starting at 0).

## Frame conventions

- Body frame: **FRD** (x forward, y right, z down). World frame: local **NED**.
- Quaternion `q = (w, x, y, z)`, Hamilton, rotates body to world: `v_ned = q * v_frd * q^-1`.
- Euler angles: yaw, pitch, roll (ZYX), shown in degrees.
- Accelerometer = specific force: lying level and still it reads `(0, 0, -9.81)`.
- Board alignment `[roll, pitch, yaw]` (degrees) rotates sensor axes into body axes,
  `v_body = R(roll, pitch, yaw) @ v_sensor`. Examples: a sensor with z pointing up is
  `[180, 0, 0]`; a sensor turned 90 deg clockwise seen from above is `[0, 0, 90]`.
- Rerun gets NED directly and is told that z points down (`ViewCoordinates.RIGHT_HAND_Z_DOWN`).
  The only conversion at the viewer edge is quaternion order `(w,x,y,z) -> (x,y,z,w)`.
- **Yaw drifts.** Without a magnetometer nothing corrects heading. Gyro calibration (`c`)
  makes the drift slower, but it never stops. That is physics, not a bug.
- Tare (`t`) only changes what is displayed. `state.parquet` always has the un-tared attitude.

## Session folder: `logs/YYYY-MM-DD_HH-MM-SS/`

| File | Content |
|---|---|
| `raw.bin` | Every received byte, the source of truth. Records: `[pc_rx_time_ns: u64][len: u32][bytes]`, little-endian. Data we can't decode yet (e.g. GPS) is kept here. |
| `imu.parquet` | One row per IMU per sample: times (`mcu_time_us, pc_rx_time_ns, host_time_ns, time_source`), `seq, imu_id`, `accel_*, gyro_*` **exactly as sent** (sensor frame, never modified), `accel_cal_*, gyro_cal_*` (host calibration, if any), `accel_body_*, gyro_body_*` (after board alignment), `euler_deg_*, status, extra` (device Euler, status byte, other blocks as JSON). Gyro-key bias **not** removed. |
| `gnss_raw.parquet` | Undecoded GNSS blocks (MTData2 0x8840 GpsSol) as hex. |
| `raw_blocks.parquet` | Other blocks of frames without IMU data (none seen so far). |
| `state.parquet` | Estimator output: `mcu_time_us, pc_rx_time_ns, q_wxyz, estimator, source, pos_ned_xyz, vel_ned_xyz, covariance`. `source` is `vehicle` or `imu<k>` (per-IMU diagnostic filters). |
| `gnss.parquet` | GnssFix rows. MTData2: LatLon/Alt/Vel of every frame; `fix_type` 0 when status bit 2 is clear, and then the position is a placeholder (~29.9999 deg): never use it. |
| `commands.parquet` | Key presses (calibrate, tare), so a replay can repeat them at the same moment. |
| `events.jsonl` | The event log. |
| `meta.json` | Label, source, parsers, estimator, calibration file path + sha256, full config, conventions, git commit, start/stop time, stats (message counts and rates, time sources, checksum errors, bad frames per MID/LEN, extended frames, per-XDI counts and precision/frame bits, seq gaps, unclaimed bytes), gyro biases. |
| `record.rrd` | Rerun recording (only with `--rrd`). |

Columns that don't apply are null (not NaN). Each stream has its own file, no wide table.
While recording, the streams are written as `*.arrows` (Arrow IPC) and flushed every second,
so a crash loses at most ~1 s. They become `*.parquet` on a clean stop; `imu info` converts
files left over after a crash.

`mcu_time_us` is the MCU clock, unwrapped to 64 bit (the u32 wraps every 71.6 min).
`pc_rx_time_ns` is `time.monotonic_ns()` when the USB chunk arrived;
`meta.json["wall_minus_monotonic_ns"]` converts it to wall-clock time.

## Reference wire protocol

Used when we control the firmware. Little-endian.

```
0xA5 0x5A | len:u8 | type:u8 | seq:u16 | t_us:u32 | payload | crc16:u16
```

- `len` = bytes from `type` to the end of the payload = 7 + payload length.
- `crc16` = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over `len .. payload`.
- `seq` counts every frame (all types) on the link; gaps = lost frames.
- `t_us` = MCU microseconds when the sample was taken (not when it was sent).

| type | payload |
|---|---|
| 0x01 | `n_imu:u8`, then per IMU `ax ay az gx gy gz` as int16 (scale from config) |
| 0x02 | same, int32 |
| 0x03 | same, float32 in m/s² and rad/s |
| 0x04 | `n_imu:u8`, per IMU `ax ay az gx gy gz qw qx qy qz` float32 (device quaternion, sensor -> NED) |
| 0x10-0x1F | reserved for GNSS |
| 0x20-0x2F | reserved for baro / mag / other |

IMU `k` in a frame is `imu_id = k`, all sampled at the same `t_us`. Unknown types are counted
and skipped. Bandwidth at 921600 baud (92 kB/s): 3 IMUs × 1 kHz with 0x01 needs 49 kB/s (OK),
with 0x02/0x03 85 kB/s (too close to the limit).

If the real device uses another format, write a new parser class (see `RefParser`) and add it
to `PARSERS` in `protocol.py`. Nothing else changes.

## Desktop GUI (several modules, live / calibration / analysis)

    uv run imu gui                       # or: uv run imuview-gui
    uv run imu gui --source sim          # 3 simulated modules, no hardware
    uv run imu gui --session logs/<dir>  # open a recorded session (analysis mode)
    uv run python tools/udp_master_sim.py --modules 3 --gps   # stand-in for the master ESP (UDP)
    uv run python tools/usb_master_sim.py --modules 3 --gps --link /tmp/ttyIMU   # same, over USB

- Sources: the master ESP over **USB** (transport "Wi-Fi: майстер ESP (USB)", default port
  `/dev/ttyACM0`, `COMx` on Windows; no Ethernet cable needed), the same master over UDP
  (datagram format in `src/imuview/netproto.py`), USB-serial (one module straight from its UART,
  `module_id` 0, for debugging) and a simulator. All of them only listen.
- Master over USB: the port carries the same datagrams as UDP plus text lines starting with `# `
  (the master's diagnostics, with a per-slave diagnosis). The status bar shows what is wrong in
  words ("#2: UART мовчить", "жодного слейва не чути"). Headless: `uv run imu net --serial
  /dev/ttyACM0 --no-record`. The program opens the port with DTR/RTS low and never writes to it.
  If another program (a serial monitor) opens the port, the ESP32-S3 may reset; that is harmless.
- Modes: **Лайв** (3D model + plots, recording on by default), **Калібрація** (wizard:
  level, 5 more sides, gyro; writes `calib/imu_calib.json` and a report), **Аналіз**
  (recorded session, time slider, play / pause / speed).
- One session folder holds all modules: every parquet row has `module_id`, `raw.bin` records
  carry the module id in the top byte of the length field (old files read as module 0).
- 3D: orientation always; with a valid GPS fix the model moves (ENU metres from the first fix)
  and leaves fading ghost copies; without GPS a force arrow is drawn (gravity can be removed).
- Models: "Керувати моделями…" in the right panel opens the model manager (add an STL, duplicate,
  delete, scale, pivot, nose / top axes, preview). STL files are copied to the app data folder
  with `models.json`; the files in `models/` seed the library on the first run. Auto scale and
  centre by default.
- Look: dark blue theme with yellow accents, all tokens in `src/imuview/gui/theme.py`; fonts
  (Inter, JetBrains Mono, OFL) and Lucide icons are in `assets/`. Settings are kept between runs.
- Module names: `[modules.names]` in the config. Layouts: one module / grid / one scene.
- Linux under Wayland: the GUI forces the X11 plugin and GLX (`gl_common.configure_platform`),
  otherwise PyOpenGL reports "no valid context".
- Rerun stays as the engineering view for one module: `imu live | sim | replay`.
- Windows .exe: `uv run pyinstaller packaging/imuview.spec --noconfirm` ON Windows (see
  `packaging/imuview.spec`); the result is `dist/imuview.exe` with `models/` inside.

## Extension points for GPS

These already exist and are tested:

1. **`GnssFix` message** (`messages.py`) with fix type, satellites, lat/lon/height,
   NED velocity, accuracies, GPS time of week. Arrival time (`mcu_time_us`, `pc_rx_time_ns`)
   is separate from the time the fix is valid (`valid_time_us`), because fixes arrive
   tens to hundreds of ms late. `imu sim --with-fake-gnss` sends them (80 ms late, at 5 Hz)
   through routing, logging (`gnss.parquet`) and the viewer.
2. **Parser plugins.** NMEA and UBX frames are already found and counted. A GNSS parser is
   a class in `PARSERS` listed in `extra_parsers`; it receives the frames of its framer.
   Test: `test_extra_parser_plugin_turns_frames_into_gnss_stream`.
3. **Estimator interface.** Anything with `name`, `process(msg) -> State | None` and
   `params()` can replace the attitude filter. `State` has `pos_ned`, `vel_ned` and
   `covariance`; the viewer uses `pos_ned` as the vehicle translation and draws the
   trajectory. Test: `test_position_estimator_fills_state_and_viewer_translation`.
4. **Geodetic math.** `frames.geodetic_to_ned` / `ned_to_geodetic` (WGS84) are ready.
   `meta.json` has a `ned_origin` slot (first good fix or a configured point).
5. **raw.bin.** Everything received is kept, so old sessions can be re-parsed with a
   GNSS parser later (`imu reparse`, stage 2).

Not done on purpose: GNSS decoding, INS/EKF, map view, baro.

## Notes

- If ROS is sourced in your shell, its pytest plugins crash; `pyproject.toml` disables them.
- Performance (sim, 3 IMUs × 1 kHz, recording, Rerun at full rate): about 40% of one CPU
  core for the pipeline, plus the simulator itself. Sending to Rerun takes ~0.1 s per 1 s.
  Decimation (`plot_hz`, `transform_hz`) is available but off by default.
