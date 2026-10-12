# BNO055 Kit — boot-time IMU recorder

A standardised, headless companion to a flight stack: a BNO055
I2C IMU daemon that starts **independently at boot** as its own systemd
unit, records every sample with a paired **CLOCK_REALTIME / CLOCK_MONOTONIC
nanosecond stamp**, and ships with an offline analysis script that aligns
its timeline against `candump -L` capture — compensating for
pipeline jitter — so the two logs can be correlated post-flight.

The calibration schema is the original 5-key BNO055 raw-unit format (the
same one the earlier bench tooling produced, now folded into this package
as `calibrate`/`bench`/`best`). The known-good calibration ships twice — as
`calibs/active.json` (the mutable active seed) and as
`calibs/factory_fallback.json` (the frozen known-good fallback) — so a
fresh Kit boots recording with the known-good cal and can always retreat to
it.

## Design doctrine

- **Better no data than wrong data**: the daemon refuses to record if the
  calibration is missing or invalid (exit `EX_CONFIG=78`, unit stays failed).
- **Monotonic ns for math, wall clock for archival**: every sample carries
  both stamps; a `clock_ok` flag latches false if CLOCK_REALTIME steps
  (NTP) mid-session.
- **Anchor pairs**: each session's `session.json` records a tight
  (wall, mono) anchor pair at start — the same reasoning as a 
  flight recorder's `sync.json` — so the IMU's monotonic timeline can be
  projected onto the candump wall-clock timeline.

## Layout

```
kit/
  src/bno055_kit/            importable package
    clock.py                   paired clock stamps + NTP-step (slew) detection
    calibration.py             load/validate/hash/save calibration + history/best
    sensor.py                  BNO055 wrapper (lazy blinka imports, reopen-safe)
    calibrate.py               interactive on-Kit calibration procedure
    bench.py                   shared bench metrics (bias/drift/noise/score)
    recorder.py                session dirs: imu.jsonl + session.json manifest
    daemon.py                  fixed-rate record loop, signals, reconnect policy
    cli.py                     scan | run | calibrate | bench | status
    analysis/candump.py        candump -L parser + HirrusUAS IMU frame decode
    analysis/sync.py           IMU<->CAN clock alignment (offset + jitter)
  scripts/deploy/
    systemd/bno055-imu.service     independent boot unit (multi-user.target)
    systemd/bno055_imu_wrapper.sh  systemd exec-wrapper
    install_kit.sh                 dry-run-default installer (own venv)
  scripts/ops/
    start_kit.py                   start/stop/status/logs + calibrate/best/fallback
    health_check.py                one-shot Kit health report
  scripts/analysis/
    sync_candump.py                align a session with a candump capture
    compare_session.py             sync + per-preset side-by-side BNO/CAN compare
  configs/bno055_imu.yaml          daemon config (installed to /etc/bno055)
  calibs/                          active seed + frozen factory fallback
                                   (cal_<ts>.json sessions + history.jsonl are
                                   generated per instance, never tracked)
  tests/                           hardware-free unit tests (pytest)
```

## On-Kit runtime layout

| Path | Purpose |
|---|---|
| `/opt/bno055` | app root, own venv at `/opt/bno055/.venv` |
| `/var/lib/bno055/active.json` | **active** calibration — mutable working copy the daemon loads; `calibrate`/`best`/`fallback` overwrite it at will (`StateDirectory`) |
| `/var/lib/bno055/factory_fallback.json` | **factory fallback** — frozen known-good cal, root-owned mode `0444`, re-installed on every install, never written or deleted by any tool |
| `/var/log/bno055/<session>/` | `imu.jsonl` samples + `session.json` manifest (`LogsDirectory`) |
| `/etc/bno055/bno055_imu.yaml` | daemon config |
| `/run/bno055` | runtime dir (`RuntimeDirectory`) |

The unit runs as the **service user/group you choose at install time** (there
is no built-in default: `install_kit.sh` takes it from `--user`/`--group`,
else auto-detects the sudo caller via `SUDO_USER`, else prompts you to type
it) with `SupplementaryGroups=i2c`, and is **independent of any other
flight-stack service** — the IMU timeline must cover the whole boot, not only
their lifetime.

## Install (on the Kit)

The installer requires `rsync`; install it first if it is not already
available:

```bash
sudo apt-get install rsync
```

The service user/group are **mandatory and have no default**: pass them
explicitly with `--user`/`--group`, or omit the flags and the installer takes
them from the sudo caller (`SUDO_USER`) automatically — and if it cannot
auto-detect them, it prompts you to type them in real time.

```bash
sudo bash scripts/deploy/install_kit.sh --user $USER            # dry-run, shows every step
sudo bash scripts/deploy/install_kit.sh --user $USER --confirm  # apply
sudo systemctl start bno055-imu.service                         # or just reboot
```

Install **enables** the unit for boot but does not
start it; the boot path is the tested path.

## Calibrate

**Two files, two roles.** `active.json` is the mutable working copy the
daemon loads — only `best` and `fallback` ever overwrite it.
`factory_fallback.json` is the frozen known-good calibration: root-owned,
mode `0444`, re-installed on **every** install, and never written or
deleted by any tool — so it survives any number of bad on-Kit sessions and
is always available as the retreat path (and as the baseline `best` must
beat).

**Default flow (unchanged):** at first install `active.json` is seeded from
the shipped seed (only if absent — an on-Kit result is never overwritten)
and the daemon *loads* it at every start. The daemon never auto-calibrates
in flight.

**On-Kit (sensor already glued to the drone)** — the same 3-step bench
procedure, interactive, English prompts:

```bash
sudo python3 scripts/ops/start_kit.py calibrate   # stops the unit, guides
                                                  # the procedure, ALWAYS
                                                  # restarts recording
sudo python3 scripts/ops/start_kit.py best        # rank history, bench
                                                  # factory, promote the
                                                  # winner if it beats it
sudo python3 scripts/ops/start_kit.py fallback    # none of the sessions
                                                  # good enough? restore
                                                  # the factory cal
```

The ops scripts re-exec themselves under `/opt/bno055/.venv/bin/python`
automatically, so plain `sudo python3 ...` works on the Kit (sudo resets
PATH to the system python, which lacks the kit's deps).

Procedure per session: gyro at rest → six accel faces validated against
gravity (magnitude, dominant axis, jitter, opposite-sign pair) → accel/mag/
sys convergence (figure-8 for the magnetometer) → offsets captured to
`/var/lib/bno055/cal_<UTC ts>.json` and appended to the Kit-local
`/var/lib/bno055/history.jsonl`. An optional 30 s bench scores the fresh
calibration. **`calibrate` never touches `active.json`** — a fresh session
only becomes active through `best`. Ctrl-C at any point aborts without
saving — and the recorder unit is restarted either way.

Every prompt shows a **live preview line** while it waits for [Enter]: the
sensor is sampled continuously, so the line reads green `READY: ...` only
when the current pose/levels would actually pass that step's gates
(yellow `adjust: <why>` / `wait: <levels>` otherwise) — a wrong position is
caught *before* wasting the 2.5 s hold, not after three failed attempts.
A face skipped after 3 attempts also says *why*: a wrong pose blames the
position ("reposition and run again — the sensor itself is fine"), never
the calibration; only a correct pose with bad magnitude blames the sensor.

`best` is the only promoter. It first ranks the scored sessions in
`history.jsonl` to pick a *candidate*, then decides by **benching the top
session and the factory fallback live, back-to-back, under identical
conditions** (same minute, same motion — `--time` sets the window, 30 s
default). The session is promoted to `active.json` **only if its live score
strictly beats the factory's live score**; otherwise the factory cal stays
active and nothing is written (a tie keeps the frozen known-good cal). Both
candidates are re-benched every run — a stored score only picks the
candidate, it never wins the comparison — so the two numbers are always
apples-to-apples.

Each bench **re-converges first**: applying a calibration round-trips the
chip through CONFIG_MODE, which resets the live fusion, and the
magnetometer only re-converges while the field *changes* — so the operator
figures-8 until `mag=3`, then holds **perfectly still** for the window. A
window is rejected (not scored) unless `accel=3` and the heading is *live*
(many distinct samples, not frozen at one value): a frozen heading scores a
fake-perfect ~0 and would poison the comparison. `mag` is deliberately not
required during the still window — it decays to 0 the instant the board
stops moving, so demanding `mag=3` while still is impossible.

`bench` scores any calibration file at rest (`bias + |drift| + noise`,
lower is better) and appends to the same `history.jsonl`; it is the shared
scoring that `calibrate` and `best` both run through, so every score in the
history is directly comparable. Exposed on the Kit as
`sudo python3 scripts/ops/start_kit.py bench [--cal-file F] [--time N]`.

## Time-sync a session against a candump capture

```bash
python3 scripts/analysis/sync_candump.py \
    /var/log/bno055/bno_20261001T111015Z \
    /mnt/nvme-app/flight_sessions/<session>/can.candump \
    --out sync.json
```

Two methods, tried in trust order (`--method auto` default):

1. **correlation** (preferred): decodes the HirrusUAS accel/gyro frames
   (IDs 0x386/0x387, DBC v5 scaling) from the capture, resamples both
   magnitude series onto a common grid, and cross-correlates to recover the
   true pipeline offset — no shared clock assumed.
2. **anchor** (fallback): takes the residual from each sample's real wall
   stamp to the nearest candump frame; assumes one shared CLOCK_REALTIME
   (an NTP step inside the session shows up in the residual spread and the
   `clock_ok` note).

Output manifest: `offset_s` (add to IMU time to reach CAN time),
`jitter_std_s`, `max_abs_residual_s`, `n`, `method`, `confidence`, `notes`.

Timestamp contract: `candump -L` prints **local** wall-clock time; the
parser interprets it as local time (the Kit and the analysis host must
share a timezone, or use `candump -t a` epoch captures, which are also
accepted).

## Watch a live session (`imu.jsonl`)

Every recorded sample is one JSON object per line in `imu.jsonl`. To watch the
recorder live, pretty-print the stream with `jq` — the fields are the
quaternion (`qw`/`qx`/`qy`/`qz`), the fused orientation (head / roll / pitch,
degrees), the linear acceleration and gyroscope vectors (gravity-stripped
acceleration and angular rate, m/s^2 and rad/s), the per-subsystem calibration
status vector `cal` (accel / gyro / mag / system), the normalised calibration
confidence `c` (mean of `cal`, 0..1), and the `clock_ok` flag (false once an
NTP step was detected mid-session):

```bash
tail -f /var/log/bno055/<session>/imu.jsonl | jq -r --unbuffered '
  "\u001b[H\u001b[J" +
  "QUATERNION\n" +
  "  qw: \(.qw)\n" +
  "  qx: \(.qx)\n" +
  "  qy: \(.qy)\n" +
  "  qz: \(.qz)\n" +
  "\n" +
  "ORIENTATION\n" +
  "  head:  \(.head)\n" +
  "  roll:  \(.roll)\n" +
  "  pitch: \(.pitch)\n" +
  "\n" +
  "LINEAR ACCELERATION: \(.lin_acc | join(", "))\n" +
  "GYROSCOPE:          \(.gyro | join(", "))\n" +
  "CALIBRATION: \(.cal | join(", "))\n" +
  "CONFIDENCE:  \(.c)\n" +
  "CLOCK OK:    \(.clock_ok)\n"
'
```

`\u001b[H\u001b[J` is the ANSI home-and-clear sequence, so each sample redraws
in place like a live dashboard. `--unbuffered` makes `jq` flush every object
as it arrives instead of batching.

## Develop / verify (host, no hardware)

```bash
pip install -e .[dev]
ruff check .
pytest -q
```

The test suite is hardware-free: calibration validation, slew detection,
candump parsing, and offset recovery on synthetic shifted series.
