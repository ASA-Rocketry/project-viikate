# Software-in-the-loop simulation

The SIL simulation builds the unmodified firmware in `code/app` for Zephyr's
`native_sim` board and connects it to a plant model through simulated IMU,
barometer and PWM devices. It runs in deterministic simulated time: the plant
steps at 1 ms, the firmware reads the published sensors and writes servo PWM,
and the next plant step uses those commands.

All commands below are run from the repository root. Python, Zephyr and the
visualization libraries come from Nix. Either enter the shell once with
`nix develop`, or prefix each command with `nix develop -c` as shown below.
See [codebase setup](codebase-setup.md) if the environment is not installed.

## Running without visualization

Build is automatic for `run`; it writes to `build/sil-<plant>/`. Use
`--no-build` only when the source has not changed and the existing binary is
known to be current.

```bash
# Build explicitly (optional)
nix develop -c python3 code/sim/sil.py build
nix develop -c python3 code/sim/sil.py build --plant toy

# Fly the default rocket design for 20 simulated seconds, print CSV to stdout
nix develop -c python3 code/sim/sil.py run --stop-at 20

# Save the same run to a CSV file
nix develop -c python3 code/sim/sil.py run --stop-at 20 --csv flight.csv

# Override parameters and choose a design
nix develop -c python3 code/sim/sil.py run --rocket viikate_v1 --stop-at 20 \
  --plant-param thrust_scale=0.9 --plant-param gust_variance=0 \
  --csv flight-thrust-90.csv

# One-axis roll test stand (kicks 1 rad/s at 2.5 s by default)
nix develop -c python3 code/sim/sil.py run --plant toy --stop-at 6 \
  --plant-param kick_roll_rate_radps=2

# Provenance check + build both plants and the replay tool + full test suite
nix develop -c python3 code/sim/sil.py test
```

`sil.py run` prints a CSV header and one `SIM` row every 10 ms of simulated
time by default (`CONFIG_VIIKATE_SIM_LOG_PERIOD_MS`) when `--csv` is omitted.
It also prints `SIMROCKET,...` with the design name, `SIMEND,ground_impact`
when the rocket lands, or `SIMERR,<reason>` for a non-finite output or plant
error.

## Running with visualization

`visualize.py` accepts the same scenario options as `sil.py run`, so it can
run the simulation itself. It shows a 3D North/East/Up trajectory with the
true attitude from the logged quaternion, plus time plots for altitude,
downrange, speed, attitude and the four commanded/actual fin angles.

```bash
# Render an MP4 directly from a new run
nix develop -c python3 code/sim/visualize.py --stop-at 20 --save flight.mp4

# Open an interactive window instead (requires a display)
nix develop -c python3 code/sim/visualize.py --stop-at 20

# Save a CSV first, then visualize that exact run later
nix develop -c python3 code/sim/sil.py run --stop-at 20 --csv flight.csv
nix develop -c python3 code/sim/visualize.py --input flight.csv \
  --rocket viikate_v1 --save flight.mp4
```

In a headless shell, use `--save out.mp4`; without it `visualize.py` exits
with `no interactive display; use --save out.mp4`. `--save` accepts `.mp4` or
`.gif`. Useful playback/rendering options:

- `--fps 30` — animation frames per second (default 30)
- `--speed 1.0` — simulated seconds per wall-clock second (default 1)
- `--dpi 110` — pixel density when saving (default 110)

Interactive keys: `space` pauses, `left`/`right` step, `home`/`end` jump to
the start/end. When using `--input`, pass `--rocket` as well so the drawing
uses the correct geometry.

A near-zero-wind example, including saving the CSV for later inspection:

```bash
nix develop -c python3 code/sim/visualize.py --stop-at 20 \
  --plant-param wind_north_mps=0.7 --plant-param wind_east_mps=0.7 \
  --plant-param gust_variance=0 \
  --csv flight-near-zero-wind.csv --save flight-near-zero-wind.mp4
```

## Common command-line options

| Option | Tools | Meaning |
|---|---|---|
| `--plant {rocket,toy}` | `sil.py build/run`, `visualize.py` | Plant model. Default `rocket`. |
| `--stop-at SECONDS` | `sil.py run`, `visualize.py` | Simulated duration. Default 6 s for `sil.py run`, 20 s for `visualize.py`. |
| `--plant-param NAME=VALUE` | `sil.py run`, `visualize.py` | Numeric plant override; repeatable. Unknown names stop the run. |
| `--rocket NAME\|PATH` | `sil.py run`, `visualize.py` | Design from `simulation/rockets/`, or a `.rocket` path. Rocket plant only. |
| `--servo-signs S1,S2,S3,S4` | `sil.py run`, `visualize.py` | Four mounting signs, each `+1` or `-1`, e.g. `1,1,-1,-1`. Applied to the four fin commands before the plant. Default `1,1,1,1`. |
| `--csv FILE` | `sil.py run`, `visualize.py` | Save the logged rows. With `--input`, `visualize.py` rejects `--csv`. |
| `--no-build` | `sil.py run`, `visualize.py` | Reuse `build/sil-<plant>/zephyr/zephyr.exe` instead of rebuilding. |
| `--input FILE` | `visualize.py` | Visualize a CSV from `sil.py run --csv`; no simulation is run. |
| `--save FILE.mp4\|.gif` | `visualize.py` | Render to a file instead of opening a window. |

`sil.py build --pristine` forces a clean CMake configure for the selected
plant.

## Plant parameters

### Rocket plant run conditions

These are not part of a `.rocket` design file. Defaults are the values built
into the generated plant.

| Parameter | Unit | Default | Meaning |
|---|---|---:|---|
| `ignition_time_s` | s | 3.0 | Motor ignition time. |
| `thrust_scale` | — | 1.0 | Multiplier on the thrust curve. |
| `wind_north_mps` | m/s | 5.0 | North wind component. |
| `wind_east_mps` | m/s | 5.0 | East wind component. |
| `wind_down_mps` | m/s | 0.0 | Down wind component; sign is inverted before entering the model's up-wind state. |
| `gust_variance` | (m/s)² | 2.0 | Variance of the random gusts; `0` disables gusts. |
| `gust_seed_1`, `gust_seed_2` | — | 1.0, 2.0 | Whole-number RNG seeds for the two gust channels. |

### Rocket design overrides

`--rocket` loads all of these from a design file. Repeating the same key as
`--plant-param` overrides just that number after the file is loaded. The
defaults below are for `viikate_v1`; see
[rocket designs](../simulation/rockets/README.md) for the full format and
sign conventions.

| Parameter | Unit | v1 value | Meaning |
|---|---|---:|---|
| `burn_time_s` | s | 1.0 | Burn cutoff; thrust and fuel flow stop after this. |
| `mass_full_kg`, `mass_fuel_kg` | kg | 1.352, 0.063 | Lift-off mass and propellant mass. |
| `length_m` | m | 1.27 | Reference length for pitch/yaw damping. |
| `frontal_area_m2` | m² | 0.00442 | Reference area. |
| `cd0` | — | 0.635 | Zero-lift drag coefficient. |
| `cn_alpha` | 1/rad | 20 | Normal-force slope. |
| `cp_aft_of_cg_m` | m | 0.346 | Centre of pressure behind the CG. |
| `cmq`, `cnr`, `clp` | — | −40, −40, −0.3 | Pitch, yaw and roll damping slopes. |
| `roll_ref_length_m` | m | 0.14 | Roll damping reference length. |
| `canard_area_m2` | m² | 0.0016875 | Area of one canard. |
| `canard_cl_alpha` | 1/rad | 1.975 | Canard lift slope. |
| `canard_fwd_of_cg_m` | m | 0.4689 | Canard centre of lift ahead of the CG. |
| `canard_lift_radius_m` | m | 0.0596975 | Rocket axis to a canard's centre of lift. |
| `servo_tau_s` | s | 0.05 | First-order servo lag time constant. |
| `servo_rate_limit_dps` | °/s | 600 | Canard rate limit. |

Formatted values must match the plant's range checks: masses, lengths, areas,
times and rate limits are positive; drag and lift slopes are non-negative;
wind and damping coefficients may be any finite number.

### Toy roll plant parameters

The toy plant holds the rocket vertical on a roll test stand. Only roll is
free, and all four fins contribute to the roll moment in the same direction.

| Parameter | Unit | Default | Meaning |
|---|---|---:|---|
| `airspeed_mps` | m/s | 100.0 | Constant test-section airspeed. |
| `initial_roll_rad` | rad | 0.0 | Initial roll angle. |
| `kick_time_s` | s | 2.5 | Time of the roll-rate kick. |
| `kick_roll_rate_radps` | rad/s | 1.0 | Roll rate added at `kick_time_s`. |
| `static_pressure_pa` | Pa | 101325.0 | Simulated barometer pressure. |
| `air_temperature_c` | °C | 15.0 | Simulated air temperature. |

## Logged columns

The CSV written by `--csv` contains:

```
t_ms, roll_deg, roll_rate_dps,
cmd0_deg..cmd3_deg, fin0_deg..fin3_deg,
pitch_deg, yaw_deg, pitch_rate_dps, yaw_rate_dps,
pos_n_m, pos_e_m, pos_d_m, vel_n_mps, vel_e_mps, vel_d_mps,
q_w, q_x, q_y, q_z,
sf_x_mps2, sf_y_mps2, sf_z_mps2, pressure_pa
```

Positions are NED relative to the pad; the quaternion is scalar-first. The
visualizer converts NED down to Up for its 3D view.

## Known limitation: zero horizontal wind

With the current generated rocket plant, setting both `wind_north_mps=0` and
`wind_east_mps=0` makes the run non-finite around 12.6 s, even with zero fin
commands. A zero-input plant replay reproduces it, so it is a plant-model
limitation rather than a visualization or firmware problem. Setting
`gust_variance=0` alone is safe with the nominal wind. For effectively still
air, use a small residual horizontal wind, for example 0.7 m/s on both north
and east with zero gust variance, as in the example above.
