# Rocket designs

Each rocket iteration is a `<name>.rocket` design file here plus a RASP
`.eng` motor file. The same files drive the Simulink model
(`simulation/matlab/plant/plant_parameters.m`) and the C simulation the
firmware flies in (`code/sim`). The C simulation reads them at run time, so a
new design needs no MATLAB and no rebuild.

| File | Purpose |
|---|---|
| `viikate_v1.rocket`, `motors/viikate_v1.eng` | Baseline: the values the frozen `closedModel` flies. Built into the generated plant. |
| `test/perturbed.rocket`, `test/motors/perturbed.eng` | Test design with every value changed; checks that each value reaches the generated C. Not flown by the flight tests. |

## Adding a design

1. Copy `viikate_v1.rocket` to `simulation/rockets/<name>.rocket`. `name` is
   lower case letters, digits and `_`.
2. Edit the values (key table below). Add a motor file under `motors/`, or
   keep v1's.
3. Run `python3 code/sim/sil.py test`. Every `*.rocket` file in this
   directory is flown by the flight tests. No MATLAB is needed.

To fly one design:

    python3 code/sim/sil.py run --rocket <name> --stop-at 20

Or directly: `zephyr.exe --rocket=<path> --plant-params=cd0=0.7`, or
`plant_replay --rocket <path> ...`. `--plant-params` overrides any
single-number key after the file is loaded, for example for sweeps.

Don't edit `viikate_v1.rocket` for a new iteration. It is the baseline tied
to closedModel, built into the generated plant and an input of
`simulation/plant_codegen/PROVENANCE`, so changing it means regenerating the
plant. A new iteration is a new file.

## Format

One `key = value` per line, `#` starts a comment, lists are comma-separated.
Units are SI, named in the key. Every key is required except where noted.
An unknown key, a missing key, a key given twice or a value out of range is
an error naming the file and line, and the run stops before it starts.

| Key | Unit | Values | Rule |
|---|---|---|---|
| `name` | | 1 | `[a-z0-9_]+` |
| `motor` | | 1 | `.eng` path relative to this file, no `..` |
| `burn_time_s` | s | 1 | optional, > 0; default the motor's last time. Thrust and fuel flow stop after it. |
| `mass_full_kg` | kg | 1 | > 0, lift-off mass |
| `mass_fuel_kg` | kg | 1 | > 0 and < `mass_full_kg` |
| `inertia_full_kgm2` | kg m² | 3 | Ixx, Iyy, Izz about the CG, full |
| `inertia_empty_kgm2` | kg m² | 3 | the same, empty |
| `inertia_full_products_kgm2` | kg m² | 3 | optional, Ixy, Ixz, Iyz; default 0 |
| `inertia_empty_products_kgm2` | kg m² | 3 | optional, the same, empty |
| `length_m` | m | 1 | > 0, used by pitch and yaw damping |
| `frontal_area_m2` | m² | 1 | > 0, reference area |
| `cd0` | | 1 | ≥ 0, zero-lift drag coefficient |
| `cn_alpha` | 1/rad | 1 | ≥ 0, normal force slope |
| `cp_aft_of_cg_m` | m | 1 | centre of pressure behind the CG (negative: ahead of it) |
| `cmq`, `cnr`, `clp` | | 1 each | pitch, yaw and roll damping slopes |
| `roll_ref_length_m` | m | 1 | > 0, roll damping reference length |
| `canard_area_m2` | m² | 1 | > 0, area of one canard |
| `canard_cl_alpha` | 1/rad | 1 | ≥ 0, canard lift slope |
| `canard_fwd_of_cg_m` | m | 1 | canard centre of lift ahead of the CG |
| `canard_lift_radius_m` | m | 1 | > 0, rocket axis to each canard's centre of lift |
| `servo_tau_s` | s | 1 | > 0, servo lag time constant |
| `servo_rate_limit_dps` | deg/s | 1 | > 0 |

The full and empty inertia tensors must be positive-definite. Inertia is
interpolated linearly in mass between them during the burn.

### Getting the values

- Positions are relative to the CG, along the body x axis towards the nose.
  From OpenRocket, which reports positions from the nose tip:
  `cp_aft_of_cg_m` = CP from nose − CG from nose, and
  `canard_fwd_of_cg_m` = CG from nose − canard centre of lift from nose.
- Products of inertia are the off-diagonal entries of the inertia tensor
  `J = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz]` the model's 6DOF block takes.
  That is the Aerospace Blockset convention, where the entries are −∫xy dm.
  CAD tools differ: some report +∫xy dm and expect the tensor to negate it.
  Check which yours gives before copying the numbers, in the body frame (x
  towards the nose) about the CG.

### Motor files

Standard RASP `.eng`, as ThrustCurve.org and OpenRocket export:

- `;` starts a comment.
- The first other line is the header
  `name diameter_mm length_mm delays prop_mass_kg total_mass_kg manufacturer`.
  Only its field count is checked; propellant mass comes from
  `mass_fuel_kg`, not from here.
- Then one `time_s thrust_n` per line: at least 2 and at most 127 points,
  times increasing from ≥ 0, thrust ≥ 0.
- One motor per file. A file with a second motor block is an error.
- If the first time is above 0, the curve starts from (0, 0), as the `.eng`
  convention means. `viikate_v1.eng` has an explicit t = 0 point because
  closedModel extrapolated its curve back to t = 0.

## Fixed in the model

Not part of a design; changing them means editing `viikate_rocket.slx`:
four canards in a + layout, constant air density 1.225 kg/m³, gravity
9.819 m/s², fuel burned at a constant rate, linear canard lift with no stall,
CG fixed during the burn.
