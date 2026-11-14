# Rocket plant model

`viikate_rocket.slx` is the flight physics the firmware is tested against in
the software-in-the-loop simulation (`code/sim`). It is generated to plain C
in `simulation/plant_codegen/`, so running the SIL needs no MATLAB.

| File | Purpose |
|---|---|
| `viikate_rocket.slx` | Plant: 6DOF rocket, aerodynamics, thrust, wind, servo dynamics, pad hold. Fin commands in, sensor-level and truth signals out. |
| `plant_parameters.m` | All model parameters. `plant_parameters(design)` reads a rocket design file (default `simulation/rockets/viikate_v1.rocket`); `rp_*` ones are tunable at run time in the generated C. |
| `read_rocket_design.m` | Reads a `.rocket` design and its `.eng` motor with the C parser's rules. Format: [`simulation/rockets/README.md`](../../rockets/README.md). |
| `test_plant.m` | Pad hold tests and a back-to-back check against `../closedModel/closedmodel.slx`. |
| `backtoback_harness.slx` | Plant plus closedmodel's roll controller, used by the back-to-back test. |
| `generate_plant.m` | Generates the C, a reference scenario and `PROVENANCE`. |

`../closedModel/closedmodel.slx` is frozen. It is the reference the plant was
derived from and is never saved.

## Regenerating

Work in the MATLAB container (`simulation/matlab/Dockerfile`), where this
directory is `/work/simulation/matlab/plant`:

```matlab
cd /work/simulation/matlab/plant
runtests('test_plant.m')
generate_plant
```

Then commit the model and `simulation/plant_codegen/` together. Saving the
`.slx` changes its hash, so any save needs a regenerate:
`code/sim/tools/check_provenance.py` (run in CI) fails otherwise.

A new rocket design needs no regeneration: every value a design file sets is
an `rp_*` tunable the C simulation sets at run time. Regenerate when the
model, `plant_parameters.m`, `read_rocket_design.m` or one of the designs
listed in `PROVENANCE` (`viikate_v1`, `test/perturbed`) changes.

The 6DOF block is a local copy with its library links broken: its mask
builds the inertia interpolation table in mask initialization, which code
generation folds into a constant. Its two direct lookups (always the empty
and full pages, since the prelookup clips) were replaced by Constant blocks
reading `rp_inertia_empty_kgm2` and `rp_inertia_full_kgm2`.

## Known limitation

Fin lift is linear in deflection, with no stall and no mechanical stop.
Results at large fin deflections (the firmware can command 45 degrees) are
not to be trusted.
