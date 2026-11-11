# Rocket plant model

`viikate_rocket.slx` is the flight physics the firmware is tested against in
the software-in-the-loop simulation (`code/sim`). It is generated to plain C
in `simulation/plant_codegen/`, so running the SIL needs no MATLAB.

| File | Purpose |
|---|---|
| `viikate_rocket.slx` | Plant: 6DOF rocket, aerodynamics, thrust, wind, servo dynamics, pad hold. Fin commands in, sensor-level and truth signals out. |
| `plant_parameters.m` | All model parameters. `rp_*` ones are tunable at run time in the generated C. |
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

## Known limitation

Fin lift is linear in deflection, with no stall and no mechanical stop.
Results at large fin deflections (the firmware can command 45 degrees) are
not to be trusted.
