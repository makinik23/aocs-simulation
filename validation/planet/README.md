# Planet Dove Orbit Validation

This folder contains tooling for local, external end-to-end validation of the
AOCS orbit propagator against Planet Labs public orbital ephemerides.

Planet publishes per-satellite CCSDS OEM predictions at:

```text
https://ephemerides.planet-labs.com/HWID_oem.txt
```

Those data are licensed separately by Planet Labs under CC BY-NC 4.0, so raw
OEM files and generated MAT fixtures are intentionally ignored by this repo.

Example local workflow:

```bash
mkdir -p validation/planet/data
curl -L https://ephemerides.planet-labs.com/2409_oem.txt \
  -o validation/planet/data/2409_oem.txt
```

```matlab
addpath("validation/planet")
generatePlanetDoveReference( ...
    "validation/planet/data/2409_oem.txt", ...
    "validation/planet/data/planet_dove_oem_reference.mat", ...
    15 * 60)
```

Then run:

```matlab
results = runtests("tests/orbit_and_environment/PlanetDoveOrbitPropagationValidationTest.m");
assertSuccess(results)
```
