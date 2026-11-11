#!/usr/bin/env python3
"""Check that the generated rocket plant matches the model it came from.

simulation/plant_codegen/PROVENANCE lists the SHA-256 of every input of
simulation/matlab/plant/generate_plant.m. If any of them changed since the
code was generated (for example the .slx was saved), the committed C no
longer matches the model. Needs no MATLAB.

    python3 code/sim/tools/check_provenance.py

Prints one line per problem and exits 1 if there are any.
"""

import hashlib
import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[3]
DEFAULT_PROVENANCE = REPO_DIR / "simulation" / "plant_codegen" / "PROVENANCE"


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            digest.update(chunk)
    return digest.hexdigest()


def check(provenance, repo_root):
    """Return a list of problems; empty if the generated code is current."""
    provenance = Path(provenance)
    repo_root = Path(repo_root)
    if not provenance.is_file():
        return [f"{provenance}: PROVENANCE file is missing"]

    problems = []
    hashed = 0
    for line in provenance.read_text().splitlines():
        fields = line.split()
        if not fields or fields[0] != "sha256":
            continue
        if len(fields) != 3:
            problems.append(f"{provenance}: malformed line {line!r}")
            continue
        hashed += 1
        _, expected, name = fields
        path = repo_root / name
        if not path.is_file():
            problems.append(f"{name}: listed in PROVENANCE but missing")
        elif _sha256(path) != expected:
            problems.append(
                f"{name}: changed since the plant was generated; "
                "run generate_plant.m in MATLAB"
            )
    if hashed == 0:
        problems.append(f"{provenance}: no sha256 lines")
    return problems


def main():
    problems = check(DEFAULT_PROVENANCE, REPO_DIR)
    for problem in problems:
        print(problem)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
