"""check_provenance: detects a plant model changed without regenerating."""

import hashlib
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import check_provenance  # noqa: E402


def sha256(data):
    return hashlib.sha256(data).hexdigest()


class ProvenanceTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        (self.root / "model").mkdir()
        self.files = {
            "model/a.slx": b"model bytes",
            "model/params.m": b"x = 1;\n",
        }
        for name, data in self.files.items():
            (self.root / name).write_bytes(data)
        self.provenance = self.root / "PROVENANCE"
        self.write_provenance(self.files)

    def tearDown(self):
        self._tmp.cleanup()

    def write_provenance(self, files):
        lines = [f"sha256 {sha256(data)} {name}" for name, data in files.items()]
        lines += [
            "matlab_release R2026b",
            "target ert.tlc",
            "generated 2026-11-11T00:00:00Z",
        ]
        self.provenance.write_text("\n".join(lines) + "\n")

    def test_matching_hashes_pass(self):
        self.assertEqual(check_provenance.check(self.provenance, self.root), [])

    def test_changed_file_is_reported(self):
        (self.root / "model/params.m").write_bytes(b"x = 2;\n")
        problems = check_provenance.check(self.provenance, self.root)
        self.assertEqual(len(problems), 1)
        self.assertIn("model/params.m", problems[0])
        self.assertIn("changed", problems[0])

    def test_missing_listed_file_is_reported(self):
        (self.root / "model/a.slx").unlink()
        problems = check_provenance.check(self.provenance, self.root)
        self.assertEqual(len(problems), 1)
        self.assertIn("model/a.slx", problems[0])
        self.assertIn("missing", problems[0])

    def test_missing_provenance_is_reported(self):
        self.provenance.unlink()
        problems = check_provenance.check(self.provenance, self.root)
        self.assertEqual(len(problems), 1)
        self.assertIn("PROVENANCE", problems[0])
        self.assertIn("missing", problems[0])

    def test_provenance_without_hashes_is_reported(self):
        self.write_provenance({})
        problems = check_provenance.check(self.provenance, self.root)
        self.assertEqual(len(problems), 1)
        self.assertIn("no sha256", problems[0])


class CommittedProvenanceTest(unittest.TestCase):
    """The PROVENANCE generate_plant.m wrote for simulation/plant_codegen."""

    def test_every_generated_file_is_listed(self):
        repo = check_provenance.REPO_DIR
        codegen = repo / "simulation" / "plant_codegen"
        listed = {
            line.split()[2]
            for line in (codegen / "PROVENANCE").read_text().splitlines()
            if line.startswith("sha256 ")
        }
        generated = [
            p for p in codegen.rglob("*") if p.is_file() and p.name != "PROVENANCE"
        ]
        self.assertGreater(len(generated), 10)
        for path in generated:
            name = path.relative_to(repo).as_posix()
            self.assertIn(name, listed)

    def test_committed_plant_is_current(self):
        self.assertEqual(
            check_provenance.check(
                check_provenance.DEFAULT_PROVENANCE, check_provenance.REPO_DIR
            ),
            [],
        )


if __name__ == "__main__":
    unittest.main()
