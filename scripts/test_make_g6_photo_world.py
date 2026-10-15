"""Offline tests for the collision-safe G6 photo-world generator."""

import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "scripts/make_g6_photo_world.sh"


class G6PhotoWorldGeneratorTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.directory = Path(self.temp_dir.name)
        self.texture = self.directory / "person.jpg"
        self.texture.write_bytes(b"fixture-image-bytes")
        self.manifest = self.directory / "person.provenance.json"
        self.output = self.directory / "generated.sdf"
        self.write_manifest()

    def write_manifest(self, **overrides):
        manifest = {
            "source_page": "https://example.invalid/fixture",
            "creator": "Fixture creator",
            "license": "CC0",
            "license_url": "https://creativecommons.org/publicdomain/zero/1.0/",
            "sha256": hashlib.sha256(self.texture.read_bytes()).hexdigest(),
            "dimensions_px": [2300, 3138],
        }
        manifest.update(overrides)
        self.manifest.write_text(json.dumps(manifest), encoding="utf-8")

    def run_generator(self, *extra_args):
        return subprocess.run(
            [str(GENERATOR), "--texture", str(self.texture),
             "--provenance", str(self.manifest), *extra_args],
            capture_output=True, text=True, check=False)

    def test_generates_world_from_matching_cc0_manifest(self):
        result = self.run_generator("--output", str(self.output))
        self.assertEqual(result.returncode, 0, result.stderr)
        world = self.output.read_text(encoding="utf-8")
        self.assertIn(str(self.texture.resolve()), world)
        self.assertNotIn("@PHOTO_", world)
        parsed = ET.parse(self.output).getroot()
        size = [float(value) for value in
                parsed.findtext(".//model[@name='g6_person_photo']/link/visual/geometry/box/size").split()]
        self.assertAlmostEqual(size[1] / size[2], 2300 / 3138, places=5)

    def test_rejects_hash_mismatch_without_writing_output(self):
        self.write_manifest(sha256="0" * 64)
        result = self.run_generator("--output", str(self.output))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SHA-256", result.stderr)
        self.assertFalse(self.output.exists())

    def test_rejects_non_cc0_license_without_writing_output(self):
        self.write_manifest(license="unknown")
        result = self.run_generator("--output", str(self.output))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("only an explicitly recorded CC0 asset", result.stderr)
        self.assertFalse(self.output.exists())

    def test_refuses_to_overwrite_existing_world(self):
        self.output.write_text("preserve me", encoding="utf-8")
        result = self.run_generator("--output", str(self.output))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.output.read_text(encoding="utf-8"), "preserve me")


if __name__ == "__main__":
    unittest.main()
