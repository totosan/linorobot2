import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "sim_release_under_test", ROOT / "scripts" / "sim_release.py"
)
sim_release = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sim_release)


def make_release(root, name, status="passed", dirty=False):
    release = root / "releases" / name
    (release / "install").mkdir(parents=True)
    (release / "install" / "setup.bash").write_text("# test\n")
    (release / "evidence").mkdir()
    manifest = {
        "release_path": str(release.resolve()),
        "git": {"dirty": dirty},
        "validation": {"status": status},
    }
    (release / "evidence" / "release-manifest.json").write_text(
        json.dumps(manifest)
    )
    return release


class ReleaseValidationTests(unittest.TestCase):
    def test_requires_overlay_clean_source_and_passed_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(
                sim_release.validate_release(make_release(root, "good")),
                (root / "releases" / "good").resolve(),
            )
            for name, status, dirty in (
                ("pending", "pending", False),
                ("failed", "failed", False),
                ("dirty", "passed", True),
            ):
                with self.subTest(name=name), self.assertRaises(ValueError):
                    sim_release.validate_release(
                        make_release(root, name, status, dirty)
                    )

    def test_rejects_manifest_path_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            release = make_release(Path(directory), "bad-path")
            path = release / "evidence" / "release-manifest.json"
            data = json.loads(path.read_text())
            data["release_path"] = "/different/release"
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "path does not match"):
                sim_release.validate_release(release)


class ReleaseActivationTests(unittest.TestCase):
    def test_activation_and_rollback_are_atomic_and_preserve_previous(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = make_release(root, "first")
            second = make_release(root, "second")
            sim_release.activate_release(root, first, "")
            self.assertEqual((root / "active").resolve(), first.resolve())
            sim_release.activate_release(root, second, "")
            self.assertEqual((root / "active").resolve(), second.resolve())
            self.assertEqual((root / "previous").resolve(), first.resolve())
            sim_release.rollback_release(root, "")
            self.assertEqual((root / "active").resolve(), first.resolve())
            self.assertEqual((root / "previous").resolve(), second.resolve())

    def test_running_simulation_blocks_activation_without_signalling(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = make_release(root, "candidate")
            rows = "123 ros2 launch linorobot2_gazebo fortress.launch.py\n"
            with self.assertRaisesRegex(RuntimeError, "123 ros2 launch"):
                sim_release.activate_release(root, release, rows)
            self.assertFalse((root / "active").exists())

    def test_invalid_previous_release_blocks_rollback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "missing"
            (root / "previous").symlink_to(target)
            with self.assertRaises(ValueError):
                sim_release.rollback_release(root, "")


class ManifestTests(unittest.TestCase):
    @mock.patch.object(sim_release, "installed_version", return_value="test")
    @mock.patch.object(sim_release, "dpkg_versions", return_value=["pkg=1"])
    @mock.patch.object(sim_release, "git_value")
    def test_manifest_records_source_identity_and_validation(
        self, git_value, _dpkg_versions, _installed_version
    ):
        git_value.side_effect = [
            "",
            "0123456789abcdef",
            "https://example.invalid/linorobot2.git",
        ]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / "release"
            source = root / "source"
            source.mkdir()
            path = sim_release.write_manifest(
                release, source, "passed", ["python3 -m unittest"]
            )
            data = json.loads(path.read_text())
            self.assertEqual(data["git"]["commit"], "0123456789abcdef")
            self.assertFalse(data["git"]["dirty"])
            self.assertEqual(data["validation"]["status"], "passed")
            self.assertEqual(
                data["validation"]["commands"], ["python3 -m unittest"]
            )
            self.assertEqual(data["versions"]["debian_packages"], ["pkg=1"])


if __name__ == "__main__":
    unittest.main()
