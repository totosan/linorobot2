"""Local-only tests: optional colcon CLI compatibility, never run the build."""

from pathlib import Path
import shutil
import subprocess
import unittest


SCRIPT = Path(__file__).with_name("setup_tf2_fix.sh")


@unittest.skipUnless(shutil.which("bash"), "requires bash")
class Tf2SetupTests(unittest.TestCase):
    def override_args(self, help_text):
        text = SCRIPT.read_text()
        fragment = text.split("OVERRIDE_ARGS=()", 1)[1].split(
            "colcon --log-base", 1)[0]
        # Only the option-selection fragment runs, with a fake colcon function.
        program = (
            'set -euo pipefail\n'
            'HELP_TEXT="$1"\ncolcon() { printf "%s\\n" "$HELP_TEXT"; }\n'
            'OVERRIDE_ARGS=()\n' + fragment +
            '\nfor arg in ${OVERRIDE_ARGS[@]+"${OVERRIDE_ARGS[@]}"}; '
            'do printf "%s\\n" "$arg"; done\n')
        result = subprocess.run(
            ["bash", "-c", program, "tf2-options-test", help_text],
            capture_output=True, text=True, check=True)
        return result.stdout.splitlines()

    def test_core_colcon_does_not_receive_extension_only_flag(self):
        self.assertEqual(self.override_args(
            "usage: colcon build [--packages-select PKG_NAME]"), [])

    def test_colcon_with_override_extension_gets_explicit_package_allowlist(self):
        self.assertEqual(self.override_args(
            "usage: colcon build\n  --allow-overriding [PKG_NAME ...]"),
            ["--allow-overriding", "tf2", "tf2_ros"])

    def test_complete_setup_script_has_valid_bash_syntax(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True)

    def library_paths(self, inherited):
        fragment = "OLD_LIBS=" + SCRIPT.read_text().split("OLD_LIBS=", 1)[1].split(
            'env LD_LIBRARY_PATH="$OLD_LIBS"', 1)[0]
        program = ('set -euo pipefail\nROOT="/tmp/tf2-overlay"\n'
                   'LD_LIBRARY_PATH="$1"\n' + fragment +
                   '\nprintf "%s\\n" "$OLD_LIBS" "$NEW_LIBS"\n')
        return subprocess.check_output(
            ["bash", "-c", program, "tf2-path-test", inherited],
            text=True).splitlines()

    def test_rmw_architecture_paths_survive_tf2_library_selection(self):
        inherited = "/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib"
        old, new = self.library_paths(inherited)
        self.assertEqual(old, "/opt/ros/humble/lib:" + inherited)
        self.assertEqual(new, "/tmp/tf2-overlay/install/lib:" + old)

    def test_empty_inherited_library_path_does_not_add_working_directory(self):
        self.assertEqual(self.library_paths(""), [
            "/opt/ros/humble/lib",
            "/tmp/tf2-overlay/install/lib:/opt/ros/humble/lib"])


if __name__ == "__main__":
    unittest.main()