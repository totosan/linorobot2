"""ROS-free regressions for the pinned G4 noise/slip profile."""

import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
PROFILE = json.loads((ROOT / "scripts/g4_robustness_profile.json").read_text())
SOURCE = ROOT


class G4RobustnessProfileTests(unittest.TestCase):
    def test_profile_is_finite_pinned_and_simulation_only(self):
        self.assertEqual(PROFILE["schema_version"], 1)
        self.assertEqual(PROFILE["profile_id"], "g4-noise-slip-v1")
        self.assertEqual(PROFILE["seed"], 20261008)
        self.assertEqual(PROFILE["lidar_range_stddev_m"], 0.01)
        self.assertEqual(PROFILE["imu_angular_velocity_stddev_rad_s"], 0.01)
        self.assertEqual(PROFILE["imu_linear_acceleration_stddev_m_s2"], 0.1)
        self.assertEqual(PROFILE["wheel_mu1"], PROFILE["wheel_mu2"])
        self.assertEqual(PROFILE["wheel_mu1"], 0.8)
        self.assertIn("not claims about a physical IMU", PROFILE["rationale"]["imu"])
        self.assertIn("Simulation-only", PROFILE["rationale"]["scope"])

    def test_g4_seed_plugin_is_first_and_matches_pinned_profile(self):
        world_path = SOURCE / "linorobot2_gazebo/worlds/fortress_g4_noise_slip.sdf"
        world = ET.parse(world_path).getroot().find("world")
        plugins = world.findall("plugin")
        self.assertGreaterEqual(len(plugins), 2)
        self.assertEqual(plugins[0].attrib["filename"], "libg4_random_seed_system.so")
        self.assertEqual(plugins[0].attrib["name"], "linorobot2_gazebo::G4RandomSeed")
        self.assertEqual(int(plugins[0].findtext("seed")), PROFILE["seed"])
        self.assertLess(
            next(i for i, p in enumerate(plugins) if "seed_system" in p.attrib["filename"]),
            next(i for i, p in enumerate(plugins) if "sensors-system" in p.attrib["filename"]))

        nominal = ET.parse(
            SOURCE / "linorobot2_gazebo/worlds/fortress_g4_test.sdf").getroot().find("world")
        def normalized(elements):
            for element in elements:
                element.tail = None
            return [ET.tostring(element) for element in elements]

        self.assertEqual(
            normalized([item for item in world if item.tag != "plugin"]),
            normalized([item for item in nominal if item.tag != "plugin"]))
        self.assertEqual(
            normalized(plugins[1:]),
            normalized(nominal.findall("plugin")))
        self.assertIsNone(nominal.find("plugin[@filename='libg4_random_seed_system.so']"))

    def test_sensor_xacros_expose_gaussian_parameters_with_zero_defaults(self):
        robot = (SOURCE / "linorobot2_description/urdf/robots/2wd_fortress.urdf.xacro").read_text()
        laser = (SOURCE / "linorobot2_description/urdf/sensors/fortress_laser.urdf.xacro").read_text()
        imu = (SOURCE / "linorobot2_description/urdf/sensors/fortress_imu.urdf.xacro").read_text()
        for name in ("lidar_noise_stddev", "imu_gyro_noise_stddev", "imu_accel_noise_stddev"):
            self.assertIn(f'name="{name}" default="0.0"', robot)
        self.assertIn("noise_stddev:=0.0", laser)
        self.assertIn("<type>gaussian</type>", laser)
        self.assertIn("${noise_stddev}", laser)
        self.assertEqual(imu.count('type="gaussian"'), 6)
        self.assertEqual(imu.count("${gyro_noise_stddev}"), 3)
        self.assertEqual(imu.count("${accel_noise_stddev}"), 3)

    def test_reduced_friction_is_scoped_to_fortress_wheels(self):
        robot = (SOURCE / "linorobot2_description/urdf/robots/2wd_fortress.urdf.xacro").read_text()
        self.assertIn('name="wheel_mu" default="1.0"', robot)
        self.assertIn('<gazebo reference="left_wheel_link">', robot)
        self.assertIn('<gazebo reference="right_wheel_link">', robot)
        self.assertEqual(robot.count("$(arg wheel_mu)"), 4)
        self.assertNotIn("wheel_mu", (SOURCE / "linorobot2_description/urdf/robots/2wd.urdf.xacro").read_text())

    def test_launch_and_gate_require_explicit_profile_opt_in_and_archive_hash(self):
        launch = (SOURCE / "linorobot2_gazebo/launch/fortress.launch.py").read_text()
        gate = (ROOT / "scripts/test_phase4_gate.sh").read_text()
        for arg in ("lidar_noise_stddev", "imu_gyro_noise_stddev", "imu_accel_noise_stddev", "wheel_mu"):
            self.assertIn(f"{arg}:=", launch)
            self.assertIn(f'"{arg}:=${{PROFILE_VALUES[', gate)
        self.assertIn("--noise-slip-profile", gate)
        self.assertIn("fortress_g4_noise_slip.sdf", gate)
        self.assertIn("g4_robustness_profile.json", gate)
        self.assertIn("g4_robustness_profile.sha256", gate)

    def test_seed_system_sets_the_shared_rng_used_by_gazebo_noise(self):
        source = (SOURCE / "linorobot2_gazebo/src/g4_random_seed_system.cpp").read_text()
        cmake = (SOURCE / "linorobot2_gazebo/CMakeLists.txt").read_text()
        hook = (SOURCE / "linorobot2_gazebo/env-hooks/linorobot2_gazebo.sh.in").read_text()
        self.assertIn("ignition::math::Rand::Seed(seed)", source)
        self.assertIn('sdf->Get<unsigned int>("seed")', source)
        self.assertIn("add_library(g4_random_seed_system SHARED", cmake)
        self.assertIn("ignition-gazebo6::ignition-gazebo6", cmake)
        self.assertIn("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", hook)


if __name__ == "__main__":
    unittest.main()
