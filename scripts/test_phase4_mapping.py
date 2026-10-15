"""ROS-free contract tests for G4 mapping helpers."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET
import phase4_geometry as geometry


SCRIPT_DIR = Path(__file__).parent
spec = importlib.util.spec_from_file_location(
    "phase4_mapping_under_test", SCRIPT_DIR / "verify_phase4_mapping.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def grid(width=50, height=40, resolution=0.1, origin=(-1.0, -1.0), values=None,
         frame="map", yaw=0.0):
    qz, qw = math.sin(yaw/2), math.cos(yaw/2)
    return NS(header=NS(frame_id=frame), info=NS(width=width, height=height,
              resolution=resolution, origin=NS(position=NS(x=origin[0], y=origin[1]),
                  orientation=NS(x=0.0, y=0.0, z=qz, w=qw))),
              data=values if values is not None else [-1] * (width*height))


class MappingHelperTests(unittest.TestCase):
    def test_map_summary_counts_only_known_and_occupied_cells(self):
        values = [-1] * (50*40)
        values[10*50+25] = 100
        values[10*50+26] = 65
        values[10*50+27] = 40
        metadata, data = probe.map_summary(grid(values=values))
        self.assertEqual(metadata["frame_id"], "map")
        self.assertEqual(metadata["known_cells"], 3)
        self.assertEqual(metadata["occupied_cells"], 2)
        self.assertEqual(data, values)

    def test_box_alignment_search_uses_grid_origin_resolution_and_row_major_data(self):
        values = [-1] * (50*40)
        values[10*50+25] = 93  # world x=1.5, y=0.0
        metadata, data = probe.map_summary(grid(values=values))
        self.assertEqual(probe.find_box_cell(metadata, data),
                         {"x_cell": 25, "y_cell": 10, "occupancy": 93})

    def test_box_search_does_not_claim_free_or_unknown_cells(self):
        values = [0] * (50*40)
        metadata, data = probe.map_summary(grid(values=values))
        self.assertIsNone(probe.find_box_cell(metadata, data))

    def test_box_search_accounts_for_rotated_map_origin(self):
        # At yaw=pi/2, world (1.5,0) maps to grid cell (30,5).
        values = [-1] * (50*40)
        values[5*50+30] = 90
        metadata, data = probe.map_summary(grid(origin=(2.0, -3.0), values=values,
                                                yaw=math.pi/2))
        self.assertEqual(probe.find_box_cell(metadata, data),
                 {"x_cell": 30, "y_cell": 5, "occupancy": 90})

    def test_map_growth_uses_new_coverage_and_obstacles_not_absolute_grid_size(self):
        initial = {"known_cells": 159, "occupied_cells": 0}
        mapped = {"known_cells": 655, "occupied_cells": 72}
        self.assertEqual(probe.validate_map_growth(initial, mapped), 496)
        with self.assertRaisesRegex(RuntimeError, "new coverage"):
            probe.validate_map_growth(initial, {"known_cells": 358, "occupied_cells": 72})
        with self.assertRaisesRegex(RuntimeError, "occupied cells"):
            probe.validate_map_growth(initial, {"known_cells": 655, "occupied_cells": 19})

    def test_map_summary_rejects_bad_geometry_frame_and_occupancy(self):
        invalid = [grid(width=0), grid(resolution=0), grid(frame="odom"),
                   grid(values=[-1] * (50*40-1)), grid(values=[101] + [-1] * (50*40-1)),
                   grid(values=[-2] + [-1] * (50*40-1))]
        for message in invalid:
            with self.subTest(message=message), self.assertRaises(ValueError):
                probe.map_summary(message)

    def test_clearance_subtracts_robot_margin_from_box_distance(self):
        pose = {"position": (0.0, 0.0, 0.1)}
        self.assertAlmostEqual(probe.world_clearance(pose), 1.2)
        self.assertLess(probe.world_clearance({"position": (1.4, 0.0, 0.1)}), 0.0)

    def test_sensor_mapping_clearance_includes_west_map_anchor(self):
        self.assertGreater(
            probe.world_clearance({"position": (-0.55, 0.0, 0.1)}), 0.15)
        self.assertAlmostEqual(
            probe.world_clearance({"position": (-1.5, 0.0, 0.1)}), 0.15)

    def test_mapping_route_clears_narrow_gate_with_footprint_margin(self):
        clearance = probe.world_clearance({
            "position": (geometry.NARROW_GATE_X,
                         geometry.NARROW_PASSAGE_CENTER_Y, 0.1)})
        self.assertGreater(clearance, 0.0)
        self.assertLess(clearance, 0.20)

    def test_g4_world_has_lidar_visible_map_anchor_and_preserves_other_world(self):
        root = Path(__file__).parents[1]/"linorobot2_gazebo/worlds"
        g4_world = ET.parse(root/"fortress_g4_test.sdf").getroot()
        self.assertEqual(g4_world.find("world").attrib["name"], "fortress_test")
        anchor = g4_world.find(".//model[@name='g4_west_map_anchor']")
        self.assertIsNotNone(anchor)
        self.assertEqual(anchor.findtext("static"), "true")
        pose = [float(value) for value in anchor.findtext("pose").split()]
        size = [float(value) for value in
                anchor.findtext("link/collision/geometry/box/size").split()]
        self.assertEqual(pose[:3], [-2.0, 0.0, 0.5])
        self.assertEqual(size, [0.1, 4.0, 1.0])
        self.assertIsNotNone(g4_world.find(".//model[@name='test_obstacle']"))

        phase_one_world = ET.parse(root/"fortress_test.sdf").getroot()
        self.assertIsNone(phase_one_world.find(".//model[@name='g4_west_map_anchor']"))
        passage_model_list = (
            g4_world.findall(".//model[@name='g4_passage_north_wall']")
            + g4_world.findall(".//model[@name='g4_passage_south_wall']"))
        passage_models = {item.attrib["name"]: item for item in passage_model_list}
        self.assertEqual(len(passage_model_list), 2)
        self.assertEqual(set(passage_models), {
            "g4_passage_north_wall", "g4_passage_south_wall"})
        self.assertEqual(g4_world.find("world").attrib["name"], "fortress_test")
        self.assertAlmostEqual(geometry.NARROW_PASSAGE_WIDTH, 1.0)
        for name, wall in passage_models.items():
            pose = [float(value) for value in wall.findtext("pose").split()]
            size = [float(value) for value in
                    wall.findtext("link/collision/geometry/box/size").split()]
            self.assertEqual(pose[0], geometry.NARROW_GATE_X)
            self.assertEqual(size[0], 2*geometry.NARROW_GATE_HALF_THICKNESS)
            low, high = pose[1]-size[1]/2, pose[1]+size[1]/2
            if name == "g4_passage_north_wall":
                self.assertAlmostEqual(low, geometry.NARROW_PASSAGE_HIGH_Y)
            else:
                self.assertAlmostEqual(high, geometry.NARROW_PASSAGE_LOW_Y)
        gate = Path(__file__).with_name("test_phase4_gate.sh").read_text()
        self.assertIn(
            'start_fortress "mapping" "$MAPPING_RVIZ_CFG" "$G4_WORLD" fortress_test',
            gate)
        self.assertIn(
            'start_fortress "reuse" "$NAV_RVIZ_CFG" "$G4_WORLD" fortress_test',
            gate)
        self.assertIn('--pid "$SIM_PID" --world "$expected_world"', gate)
        self.assertEqual(gate.count('start_fortress "reuse"'), 1)

    def test_return_lane_is_beyond_obstacle_clearance_margin(self):
        # The northbound return leg is at the far-side x after the 0.25m approach.
        lane_x = 0.25+probe.MAP_FAR_SIDE_METERS
        self.assertGreater(probe.world_clearance({"position": (lane_x, 0.0, 0.1)}), 0.15)
        south_lane = probe.world_clearance({"position": (2.0, -1.0, 0.1)})
        self.assertGreater(south_lane, 0.15)

    def test_g4_gate_records_and_validates_rosbag_before_passing(self):
        gate = Path(__file__).with_name("test_phase4_gate.sh").read_text()
        for topic in ("/clock", "/tf", "/tf_static", "/scan", "/map", "/odom",
                      "/cmd_vel", "/plan", "/global_costmap/costmap"):
            with self.subTest(topic=topic):
                self.assertIn(topic, gate)
        self.assertIn("ros2 bag record --storage sqlite3 --output", gate)
        self.assertIn('ros2 bag info "$LOG_DIR/rosbag"', gate)
        self.assertIn('"$LOG_DIR/rosbag/metadata.yaml"', gate)
        self.assertIn('^[[:space:]]*Messages:[[:space:]]*[1-9][0-9]*', gate)
        self.assertIn('stop_owned "$BAG_PID"', gate)
        self.assertLess(gate.index("stop_rosbag\n    echo \"PASS: accepted unreachable"),
                        gate.index("PASS: accepted unreachable"))
        self.assertLess(gate.rindex("stop_rosbag\necho \"PASS: measured Fortress"),
                        gate.rindex("PASS: measured Fortress"))


if __name__ == "__main__":
    unittest.main()
