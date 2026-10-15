#!/usr/bin/env python3
"""Prepare sensor-derived or explicitly GT-assisted fixture maps for G4 Nav2."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys


ROBOT_RADIUS = 0.22
MAPPING_ROBOT_MARGIN = 0.30
MIN_NAVIGATION_CLEARANCE = 0.15
BOX_LEFT_FACE_X = 1.5
BOX_HALF_SIZE_Y = 0.5
FREE_PIXEL = 254
MAP_SAVER_UNKNOWN_PIXEL = 205
UNKNOWN_PIXEL = 127


def read_pgm(path):
    content = Path(path).read_bytes()
    position = 0
    tokens = []
    while len(tokens) < 4:
        while position < len(content) and content[position] in b" \t\r\n\f\v":
            position += 1
        if position >= len(content):
            raise ValueError("truncated PGM header")
        if content[position] == ord("#"):
            while position < len(content) and content[position] not in b"\r\n":
                position += 1
            continue
        end = position
        while end < len(content) and content[end] not in b" \t\r\n\f\v#":
            end += 1
        tokens.append(content[position:end])
        position = end
    magic, width, height, maximum = tokens[0].decode("ascii"), *(
        int(value) for value in tokens[1:])
    if magic != "P5" or width <= 0 or height <= 0 or maximum != 255:
        raise ValueError("map must be a nonempty 8-bit binary PGM (P5)")
    if position >= len(content) or content[position] not in b" \t\r\n\f\v":
        raise ValueError("PGM max value is not followed by a header separator")
    if content[position:position+2] == b"\r\n":
        position += 2
    else:
        position += 1
    pixels = content[position:]
    if len(pixels) != width*height:
        raise ValueError("PGM pixel payload length does not match dimensions")
    return width, height, bytearray(pixels)


def parse_map_yaml(path):
    text = Path(path).read_text()

    def scalar(name):
        match = re.search(rf"^{name}:\s*([^\s#]+)\s*$", text, re.MULTILINE)
        if not match:
            raise ValueError(f"map YAML is missing {name}")
        return match.group(1)

    image = scalar("image")
    mode = scalar("mode")
    resolution = float(scalar("resolution"))
    negate = int(scalar("negate"))
    occupied = float(scalar("occupied_thresh"))
    free = float(scalar("free_thresh"))
    origin_match = re.search(
        r"^origin:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]\s*$",
        text, re.MULTILINE)
    if not origin_match:
        raise ValueError("map YAML origin must be a three-number list")
    origin = tuple(float(value) for value in origin_match.groups())
    if (mode != "trinary" or negate != 0 or not math.isfinite(resolution)
            or resolution <= 0 or not 0 < free < occupied < 1
            or not all(math.isfinite(value) for value in origin)
            or abs(origin[2]) > 1e-6):
        raise ValueError("map YAML must use finite trinary, non-negated, axis-aligned geometry")
    return text, image, resolution, origin, free, occupied


def point_segment_distance(x, y, start, end):
    ax, ay = start
    bx, by = end
    dx, dy = bx-ax, by-ay
    length_sq = dx*dx+dy*dy
    if length_sq == 0:
        return math.hypot(x-ax, y-ay)
    ratio = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/length_sq))
    return math.hypot(x-(ax+ratio*dx), y-(ay+ratio*dy))


def grid_pixel_index(width, height, resolution, origin, x, y):
    column = math.floor((x-origin[0])/resolution)
    row_from_bottom = math.floor((y-origin[1])/resolution)
    if not (0 <= column < width and 0 <= row_from_bottom < height):
        return None
    return (height-1-row_from_bottom)*width+column


def pixel_is_free(value, free_threshold):
    return 1.0-value/255.0 < free_threshold


def classify_pixel(value, free_threshold, occupied_threshold):
    occupancy = 1.0-value/255.0
    if occupancy > occupied_threshold:
        return 100
    if occupancy < free_threshold:
        return 0
    return -1


def prepare_sensor_seeded_map(yaml_path, output_prefix, robot_radius=ROBOT_RADIUS,
                              initial_pose=(0.0, 0.0)):
    """Pad a raw SLAM map; seed only the configured start footprint.

    Map-saver's unknown pixel (205) is normalized because the configured free
    threshold otherwise makes Nav2 treat it as traversable. Only unknown cells
    inside the configured initial robot footprint may be marked free, because
    the robot occludes its own footprint from the lidar. No mapping report,
    ground-truth pose, trajectory, or traversed corridor is accepted.
    """
    yaml_path = Path(yaml_path)
    output_prefix = Path(output_prefix)
    output_yaml = output_prefix.with_suffix(".yaml")
    output_pgm = output_prefix.with_suffix(".pgm")
    report_path = output_prefix.with_suffix(".prepare.json")
    if any(path.exists() for path in (output_yaml, output_pgm, report_path)):
        raise FileExistsError("prepared map output already exists")

    map_yaml, image_name, resolution, origin, free_thresh, occupied_thresh = parse_map_yaml(yaml_path)
    image_path = (yaml_path.parent/image_name).resolve()
    width, height, pixels = read_pgm(image_path)
    if not math.isfinite(robot_radius) or not 0.05 <= robot_radius <= 0.5:
        raise ValueError("robot radius must be finite and in [0.05,0.5] m")
    if (len(initial_pose) != 2
            or not all(math.isfinite(float(value)) for value in initial_pose)):
        raise ValueError("initial pose must contain finite x/y coordinates")

    unknown_count = sum(value == MAP_SAVER_UNKNOWN_PIXEL for value in pixels)
    normalized = bytearray(UNKNOWN_PIXEL if value == MAP_SAVER_UNKNOWN_PIXEL else value
                          for value in pixels)
    pad_cells = math.ceil(robot_radius/resolution)
    new_width, new_height = width+2*pad_cells, height+2*pad_cells
    new_origin = (origin[0]-pad_cells*resolution,
                  origin[1]-pad_cells*resolution, origin[2])
    padded = bytearray()
    for row in range(height):
        padded.extend([UNKNOWN_PIXEL]*pad_cells)
        padded.extend(normalized[row*width:(row+1)*width])
        padded.extend([UNKNOWN_PIXEL]*pad_cells)
    padded = (bytearray([UNKNOWN_PIXEL]*(new_width*pad_cells)) + padded
              + bytearray([UNKNOWN_PIXEL]*(new_width*pad_cells)))

    seeded_footprint_cells = 0
    for row_from_bottom in range(new_height):
        y = new_origin[1]+(row_from_bottom+0.5)*resolution
        pgm_row = new_height-1-row_from_bottom
        for column in range(new_width):
            x = new_origin[0]+(column+0.5)*resolution
            if math.hypot(x-float(initial_pose[0]), y-float(initial_pose[1])) > robot_radius:
                continue
            index = pgm_row*new_width+column
            if padded[index] < (1.0-occupied_thresh)*255:
                raise ValueError(
                    f"configured initial footprint overlaps occupied map cell at ({x:.3f},{y:.3f})")
            if padded[index] != FREE_PIXEL:
                padded[index] = FREE_PIXEL
                seeded_footprint_cells += 1

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    output_pgm.write_bytes(f"P5\n{new_width} {new_height}\n255\n".encode("ascii")+padded)
    prepared_yaml = re.sub(r"^image:\s*[^\s#]+\s*$",
                           f"image: {output_pgm.name}", map_yaml, count=1, flags=re.MULTILINE)
    prepared_yaml = re.sub(
        r"^origin:\s*\[.*?\]\s*$",
        f"origin: [{new_origin[0]:.6f}, {new_origin[1]:.6f}, {new_origin[2]:.6f}]",
        prepared_yaml, count=1, flags=re.MULTILINE)
    if prepared_yaml == map_yaml:
        output_pgm.unlink(missing_ok=True)
        raise ValueError("could not update map YAML image/origin fields")
    output_yaml.write_text(prepared_yaml)

    report = {
        "status": "PASS",
        "scope": "G4_SENSOR_DERIVED_MAP_WITH_START_FOOTPRINT_SEED",
        "navigation_source": "raw_slam_map_plus_configured_initial_footprint",
        "ground_truth_used_for_navigation_map": False,
        "configured_initial_pose_xy_m": [float(value) for value in initial_pose],
        "start_footprint_seed_source": "configured_spawn_and_AMCL_initial_pose_not_ground_truth_trajectory",
        "start_footprint_seeded_cells": seeded_footprint_cells,
        "free_cells_added": seeded_footprint_cells,
        "occupied_cells_removed": 0,
        "unknown_cells_normalized": unknown_count,
        "source_map_yaml": str(yaml_path),
        "source_map_pgm": str(image_path),
        "source_map_yaml_sha256": hashlib.sha256(yaml_path.read_bytes()).hexdigest(),
        "source_map_pgm_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        "prepared_map_yaml": str(output_yaml),
        "prepared_map_pgm": str(output_pgm),
        "raw_slam_map_preserved": True,
        "dimensions_before": [width, height],
        "dimensions_after": [new_width, new_height],
        "resolution_m": resolution,
        "origin_before": list(origin),
        "origin_after": list(new_origin),
        "unknown_padding_cells": new_width*new_height-width*height,
        "robot_radius_m": robot_radius,
        "unknown_pixel_value": UNKNOWN_PIXEL,
    }
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    return report


def prepare_map(yaml_path, mapping_report_path, output_prefix,
                robot_radius=ROBOT_RADIUS, goal_x=0.55):
    yaml_path = Path(yaml_path)
    output_prefix = Path(output_prefix)
    output_yaml = output_prefix.with_suffix(".yaml")
    output_pgm = output_prefix.with_suffix(".pgm")
    report_path = output_prefix.with_suffix(".prepare.json")
    if any(path.exists() for path in (output_yaml, output_pgm, report_path)):
        raise FileExistsError("prepared map output already exists")

    map_yaml, image_name, resolution, origin, free_thresh, occupied_thresh = parse_map_yaml(yaml_path)
    image_path = (yaml_path.parent/image_name).resolve()
    width, height, pixels = read_pgm(image_path)
    normalized_unknown_cells = sum(value == MAP_SAVER_UNKNOWN_PIXEL for value in pixels)
    pixels = bytearray(UNKNOWN_PIXEL if value == MAP_SAVER_UNKNOWN_PIXEL else value
                       for value in pixels)
    mapping = json.loads(Path(mapping_report_path).read_text())
    if mapping.get("status") != "PASS" or mapping.get("scope") != "G4_SIMULATION_ONLY":
        raise ValueError("start corridor requires a passing G4 mapping report")
    metadata = mapping.get("map")
    if not isinstance(metadata, dict) or metadata.get("frame_id", "").lstrip("/") != "map":
        raise ValueError("mapping report has no measured map-frame metadata")
    if (metadata.get("width") != width or metadata.get("height") != height
            or abs(float(metadata.get("resolution", 0))-resolution) > 1e-6
            or any(abs(float(metadata["origin"][index])-origin[index]) > resolution*0.1
                   for index in range(3))):
        raise ValueError("saved map geometry does not match the measured SLAM report")

    box = mapping.get("test_box_occupied_cell_near_left_face")
    if not isinstance(box, dict) or box.get("occupancy", 0) < 65:
        raise ValueError("mapping report has no occupied test-box cell")
    box_index = grid_pixel_index(width, height, resolution, origin,
                                 (box["x_cell"]+0.5)*resolution+origin[0],
                                 (box["y_cell"]+0.5)*resolution+origin[1])
    box_x = origin[0]+(box["x_cell"]+0.5)*resolution
    box_y = origin[1]+(box["y_cell"]+0.5)*resolution
    if (box_index is None or pixels[box_index] >= (1.0-occupied_thresh)*255
            or abs(box_x-BOX_LEFT_FACE_X) > 0.25
            or abs(box_y) > BOX_HALF_SIZE_Y+0.05):
        raise ValueError("saved map does not preserve an aligned occupied test-box cell")

    segments = {item.get("name"): item for item in mapping.get("segments", [])}
    names = ("cover_west_of_start", "approach_box")
    south_names = ("pass_south_of_box", "map_far_side")
    if any(name not in segments or not segments[name].get("pass")
           for name in names+south_names):
        raise ValueError("mapping report lacks passing start-corridor segments")
    first, second = (segments[name] for name in names)
    south_segments = [segments[name] for name in south_names]
    start = first["ground_truth_start"]["position"][:2]
    join_a = first["ground_truth_end"]["position"][:2]
    join_b = second["ground_truth_start"]["position"][:2]
    end = second["ground_truth_end"]["position"][:2]
    if (math.dist(start, (0.0, 0.0)) > 0.08 or math.dist(join_a, join_b) > 0.08
            or min(first["min_obstacle_clearance_m"],
                   second["min_obstacle_clearance_m"]) <= 0.0):
        raise ValueError("measured start-corridor path is discontinuous or not clearance-checked")
    path_segments = [(start, join_a), (join_b, end)]
    clearance_adjustment = MAPPING_ROBOT_MARGIN-robot_radius
    for name, segment in zip(south_names, south_segments):
        measured_clearance = float(segment.get("min_obstacle_clearance_m", -math.inf))
        if (not math.isfinite(measured_clearance)
                or measured_clearance+clearance_adjustment < MIN_NAVIGATION_CLEARANCE):
            raise ValueError(f"measured south-lane segment lacks obstacle clearance: {name}")
        samples = segment.get("ground_truth_samples")
        if not isinstance(samples, list) or len(samples) < 2:
            raise ValueError(f"measured south-lane segment lacks trajectory samples: {name}")
        points = [sample["pose"]["position"][:2] for sample in samples]
        if (math.dist(points[0], segment["ground_truth_start"]["position"][:2]) > 0.08
                or math.dist(points[-1], segment["ground_truth_end"]["position"][:2]) > 0.08):
            raise ValueError(f"measured south-lane trajectory endpoints disagree: {name}")
        for before, after in zip(samples, samples[1:]):
            clearance = float(after.get("obstacle_clearance_m", -math.inf))
            gap = math.dist(before["pose"]["position"][:2],
                            after["pose"]["position"][:2])
            if (not math.isfinite(clearance)
                    or clearance+clearance_adjustment < MIN_NAVIGATION_CLEARANCE
                    or gap > 0.35):
                raise ValueError(f"measured south-lane trajectory is unsafe or undersampled: {name}")
            path_segments.append((before["pose"]["position"][:2],
                                  after["pose"]["position"][:2]))

    if not math.isfinite(robot_radius) or not 0.05 <= robot_radius <= 0.5:
        raise ValueError("robot radius must be finite and in [0.05,0.5] m")
    if not math.isfinite(goal_x) or not 0.2 <= goal_x <= 2.0:
        raise ValueError("free goal x must be finite and in [0.2,2.0] m")
    carve_radius = robot_radius+resolution*math.sqrt(2.0)/2.0
    pad_cells = math.ceil(carve_radius/resolution)
    lowest_corridor_y = min(min(start[1], end[1]) for start, end in path_segments)
    minimum_safe_origin_y = lowest_corridor_y-carve_radius-resolution/2.0
    south_pad_cells = max(
        pad_cells, math.ceil((origin[1]-minimum_safe_origin_y)/resolution))
    new_width = width+pad_cells
    new_height = height+south_pad_cells
    new_origin = (origin[0]-pad_cells*resolution,
                  origin[1]-south_pad_cells*resolution, origin[2])
    padded = bytearray()
    for row in range(height):
        padded.extend([UNKNOWN_PIXEL]*pad_cells)
        padded.extend(pixels[row*width:(row+1)*width])
    padded.extend([UNKNOWN_PIXEL]*(new_width*south_pad_cells))

    carved_cells = 0
    for row_from_bottom in range(new_height):
        y = new_origin[1]+(row_from_bottom+0.5)*resolution
        pgm_row = new_height-1-row_from_bottom
        for column in range(new_width):
            x = new_origin[0]+(column+0.5)*resolution
            if min(point_segment_distance(x, y, a, b) for a, b in path_segments) > carve_radius:
                continue
            index = pgm_row*new_width+column
            if padded[index] < (1.0-occupied_thresh)*255:
                raise ValueError(f"verified start corridor intersects occupied map cell at ({x:.3f},{y:.3f})")
            if padded[index] != FREE_PIXEL:
                padded[index] = FREE_PIXEL
                carved_cells += 1

    def require_free(x, y, label):
        index = grid_pixel_index(new_width, new_height, resolution, new_origin, x, y)
        if index is None or not pixel_is_free(padded[index], free_thresh):
            raise ValueError(f"prepared map does not contain known free {label} at ({x:.3f},{y:.3f})")

    require_free(0.0, 0.0, "robot start")
    path_count = max(1, math.ceil(goal_x/(resolution/2)))
    for i in range(path_count+1):
        require_free(goal_x*i/path_count, 0.0, "start-to-free-goal path")
    footprint_cells = 0
    for row_from_bottom in range(new_height):
        y = new_origin[1]+(row_from_bottom+0.5)*resolution
        for column in range(new_width):
            x = new_origin[0]+(column+0.5)*resolution
            if math.hypot(x, y) <= robot_radius:
                require_free(x, y, "start footprint")
                footprint_cells += 1

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    output_pgm.write_bytes(f"P5\n{new_width} {new_height}\n255\n".encode("ascii")+padded)
    prepared_yaml = re.sub(r"^image:\s*[^\s#]+\s*$",
                           f"image: {output_pgm.name}", map_yaml, count=1, flags=re.MULTILINE)
    prepared_yaml = re.sub(
        r"^origin:\s*\[.*?\]\s*$",
        f"origin: [{new_origin[0]:.6f}, {new_origin[1]:.6f}, {new_origin[2]:.6f}]",
        prepared_yaml, count=1, flags=re.MULTILINE)
    if prepared_yaml == map_yaml:
        output_pgm.unlink(missing_ok=True)
        raise ValueError("could not update map YAML image/origin fields")
    output_yaml.write_text(prepared_yaml)

    report = {"status": "PASS", "scope": "G4_VERIFIED_MOTION_CORRIDORS_MAP",
              "source_map_yaml": str(yaml_path), "source_mapping_report": str(mapping_report_path),
              "prepared_map_yaml": str(output_yaml), "prepared_map_pgm": str(output_pgm),
              "raw_slam_map_preserved": True,
              "map_origin_before": list(origin), "map_origin_after": list(new_origin),
              "resolution_m": resolution, "padding_columns_west": pad_cells,
              "padding_rows_south": south_pad_cells,
              "robot_radius_m": robot_radius, "carved_free_cells": carved_cells,
              "measured_south_corridor_segments": list(south_names),
              "source_unknown_cells_normalized": normalized_unknown_cells,
              "nav2_unknown_pixel_value": UNKNOWN_PIXEL,
              "start_footprint_cells_verified": footprint_cells,
              "free_goal_x_m": goal_x, "occupied_test_box_cell_preserved": box,
              "source_segments": list(names)}
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaml", type=Path, required=True)
    parser.add_argument("--mode", choices=("sensor-seeded", "fixture"),
                        default="sensor-seeded")
    parser.add_argument("--mapping-report", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--robot-radius", type=float, default=ROBOT_RADIUS)
    parser.add_argument("--goal-x", type=float, default=0.55)
    args = parser.parse_args(argv)
    try:
        if args.mode == "sensor-seeded":
            if args.mapping_report is not None:
                raise ValueError("sensor-seeded mode must not receive a mapping report")
            report = prepare_sensor_seeded_map(args.yaml, args.output_prefix,
                                               args.robot_radius)
        else:
            if args.mapping_report is None:
                raise ValueError("fixture mode requires --mapping-report")
            report = prepare_map(args.yaml, args.mapping_report, args.output_prefix,
                                 args.robot_radius, args.goal_x)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"FAIL [G4_MAP_PREPARATION]: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(f"PASS [G4_MAP_PREPARATION]: {report['prepared_map_yaml']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())