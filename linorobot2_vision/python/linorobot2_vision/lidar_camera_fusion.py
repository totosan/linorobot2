"""Project 2D LiDAR returns into camera detections for planar association.

The output is a measured LiDAR surface point in the robot plane, not a 3D
object centroid. No Nav2 or command interface is provided here.
"""

import math
import statistics

from .contract import VisionContractError, validate_response


_NSEC_PER_SEC = 1_000_000_000


def transform_point(point, transform):
    """Apply a rigid transform represented by translation and XYZW quaternion.

    ``transform`` is ``(tx, ty, tz, qx, qy, qz, qw)`` and maps coordinates
    from the source frame into the target frame.
    """
    if len(point) != 3 or len(transform) != 7:
        raise ValueError("point and transform must have lengths 3 and 7")
    px, py, pz = (float(value) for value in point)
    tx, ty, tz, qx, qy, qz, qw = (float(value) for value in transform)
    values = (px, py, pz, tx, ty, tz, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("point and transform values must be finite")
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm <= 1e-12:
        raise ValueError("transform quaternion must be nonzero")
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # v' = v + qw * (2 q_xyz x v) + q_xyz x (2 q_xyz x v)
    cx = qy*pz - qz*py
    cy = qz*px - qx*pz
    cz = qx*py - qy*px
    cx, cy, cz = 2.0*cx, 2.0*cy, 2.0*cz
    rx = px + qw*cx + qy*cz - qz*cy
    ry = py + qw*cy + qz*cx - qx*cz
    rz = pz + qw*cz + qx*cy - qy*cx
    return rx + tx, ry + ty, rz + tz


def _project(point, projection):
    """Project an optical-frame point with CameraInfo.P; return pixel or None."""
    if len(projection) != 12:
        raise VisionContractError("camera_info.p must contain 12 values")
    if not all(math.isfinite(float(value)) for value in projection):
        raise VisionContractError("camera_info.p values must be finite")
    x, y, z = point
    p = projection
    denominator = p[8]*x + p[9]*y + p[10]*z + p[11]
    if denominator <= 1e-9:
        return None
    u = (p[0]*x + p[1]*y + p[2]*z + p[3]) / denominator
    v = (p[4]*x + p[5]*y + p[6]*z + p[7]) / denominator
    if not math.isfinite(u) or not math.isfinite(v):
        return None
    return u, v


def _median(values):
    return statistics.median(values)


def _clusters(points, range_jump_m):
    """Group projected rays by beam adjacency and range continuity."""
    clusters = []
    current = []
    for candidate in points:
        if current:
            previous = current[-1]
            adjacent = candidate["index"] - previous["index"] <= 1
            continuous = abs(candidate["laser_range_m"] - previous["laser_range_m"]) <= range_jump_m
            if not adjacent or not continuous:
                clusters.append(current)
                current = []
        current.append(candidate)
    if current:
        clusters.append(current)
    return clusters


def fuse_scan_with_detections(response, camera_info, scan,
                              camera_from_laser, base_from_laser, *,
                              max_scan_skew_ns=50_000_000,
                              max_camera_info_skew_ns=50_000_000,
                              cluster_range_jump_m=0.15):
    """Associate each Contract-v1 pixel box with projected LaserScan returns.

    Inputs are plain mappings so this function stays ROS-free. ``camera_info``
    must contain stamp_ns, frame_id, width, height, projection (CameraInfo.P),
    and distortion (CameraInfo.D). ``scan`` must contain stamp_ns, frame_id,
    angle_min, angle_increment, range_min, range_max, and ranges. Transforms
    map ``scan.frame_id`` to the camera optical projection frame and ``base_link``.

    The nearest continuous candidate cluster is returned as a planar surface
    measurement. A missing candidate is an explicit unmatched result. This is
    not a claim of semantic object identity and must not be sent directly to
    robot control.
    """
    response = validate_response(response)
    if response["status"] != "ok":
        return {"state": response["status"], "detections": []}
    if not isinstance(max_scan_skew_ns, int) or isinstance(max_scan_skew_ns, bool) or max_scan_skew_ns < 0:
        raise ValueError("max_scan_skew_ns must be a nonnegative integer")
    if (not isinstance(max_camera_info_skew_ns, int)
            or isinstance(max_camera_info_skew_ns, bool)
            or max_camera_info_skew_ns < 0):
        raise ValueError("max_camera_info_skew_ns must be a nonnegative integer")
    if (isinstance(cluster_range_jump_m, bool)
            or not isinstance(cluster_range_jump_m, (int, float))
            or not math.isfinite(float(cluster_range_jump_m))
            or cluster_range_jump_m <= 0):
        raise ValueError("cluster_range_jump_m must be finite and positive")

    capture = response["result"]["capture"]
    if capture["clock_domain"] != "ros_sim":
        raise VisionContractError("LiDAR fusion requires a ros_sim capture")
    if not isinstance(camera_info, dict) or not isinstance(scan, dict):
        raise VisionContractError("camera_info and scan must be mappings")
    for mapping, field in ((camera_info, "camera_info"), (scan, "scan")):
        if not isinstance(mapping.get("stamp_ns"), int) or isinstance(mapping.get("stamp_ns"), bool):
            raise VisionContractError(f"{field}.stamp_ns must be an integer")
        if not isinstance(mapping.get("frame_id"), str) or not mapping["frame_id"].strip():
            raise VisionContractError(f"{field}.frame_id must be non-empty")

    if abs(capture["stamp_ns"] - camera_info["stamp_ns"]) > max_camera_info_skew_ns:
        raise VisionContractError("CameraInfo is not synchronized with the detection capture")
    if abs(capture["stamp_ns"] - scan["stamp_ns"]) > max_scan_skew_ns:
        raise VisionContractError("LaserScan is outside the allowed capture-time window")
    if capture["frame_id"] != camera_info["frame_id"]:
        raise VisionContractError("detection and CameraInfo frame IDs do not match")
    if (capture["width"] != camera_info.get("width")
            or capture["height"] != camera_info.get("height")):
        raise VisionContractError("detection and CameraInfo dimensions do not match")

    distortion = camera_info.get("distortion", [])
    if not isinstance(distortion, (list, tuple)) or not all(
            isinstance(value, (int, float)) and math.isfinite(float(value))
            for value in distortion):
        raise VisionContractError("CameraInfo distortion coefficients are invalid")
    if any(abs(float(value)) > 1e-12 for value in distortion):
        raise VisionContractError(
            "nonzero camera distortion requires rectification before LiDAR projection")
    projection = camera_info.get("projection")
    if not isinstance(projection, (list, tuple)):
        raise VisionContractError("camera_info.projection must be a sequence")
    if scan["frame_id"] == camera_info["frame_id"]:
        raise VisionContractError("LaserScan frame must differ from the camera frame")

    angle_min = scan.get("angle_min")
    angle_increment = scan.get("angle_increment")
    range_min = scan.get("range_min")
    range_max = scan.get("range_max")
    ranges = scan.get("ranges")
    numeric = (angle_min, angle_increment, range_min, range_max)
    if (not all(isinstance(value, (int, float)) and math.isfinite(float(value))
                for value in numeric)
            or angle_increment == 0 or range_min < 0 or range_max <= range_min
            or not isinstance(ranges, (list, tuple))):
        raise VisionContractError("LaserScan geometry or ranges are invalid")

    detections_out = []
    for detection in response["result"]["detections"]:
        x_min, y_min, x_max, y_max = detection["bbox_xyxy_px"]
        candidates = []
        for index, raw_range in enumerate(ranges):
            if (isinstance(raw_range, bool)
                    or not isinstance(raw_range, (int, float))
                    or not math.isfinite(float(raw_range))
                    or raw_range < range_min or raw_range > range_max):
                continue
            distance = float(raw_range)
            angle = float(angle_min) + index*float(angle_increment)
            point_laser = (distance*math.cos(angle), distance*math.sin(angle), 0.0)
            point_camera = transform_point(point_laser, camera_from_laser)
            pixel = _project(point_camera, projection)
            if pixel is None:
                continue
            u, v = pixel
            if not (x_min <= u < x_max and y_min <= v < y_max):
                continue
            point_base = transform_point(point_laser, base_from_laser)
            candidates.append({
                "index": index,
                "pixel": [u, v],
                "laser_range_m": distance,
                "point_base": point_base,
                "range_base_m": math.hypot(point_base[0], point_base[1]),
                "bearing_base_rad": math.atan2(point_base[1], point_base[0]),
            })

        clusters = _clusters(candidates, float(cluster_range_jump_m))
        if not clusters:
            lidar = {"status": "unmatched", "candidate_points": 0}
        else:
            # Choose the closest surface cluster. The full cluster count is
            # exposed so callers can flag crowded/ambiguous image regions.
            cluster = min(clusters, key=lambda group: _median(
                [point["range_base_m"] for point in group]))
            lidar = {
                "status": "matched" if len(clusters) == 1 else "nearest_cluster",
                "candidate_points": len(candidates),
                "cluster_count": len(clusters),
                "cluster_points": len(cluster),
                "point_base_link_m": [
                    _median([point["point_base"][axis] for point in cluster])
                    for axis in range(3)
                ],
                "range_m": _median([point["range_base_m"] for point in cluster]),
                "bearing_rad": _median([point["bearing_base_rad"] for point in cluster]),
                "projected_pixel_bounds": [
                    min(point["pixel"][0] for point in cluster),
                    min(point["pixel"][1] for point in cluster),
                    max(point["pixel"][0] for point in cluster),
                    max(point["pixel"][1] for point in cluster),
                ],
            }
        detections_out.append({
            "class_id": detection["class_id"],
            "label": detection["label"],
            "confidence": detection["confidence"],
            "bbox_xyxy_px": detection["bbox_xyxy_px"],
            "lidar": lidar,
        })

    return {
        "schema_version": "1.0",
        "state": "ok" if detections_out else "empty",
        "capture": dict(capture),
        "scan": {
            "stamp_ns": scan["stamp_ns"],
            "frame_id": scan["frame_id"],
            "skew_ns": scan["stamp_ns"] - capture["stamp_ns"],
        },
        "output_frame": "base_link",
        "position_semantics": "median_lidar_surface_return_in_scan_plane",
        "detections": detections_out,
    }
