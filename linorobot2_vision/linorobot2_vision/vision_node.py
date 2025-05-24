#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs_py.point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CompressedImage
import requests

class ReprojectionNode(Node):
    def __init__(self):
        super().__init__('reprojection')
        self.bridge = CvBridge()
        self.lp = lg.LaserProjection()

        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.image_topic = self.declare_parameter("image_topic", "/image_raw").value
        self.calib_file = self.declare_parameter("calib_file", "calib.yaml").value
        self.config_file = self.declare_parameter("config_file", "config.yaml").value
        self.laser_point_radius = self.declare_parameter("laser_point_radius", 3).value
        self.time_diff = self.declare_parameter("time_diff", 0.1).value
        # maximum distance for laser points to be visualised (metres)
        self.max_laser_distance = self.declare_parameter("max_laser_distance", 5.0).value

        # Parameters for object-specific depth filtering
        self.min_points_for_depth_filter = self.declare_parameter("min_points_for_depth_filter", 5).value
        self.object_depth_filter_tolerance = self.declare_parameter("object_depth_filter_tolerance", 0.25).value

        self.load_calibration()
        self.load_camera_config()

        self.pub = self.create_publisher(Image, "/reprojection", 10)
        self.scan_sub = Subscriber(self, LaserScan, self.scan_topic)
        self.image_sub = Subscriber(self, CompressedImage, self.image_topic)
        self.ts = ApproximateTimeSynchronizer([self.scan_sub, self.image_sub], 10, self.time_diff)
        self.ts.registerCallback(self.callback)

        # print the init setup:
        self.get_logger().info("scan_topic: %s" % self.scan_topic)
        self.get_logger().info("image_topic: %s" % self.image_topic)
        self.get_logger().info("calib_file: %s" % self.calib_file)
        self.get_logger().info("config_file: %s" % self.config_file)
        self.get_logger().info("laser_point_radius: %d" % self.laser_point_radius)
        self.get_logger().info("time_diff: %f" % self.time_diff)
        self.get_logger().info("max_laser_distance: %.2f" % self.max_laser_distance)
        self.get_logger().info(f"Min points for depth filter: {self.min_points_for_depth_filter}")
        self.get_logger().info(f"Object depth filter tolerance: {self.object_depth_filter_tolerance}m")
        
        # API and detection settings
        self.api_url = "http://localhost:3000/api/detect"
        self.api_confidence_threshold = self.declare_parameter("api_confidence_threshold", 0.3).value
        self.label_colors = {}
        self.get_logger().info(f"API URL: {self.api_url}")
        self.get_logger().info(f"API Confidence Threshold: {self.api_confidence_threshold}")


    def load_calibration(self):
        with open(self.calib_file, 'r') as f:
            data = f.read().split()
            qx = float(data[0])
            qy = float(data[1])
            qz = float(data[2])
            qw = float(data[3])
            tx = float(data[4])
            ty = float(data[5])
            tz = float(data[6])
        q = Quaternion(qw, qx, qy, qz).transformation_matrix
        q[0, 3] = tx
        q[1, 3] = ty
        q[2, 3] = tz
        print("Extrinsic parameter - camera to laser")
        print(q)
        self.tvec = q[:3, 3]
        rot_mat = q[:3, :3]
        self.rvec, _ = cv2.Rodrigues(rot_mat)
        self.q = q

    def load_camera_config(self):
        with open(self.config_file, 'r') as f:
            f.readline()
            config = yaml.safe_load(f)
            self.lens = config['lens']
            fx = float(config['fx'])
            fy = float(config['fy'])
            cx = float(config['cx'])
            cy = float(config['cy'])
            k1 = float(config['k1'])
            k2 = float(config['k2'])
            p1 = float(config['p1/k3'])
            p2 = float(config['p2/k4'])
        self.K = np.matrix([[fx, 0.0, cx],
                            [0.0, fy, cy],
                            [0.0, 0.0, 1.0]])
        self.D = np.array([k1, k2, p1, p2])
        print("Camera parameters")
        print("Lens = %s" % self.lens)
        print("K =")
        print(self.K)
        print("D =")
        print(self.D)

    def get_z(self, T_cam_world, T_world_pc, K):
        R = T_cam_world[:3, :3]
        t = T_cam_world[:3, 3]
        proj_mat = np.dot(K, np.hstack((R, t[:, np.newaxis])))
        xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
        xy_hom = np.dot(proj_mat, xyz_hom.T).T
        z = xy_hom[:, -1]
        z = np.asarray(z).squeeze()
        return z

    def extract(self, point):
        return [point[0], point[1], point[2]]

    def callback(self, scan, image):
        # Convert ROS2 builtin time to floating-point seconds to get an accurate difference
        image_time = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9
        scan_time  = scan.header.stamp.sec  + scan.header.stamp.nanosec  * 1e-9
        diff = abs(image_time - scan_time)
        self.get_logger().debug(f"time difference (image-scan): {diff:.6f} s")

        img = self.bridge.compressed_imgmsg_to_cv2(image)
        img_height, img_width = img.shape[:2]

        # Object detection via API
        _, img_encoded = cv2.imencode('.jpg', img)
        files = {'image': ('image.jpg', img_encoded.tobytes(), 'image/jpeg')}
        
        detections = []
        try:
            response = requests.post(self.api_url, files=files, timeout=2) # Added timeout
            response.raise_for_status()  # Raise an exception for HTTP errors (4xx or 5xx)
            api_response = response.json()

            # --- updated parsing logic ---
            if isinstance(api_response, dict):
                if 'detections' in api_response:           # new: expected response shape
                    detections = api_response['detections']
                elif 'box' in api_response:                # single-detection shorthand
                    detections = [api_response]
            elif isinstance(api_response, list):
                detections = api_response
            # --------------------------------

        except requests.exceptions.Timeout:
            self.get_logger().warn(f"API request timed out: {self.api_url}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"API request failed: {e}")
        except ValueError: # Includes JSONDecodeError
            self.get_logger().error(f"Failed to decode JSON response from API.")

        # First, project and filter all laser points
        cloud = self.lp.projectLaser(scan)
        points = pc2.read_points(cloud)
        objPoints = np.array(list(map(self.extract, points)))

        max_distance = self.max_laser_distance
        if objPoints.shape[0] > 0:
            objPoints = objPoints[np.linalg.norm(objPoints, axis=1) <= max_distance]
        
        img_points = np.array([])
        if objPoints.shape[0] > 0:
            if self.lens == 'pinhole':
                img_points, _ = cv2.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
            elif self.lens == 'fisheye':
                objPoints_reshaped = np.reshape(objPoints, (1, objPoints.shape[0], objPoints.shape[1]))
                img_points, _ = cv2.fisheye.projectPoints(objPoints_reshaped, self.rvec, self.tvec, self.K, self.D)
            img_points = np.squeeze(img_points)

        objPoints_filtered = np.array([])
        img_points_filtered = np.array([])

        if img_points.ndim == 2 and img_points.shape[0] > 0:
            valid_indices = np.where((img_points[:, 0] >= 0) & (img_points[:, 0] < img_width) &
                                     (img_points[:, 1] >= 0) & (img_points[:, 1] < img_height))[0]
            if len(valid_indices) > 0:
                objPoints_filtered = objPoints[valid_indices]
                img_points_filtered = img_points[valid_indices]
        
        if objPoints_filtered.shape[0] > 0:
            Z = self.get_z(self.q, objPoints_filtered, self.K)
            if Z.shape[0] == objPoints_filtered.shape[0]:
                valid_z_indices = Z > 0
                objPoints_filtered = objPoints_filtered[valid_z_indices]
                img_points_filtered = img_points_filtered[valid_z_indices]
            else:
                objPoints_filtered = np.array([])
                img_points_filtered = np.array([])
        else:
            objPoints_filtered = np.array([])
            img_points_filtered = np.array([])

        # Initialize colors for all filtered laser points to default (green)
        point_colors = None
        default_point_color = (0, 255, 0) # Green
        if img_points_filtered.ndim == 2 and img_points_filtered.shape[0] > 0:
            point_colors = [default_point_color] * img_points_filtered.shape[0]

        # Now, process detections and draw bounding boxes and distances
        for det in detections:
            if 'box' in det and 'label' in det and 'confidence' in det:
                confidence = float(det['confidence'])
                if confidence >= self.api_confidence_threshold:
                    box = [int(coord) for coord in det['box']] # [x_min, y_min, x_max, y_max]
                    label = det['label']

                    if label not in self.label_colors:
                        self.label_colors[label] = np.random.randint(0, 255, size=3).tolist()
                    color = self.label_colors[label] # This is the object's color

                    cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), color, 2)
                    text = f"{label}: {confidence:.2f}"
                    cv2.putText(img, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    # Calculate and display distance for the detected object
                    if img_points_filtered.ndim == 2 and img_points_filtered.shape[0] > 0:
                        x_min_obj, y_min_obj, x_max_obj, y_max_obj = box[0], box[1], box[2], box[3]
                        
                        points_in_bbox_mask = (img_points_filtered[:, 0] >= x_min_obj) & \
                                              (img_points_filtered[:, 0] <= x_max_obj) & \
                                              (img_points_filtered[:, 1] >= y_min_obj) & \
                                              (img_points_filtered[:, 1] <= y_max_obj)
                        
                        laser_points_3d_in_bbox = objPoints_filtered[points_in_bbox_mask]
                        # Get original indices of points within the bounding box (relative to img_points_filtered)
                        original_indices_in_bbox = np.where(points_in_bbox_mask)[0]
                        
                        if laser_points_3d_in_bbox.shape[0] > 0:
                            points_for_distance_calc = laser_points_3d_in_bbox 
                            indices_to_color_for_this_object = original_indices_in_bbox

                            # Attempt to refine points if enough are available
                            if laser_points_3d_in_bbox.shape[0] >= self.min_points_for_depth_filter:
                                point_distances_initial = np.linalg.norm(laser_points_3d_in_bbox, axis=1)
                                median_obj_dist = np.median(point_distances_initial)
                                
                                # refined_mask_for_bbox_subset is a mask for laser_points_3d_in_bbox (and for original_indices_in_bbox)
                                refined_mask_for_bbox_subset = np.abs(point_distances_initial - median_obj_dist) <= self.object_depth_filter_tolerance
                                
                                if np.any(refined_mask_for_bbox_subset): 
                                    points_for_distance_calc = laser_points_3d_in_bbox[refined_mask_for_bbox_subset]
                                    indices_to_color_for_this_object = original_indices_in_bbox[refined_mask_for_bbox_subset]
                                else:
                                    self.get_logger().warn(
                                        f"Depth filter for object '{label}' (median: {median_obj_dist:.2f}m, "
                                        f"{laser_points_3d_in_bbox.shape[0]} pts) "
                                        f"with tolerance {self.object_depth_filter_tolerance}m "
                                        f"removed all points. Using all {laser_points_3d_in_bbox.shape[0]} points in bbox."
                                    )
                            
                            # Update colors for the identified points
                            if point_colors is not None and indices_to_color_for_this_object.shape[0] > 0:
                                for idx in indices_to_color_for_this_object:
                                    point_colors[idx] = color # Assign object's color
                            
                            # Calculate average distance using the (potentially refined) set of points
                            if points_for_distance_calc.shape[0] > 0:
                                final_distances = np.linalg.norm(points_for_distance_calc, axis=1)
                                avg_distance = np.mean(final_distances)
                                
                                distance_text = f"{avg_distance:.2f}m"
                                
                                text_center_x = (x_min_obj + x_max_obj) // 2
                                text_center_y = (y_min_obj + y_max_obj) // 2
                                
                                (text_width, text_height), _ = cv2.getTextSize(distance_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                                text_origin_x = text_center_x - text_width // 2
                                text_origin_y = text_center_y + text_height // 2
                                
                                cv2.putText(img, distance_text, (text_origin_x, text_origin_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                self.get_logger().warn(f"Malformed detection object from API: {det}")

        if img_points_filtered.ndim == 2 and img_points_filtered.shape[0] > 0 and point_colors is not None:
            img_points_filtered_int = np.round(img_points_filtered).astype(np.int32)
            for i, point_coord_int in enumerate(img_points_filtered_int): # Use enumerate to get index i
                cv2.circle(img, tuple(point_coord_int), self.laser_point_radius, point_colors[i], 1) # Use point_colors[i]
        self.pub.publish(self.bridge.cv2_to_imgmsg(img))
        img = None  # Explicitly release the image

    def destroy(self):
        self.bridge = None

def main(args=None):
    rclpy.init(args=args)
    node = ReprojectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy() # Call the destroy method to release resources
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()