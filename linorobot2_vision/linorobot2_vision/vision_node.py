#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, LaserScan, PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Quaternion as GeoQuaternion # Added GeoQuaternion for clarity if needed elsewhere
from geometry_msgs.msg import Point as GeoPointMsg # For DetectedObject
from laser_geometry import LaserProjection
import yaml
import os
import zmq
import json
import traceback
from quaternion import quaternion as Quaternion # Changed import for Quaternion
from quaternion import as_rotation_matrix # Added import for as_rotation_matrix
from linorobot2_vision.msg import DetectedObject, DetectedObjectArray # Added for custom messages

class ReprojectionNode(Node):
    def __init__(self):
        super().__init__('reprojection')
        self.bridge = CvBridge()
        self.lp = LaserProjection()
        self.frame_counter = 0 # Add frame counter

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

        # ZeroMQ PUSH socket for sending images
        self.image_sender_endpoint = self.declare_parameter(
            "image_sender_endpoint", "tcp://localhost:5555"  # Changed to TCP
        ).get_parameter_value().string_value
        
        # ZeroMQ REQ socket for receiving detection results
        self.results_receiver_endpoint = self.declare_parameter(
            "results_receiver_endpoint", "tcp://localhost:5556"  # Changed to TCP
        ).get_parameter_value().string_value

        self.get_logger().info(f"ZMQ Image PUSH Endpoint: {self.image_sender_endpoint}")
        self.get_logger().info(f"ZMQ Results REQ Endpoint: {self.results_receiver_endpoint}")

        try:
            self.zmq_context = zmq.Context()
            
            # PUSH socket for sending images
            self.image_push_socket = self.zmq_context.socket(zmq.PUSH)
            # Set LINGER to 0 to prevent hanging on close if messages are queued
            self.image_push_socket.setsockopt(zmq.LINGER, 0) 
            # Set a send timeout (e.g., 1 second) to prevent indefinite blocking
            self.image_push_socket.setsockopt(zmq.SNDTIMEO, 1000) 
            self.image_push_socket.connect(self.image_sender_endpoint)
            self.get_logger().info(f"ZMQ PUSH socket connected to {self.image_sender_endpoint}")

            # REQ socket for results
            self.results_req_socket = self.zmq_context.socket(zmq.REQ)
            self.results_req_socket.setsockopt(zmq.LINGER, 0) # Prevent hanging on close
            self.results_req_socket.setsockopt(zmq.RCVTIMEO, 2000) # Timeout for receive
            self.results_req_socket.connect(self.results_receiver_endpoint)
            self.get_logger().info(f"ZMQ REQ socket connected to {self.results_receiver_endpoint}")

        except zmq.error.ZMQError as e:
            self.get_logger().error(f"Failed to initialize ZeroMQ sockets: {e}")
            # Handle error appropriately, maybe rclpy.shutdown() or raise an exception
            # For now, just log and the node might not function correctly.
            # Consider setting a flag to prevent operations if ZMQ fails.
            self.image_push_socket = None 
            self.results_req_socket = None
            self.zmq_context = None # Or handle context termination carefully
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during ZMQ initialization: {e}")
            self.image_push_socket = None
            self.results_req_socket = None
            self.zmq_context = None


        self.pub = self.create_publisher(Image, "/reprojection", 10)
        self.marked_scan_pub = self.create_publisher(LaserScan, "/marked_scan", 10) # New publisher
        self.detected_objects_pub = self.create_publisher(DetectedObjectArray, "vision/objects", 10) # New publisher for detected objects
        # Corrected message_filters imports and usage
        self.scan_sub = message_filters.Subscriber(self, LaserScan, self.scan_topic)
        self.image_sub = message_filters.Subscriber(self, CompressedImage, self.image_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.image_sub], 10, self.time_diff)
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
        
        # Create a quaternion object
        quat_obj = Quaternion(qw, qx, qy, qz)
        # Convert quaternion to rotation matrix
        rotation_matrix = as_rotation_matrix(quat_obj)

        # Create the 4x4 transformation matrix
        q_transform = np.eye(4)
        q_transform[:3, :3] = rotation_matrix
        q_transform[0, 3] = tx
        q_transform[1, 3] = ty
        q_transform[2, 3] = tz
        
        print("Extrinsic parameter - camera to laser")
        print(q_transform)
        self.tvec = q_transform[:3, 3]
        rot_mat = q_transform[:3, :3]
        self.rvec, _ = cv2.Rodrigues(rot_mat)
        self.q = q_transform

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
        # Ensure T_world_pc is 2D array for hstack
        if T_world_pc.ndim == 1:
            T_world_pc = T_world_pc.reshape(1, -1)
        xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
        xy_hom = np.dot(proj_mat, xyz_hom.T).T
        z = xy_hom[:, -1]
        z = np.asarray(z).squeeze()
        return z

    def callback(self, scan, image):
        # Convert ROS2 builtin time to floating-point seconds to get an accurate difference
        image_time = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9
        scan_time  = scan.header.stamp.sec  + scan.header.stamp.nanosec  * 1e-9
        diff = abs(image_time - scan_time)
        self.get_logger().debug(f"time difference (image-scan): {diff:.6f} s")

        self.frame_counter += 1 # Increment frame counter

        img = self.bridge.compressed_imgmsg_to_cv2(image)
        img_height, img_width = img.shape[:2]

        detections = [] # Initialize detections

        if self.frame_counter % 1 == 0: # Process every 2nd frame
            self.get_logger().debug(f"Processing frame {self.frame_counter} for object detection.")
            # Object detection via ZeroMQ
            _, img_encoded = cv2.imencode('.jpg', img)
            
            if img_encoded is None:
                self.get_logger().error("Failed to encode image to JPEG for ZMQ. Skipping ZMQ communication for this frame.")
                # Publish original image and default scan if encoding fails, then return
                # self.pub.publish(self.bridge.cv2_to_imgmsg(img)) # This will be handled later
                # # Consider publishing a default/empty marked_scan_msg as well
                # # For now, just returning to avoid further errors in this callback iteration
                # return # Removed to allow processing of laser scan even if ZMQ fails for this frame
            else:
                img_bytes_to_send = img_encoded.tobytes()
                if not img_bytes_to_send:
                    self.get_logger().error("Encoded image is empty. Skipping ZMQ communication for this frame.")
                    # self.pub.publish(self.bridge.cv2_to_imgmsg(img)) # Handled later
                    # return # Removed
                else:
                    try:
                        self.get_logger().debug(f"Attempting to send {len(img_bytes_to_send)} image bytes via ZMQ PUSH.")
                        # Send image
                        self.image_push_socket.send(img_bytes_to_send)
                        self.get_logger().debug(f"Successfully sent {len(img_bytes_to_send)} image bytes via ZMQ PUSH.")
                        
                        # Request and receive detections
                        self.get_logger().debug("Sending 'detect' signal via ZMQ REQ.")
                        self.results_req_socket.send_string("detect") # Send a simple request
                        self.get_logger().debug("'detect' signal sent. Polling for response...")
                        
                        # Wait for the response
                        poller = zmq.Poller()
                        poller.register(self.results_req_socket, zmq.POLLIN)
                        
                        # Wait for 1 second (1000ms)
                        if poller.poll(1000): 
                            detections_payload = self.results_req_socket.recv_json()

                            if isinstance(detections_payload, dict):
                                if 'detections' in detections_payload:
                                    detections = detections_payload['detections']
                                # Handle potential error message from server
                                elif 'error' in detections_payload:
                                    self.get_logger().error(f"Detection service error: {detections_payload['error']}")
                                    detections = []
                                else: # single-detection shorthand or other dict format
                                    detections = [detections_payload] if 'box' in detections_payload else []
                            elif isinstance(detections_payload, list):
                                detections = detections_payload
                            else:
                                self.get_logger().warn(f"Received unexpected detection format: {type(detections_payload)}")
                                detections = []
                        else:
                            self.get_logger().warn("Timeout waiting for detection results from ZMQ REP socket.")
                            # Attempt to recover the REQ socket
                            self.get_logger().info("Attempting to recover ZMQ REQ socket...")
                            self.results_req_socket.close()
                            self.results_req_socket = self.zmq_context.socket(zmq.REQ)
                            self.results_req_socket.setsockopt(zmq.LINGER, 0) # Set LINGER to 0 to prevent hanging on close
                            self.results_req_socket.setsockopt(zmq.RCVTIMEO, 2000) # Set a timeout for receive operations
                            self.results_req_socket.connect(self.results_receiver_endpoint) # Corrected endpoint
                            self.get_logger().info(f"ZMQ REQ socket reconnected to {self.results_receiver_endpoint}")

                    except zmq.error.Again as e: # Timeout
                        # This specifically catches timeout on results_req_socket.recv_json() due to RCVTIMEO
                        self.get_logger().warn(f"ZeroMQ REQ socket timeout waiting for detection results: {e}")
                        detections = [] # Proceed without detections
                    except zmq.error.ZMQError as e:
                        # This can catch other ZMQ errors, including potential send errors if not EAGAIN
                        self.get_logger().error(f"ZeroMQ communication error: {e} (errno: {e.errno if hasattr(e, 'errno') else 'N/A'})")
                        if hasattr(e, 'errno') and e.errno == zmq.EFSM:
                             self.get_logger().error("ZMQ EFSM error: REQ socket might be in a bad state (e.g. send/recv out of sequence). Attempting recovery.")
                             # Attempt recovery for REQ socket specifically if it's an FSM error
                             try:
                                self.results_req_socket.close()
                                self.results_req_socket = self.zmq_context.socket(zmq.REQ)
                                self.results_req_socket.setsockopt(zmq.LINGER, 0)
                                self.results_req_socket.setsockopt(zmq.RCVTIMEO, 2000)
                                self.results_req_socket.connect(self.results_receiver_endpoint)
                                self.get_logger().info(f"ZMQ REQ socket reconnected after EFSM error to {self.results_receiver_endpoint}")
                             except Exception as recovery_e:
                                self.get_logger().error(f"Failed to recover REQ socket after EFSM error: {recovery_e}")
                        detections = [] # Proceed without detections
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"Failed to decode JSON response from ZeroMQ: {e}")
                        detections = []
                    except Exception as e:
                        self.get_logger().error(f"Error during ZeroMQ detection processing: {e}\\nTraceback: {traceback.format_exc()}")
                        detections = []
        else:
            self.get_logger().debug(f"Skipping object detection for frame {self.frame_counter}.")
            # Detections list will remain empty if not processed
        
        # --- Use laser_geometry to project laser scan to PointCloud2 ---
        try:
            self.get_logger().debug("Starting laser scan processing.")
            # Using laser_geometry to project laser scan to 3D points
            cloud = self.lp.projectLaser(scan)
            self.get_logger().debug("Laser scan projected to PointCloud2.")
            
            # Extract x, y, z from cloud but track indices manually
            points_data = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
            self.get_logger().debug(f"Extracted {len(points_data)} points from PointCloud2.")
            
            if not points_data:
                self.get_logger().debug("No valid laser points from scan to process for reprojection.")
                self.pub.publish(self.bridge.cv2_to_imgmsg(img)) # Publish original image
                marked_scan_msg = LaserScan()
                marked_scan_msg.header = scan.header
                marked_scan_msg.angle_min = scan.angle_min
                marked_scan_msg.angle_max = scan.angle_max
                marked_scan_msg.angle_increment = scan.angle_increment
                marked_scan_msg.time_increment = scan.time_increment
                marked_scan_msg.scan_time = scan.scan_time
                marked_scan_msg.range_min = scan.range_min
                marked_scan_msg.range_max = scan.range_max
                marked_scan_msg.ranges = list(scan.ranges)
                marked_scan_msg.intensities = [0.0] * len(scan.ranges)
                self.marked_scan_pub.publish(marked_scan_msg)
                img = None
                return
            
            # Map point cloud points back to original scan indices
            # This is an approximation based on angle calculation
            self.get_logger().debug("Processing points_data to create objPoints_in_laser_frame.")
            valid_points_xyz = []
            for idx, p_candidate in enumerate(points_data):
                if hasattr(p_candidate, '__getitem__') and hasattr(p_candidate, '__len__') and len(p_candidate) == 3:
                    try:
                        # Ensure elements can be converted to float for numpy array
                        x = float(p_candidate[0])
                        y = float(p_candidate[1])
                        z = float(p_candidate[2])
                        valid_points_xyz.append((x, y, z))
                    except (ValueError, TypeError) as ve:
                        self.get_logger().warn(f"Point at index {idx} has non-numeric data: {p_candidate}, error: {ve}. Skipping.")
                else:
                    self.get_logger().warn(f"Point at index {idx} is malformed (not a 3-element sequence): {p_candidate}, type: {type(p_candidate)}. Skipping.")
            
            if not valid_points_xyz:
                self.get_logger().warn("No valid XYZ points extracted from PointCloud2 after filtering. objPoints_in_laser_frame will be empty.")
                objPoints_in_laser_frame = np.array([]) 
            else:
                objPoints_in_laser_frame = np.array(valid_points_xyz)
            
            self.get_logger().debug(f"objPoints_in_laser_frame shape after processing: {objPoints_in_laser_frame.shape}")

            # Calculate angles from x,y coordinates of points (atan2(y,x))
            if objPoints_in_laser_frame.shape[0] == 0:
                self.get_logger().debug("objPoints_in_laser_frame is empty. Initializing point_angles as empty.")
                point_angles = np.array([])
            elif objPoints_in_laser_frame.shape[0] == 1:
                self.get_logger().debug("Calculating angle for a single point.")
                y_coord = objPoints_in_laser_frame[0, 1]
                x_coord = objPoints_in_laser_frame[0, 0]
                angle_scalar = np.arctan2(y_coord, x_coord)
                point_angles = np.array([angle_scalar]) 
            else: # objPoints_in_laser_frame.shape[0] > 1
                self.get_logger().debug("Calculating angles for multiple points.")
                point_angles = np.arctan2(objPoints_in_laser_frame[:, 1], objPoints_in_laser_frame[:, 0])

            self.get_logger().debug(f"Initial point_angles shape: {point_angles.shape}, size: {point_angles.size}")

            # At this point, point_angles should be 1D (possibly empty if objPoints_in_laser_frame was empty)
            point_angles = np.atleast_1d(point_angles)
            self.get_logger().debug(f"point_angles shape after atleast_1d: {point_angles.shape}, size: {point_angles.size}")
            
            # Normalize angles to be in the same range as scan angles
            if point_angles.size > 0: # Check if there are any angles to normalize
                if point_angles.size == 1:
                    self.get_logger().debug(f"Normalizing single angle: {point_angles[0]}")
                    while point_angles[0] < scan.angle_min:
                        point_angles[0] += 2 * np.pi
                    while point_angles[0] > scan.angle_max:
                        point_angles[0] -= 2 * np.pi
                    self.get_logger().debug(f"Normalized single angle: {point_angles[0]}")
                elif point_angles.size > 1:
                    self.get_logger().debug("Normalizing multiple angles.")
                    # Handle angles less than minimum
                    mask_lt = point_angles < scan.angle_min
                    while np.any(mask_lt):
                        point_angles[mask_lt] += 2 * np.pi
                        mask_lt = point_angles < scan.angle_min # Re-evaluate mask
                    
                    # Handle angles greater than maximum
                    mask_gt = point_angles > scan.angle_max
                    while np.any(mask_gt):
                        point_angles[mask_gt] -= 2 * np.pi
                        mask_gt = point_angles > scan.angle_max # Re-evaluate mask
                    self.get_logger().debug("Finished normalizing multiple angles.")
            else:
                self.get_logger().debug("point_angles is empty, skipping normalization.")
                
            # Find closest index in scan for each point
            # This line could fail if point_angles is empty, though the normalization block now checks for .size > 0
            if point_angles.size > 0:
                indices = np.round((point_angles - scan.angle_min) / scan.angle_increment).astype(int)
                self.get_logger().debug(f"Calculated indices shape: {indices.shape}")
                # Ensure indices are within valid range
                indices = np.clip(indices, 0, len(scan.ranges) - 1)
                self.get_logger().debug(f"Clipped indices shape: {indices.shape}")
            else:
                indices = np.array([], dtype=int) # Ensure indices is an empty int array if no angles
                self.get_logger().debug("point_angles was empty, so indices is an empty array.")

            # Now we have our tracked original indices
            current_original_scan_indices = indices
            self.get_logger().debug(f"current_original_scan_indices shape: {current_original_scan_indices.shape}")
            self.get_logger().debug("Finished laser scan processing successfully.")
            
        except Exception as e:
            self.get_logger().error(f"Error processing laser scan: {e}\\nTraceback: {traceback.format_exc()}") # Corrected logging
            self.pub.publish(self.bridge.cv2_to_imgmsg(img)) # Publish original image
            marked_scan_msg = LaserScan()
            marked_scan_msg.header = scan.header
            marked_scan_msg.angle_min = scan.angle_min
            marked_scan_msg.angle_max = scan.angle_max
            marked_scan_msg.angle_increment = scan.angle_increment
            marked_scan_msg.time_increment = scan.time_increment
            marked_scan_msg.scan_time = scan.scan_time
            marked_scan_msg.range_min = scan.range_min
            marked_scan_msg.range_max = scan.range_max
            marked_scan_msg.ranges = list(scan.ranges) # Keep original ranges
            marked_scan_msg.intensities = [0.0] * len(scan.ranges) # No intensity markings
            self.marked_scan_pub.publish(marked_scan_msg)
            img = None
            return

        # Filter points by max_laser_distance
        max_distance = self.max_laser_distance
        # Add a check for objPoints_in_laser_frame existence before trying to access its shape
        if 'objPoints_in_laser_frame' in locals() and objPoints_in_laser_frame.shape[0] > 0:
            self.get_logger().debug(f"Filtering by max_laser_distance. objPoints_in_laser_frame shape: {objPoints_in_laser_frame.shape}, current_original_scan_indices shape: {current_original_scan_indices.shape}")
            # Ensure current_original_scan_indices is not empty and matches dimensions if we are to mask it
            if current_original_scan_indices.size == objPoints_in_laser_frame.shape[0]:
                norm_mask = np.linalg.norm(objPoints_in_laser_frame, axis=1) <= max_distance
                objPoints_in_laser_frame = objPoints_in_laser_frame[norm_mask]
                current_original_scan_indices = current_original_scan_indices[norm_mask]
                self.get_logger().debug(f"After distance filtering: objPoints shape {objPoints_in_laser_frame.shape}, indices shape {current_original_scan_indices.shape}")
            else:
                self.get_logger().warn(f"Skipping distance filtering mask on current_original_scan_indices due to mismatch or empty: indices size {current_original_scan_indices.size}, objPoints shape {objPoints_in_laser_frame.shape[0]}")
                # Filter only objPoints_in_laser_frame if indices are problematic
                norm_mask = np.linalg.norm(objPoints_in_laser_frame, axis=1) <= max_distance
                objPoints_in_laser_frame = objPoints_in_laser_frame[norm_mask]
                # current_original_scan_indices might become stale here if not filtered, or we might need to re-calculate/invalidate
                # For now, let's log and proceed, this might be a source of issues if indices are used later without re-evaluation
                self.get_logger().debug(f"After distance filtering (only objPoints): objPoints shape {objPoints_in_laser_frame.shape}")

        else:
            self.get_logger().debug("objPoints_in_laser_frame is not available or empty before distance filtering.")
        
        img_points = np.array([])
        # Add a check for objPoints_in_laser_frame existence
        if 'objPoints_in_laser_frame' in locals() and objPoints_in_laser_frame.shape[0] > 0:
            self.get_logger().debug(f"Projecting points to image. objPoints_in_laser_frame shape: {objPoints_in_laser_frame.shape}")
            if self.lens == 'pinhole':
                img_points, _ = cv2.projectPoints(objPoints_in_laser_frame, self.rvec, self.tvec, self.K, self.D)
            elif self.lens == 'fisheye':
                objPoints_reshaped = np.reshape(objPoints_in_laser_frame, (1, objPoints_in_laser_frame.shape[0], objPoints_in_laser_frame.shape[1]))
                img_points, _ = cv2.fisheye.projectPoints(objPoints_reshaped, self.rvec, self.tvec, self.K, self.D)
            img_points = np.squeeze(img_points)
            self.get_logger().debug(f"img_points shape after projection: {img_points.shape}")
        else:
            self.get_logger().debug("objPoints_in_laser_frame is not available or empty, skipping projection to image.")

        objPoints_filtered_img_bounds = np.array([])
        img_points_filtered_img_bounds = np.array([])
        original_indices_filtered_img_bounds = np.array([])

        if img_points.ndim == 2 and img_points.shape[0] > 0: # Ensure img_points is 2D
            valid_indices_mask = (img_points[:, 0] >= 0) & (img_points[:, 0] < img_width) & \
                                 (img_points[:, 1] >= 0) & (img_points[:, 1] < img_height)
            if np.any(valid_indices_mask): # Check if any points are valid
                objPoints_filtered_img_bounds = objPoints_in_laser_frame[valid_indices_mask]
                img_points_filtered_img_bounds = img_points[valid_indices_mask]
                original_indices_filtered_img_bounds = current_original_scan_indices[valid_indices_mask]
        
        objPoints_final_filtered = np.array([])
        img_points_final_filtered = np.array([])
        original_indices_final_filtered = np.array([])

        if objPoints_filtered_img_bounds.shape[0] > 0:
            Z = self.get_z(self.q, objPoints_filtered_img_bounds, self.K)
            if Z.ndim > 0 and Z.shape[0] == objPoints_filtered_img_bounds.shape[0]: # Check Z is valid
                valid_z_indices_mask = Z > 0
                if np.any(valid_z_indices_mask):
                    objPoints_final_filtered = objPoints_filtered_img_bounds[valid_z_indices_mask]
                    img_points_final_filtered = img_points_filtered_img_bounds[valid_z_indices_mask]
                    original_indices_final_filtered = original_indices_filtered_img_bounds[valid_z_indices_mask]
            else: # Z calculation failed or returned scalar for single point
                if Z > 0 and objPoints_filtered_img_bounds.shape[0] == 1: # single point case
                    objPoints_final_filtered = objPoints_filtered_img_bounds
                    img_points_final_filtered = img_points_filtered_img_bounds
                    original_indices_final_filtered = original_indices_filtered_img_bounds


        # Initialize colors for all filtered laser points to default (green)
        point_colors = None
        default_point_color = (0, 255, 0) # Green
        if img_points_final_filtered.ndim == 2 and img_points_final_filtered.shape[0] > 0:
            point_colors = [default_point_color] * img_points_final_filtered.shape[0]

        # For publishing marked scan
        all_object_scan_indices = [] # List of (label, list_of_indices)
        detected_objects_list = [] # List to store DetectedObject messages
        next_tracking_id = 0 # Simple tracking ID counter

        # Now, process detections and draw rectangles around grouped points with annotations
        for det in detections:
            if 'box' in det and 'label' in det and 'confidence' in det:
                confidence = float(det['confidence'])
                if confidence < 0.5:
                    continue  # Skip detections with confidence under 50%
                if confidence >= self.api_confidence_threshold:
                    box = [int(coord) for coord in det['box']] 
                    label = det['label']

                    if label not in self.label_colors:
                        self.label_colors[label] = np.random.randint(0, 255, size=3).tolist()
                    color = self.label_colors[label]

                    if img_points_final_filtered.ndim == 2 and img_points_final_filtered.shape[0] > 0:
                        x_min_obj, y_min_obj, x_max_obj, y_max_obj = box[0], box[1], box[2], box[3]
                        points_in_bbox_mask = (img_points_final_filtered[:, 0] >= x_min_obj) & \
                                              (img_points_final_filtered[:, 0] <= x_max_obj) & \
                                              (img_points_final_filtered[:, 1] >= y_min_obj) & \
                                              (img_points_final_filtered[:, 1] <= y_max_obj)
                        
                        laser_points_3d_in_bbox = objPoints_final_filtered[points_in_bbox_mask]
                        img_points_in_bbox = img_points_final_filtered[points_in_bbox_mask]
                        original_indices_in_bbox = original_indices_final_filtered[points_in_bbox_mask]
                        
                        current_object_scan_indices = np.array([]) # For this specific object

                        if laser_points_3d_in_bbox.shape[0] > 0:
                            points_for_distance_calc = laser_points_3d_in_bbox
                            indices_to_color_for_this_object_mask = np.ones(laser_points_3d_in_bbox.shape[0], dtype=bool) # Mask relative to laser_points_3d_in_bbox
                            current_object_scan_indices = original_indices_in_bbox # Default to all in bbox

                            if laser_points_3d_in_bbox.shape[0] >= self.min_points_for_depth_filter:
                                point_distances_initial = np.linalg.norm(laser_points_3d_in_bbox, axis=1)
                                median_obj_dist = np.median(point_distances_initial)
                                refined_mask_for_bbox_subset = np.abs(point_distances_initial - median_obj_dist) <= self.object_depth_filter_tolerance
                                
                                if np.any(refined_mask_for_bbox_subset): 
                                    points_for_distance_calc = laser_points_3d_in_bbox[refined_mask_for_bbox_subset]
                                    img_points_in_bbox = img_points_in_bbox[refined_mask_for_bbox_subset] # Update img_points_in_bbox as well
                                    indices_to_color_for_this_object_mask = refined_mask_for_bbox_subset
                                    current_object_scan_indices = original_indices_in_bbox[refined_mask_for_bbox_subset]
                                else:
                                    self.get_logger().warn(
                                        f"Depth filter for object '{label}' (median: {median_obj_dist:.2f}m, "
                                        f"{laser_points_3d_in_bbox.shape[0]} pts) "
                                        f"with tolerance {self.object_depth_filter_tolerance}m "
                                        f"removed all points. Using all {laser_points_3d_in_bbox.shape[0]} points in bbox."
                                    )
                            
                            if point_colors is not None: # Ensure point_colors is initialized
                                # original_indices_in_bbox are indices into objPoints_final_filtered etc.
                                # We need indices relative to img_points_final_filtered for point_colors
                                # This requires finding where points_in_bbox_mask is true
                                true_indices_in_final_filtered = np.where(points_in_bbox_mask)[0]
                                if indices_to_color_for_this_object_mask.shape[0] == np.sum(points_in_bbox_mask): # Check if mask length matches
                                    for i, original_idx_in_ff in enumerate(true_indices_in_final_filtered):
                                        if indices_to_color_for_this_object_mask[i]: # if this point (from bbox) survived depth filter
                                            if original_idx_in_ff < len(point_colors):
                                                 point_colors[original_idx_in_ff] = color


                            if points_for_distance_calc.shape[0] > 0:
                                all_object_scan_indices.append((label, current_object_scan_indices.tolist())) # Store for marked scan

                                final_distances = np.linalg.norm(points_for_distance_calc, axis=1)
                                avg_distance = np.mean(final_distances)
                                
                                # Create DetectedObject message
                                detected_obj = DetectedObject()
                                detected_obj.name = label
                                detected_obj.distance = float(avg_distance)
                                detected_obj.tracking_id = next_tracking_id
                                next_tracking_id += 1
                                
                                # Convert laser points to geometry_msgs/Point
                                for laser_point in points_for_distance_calc:
                                    geo_point = GeoPointMsg()
                                    geo_point.x = float(laser_point[0])
                                    geo_point.y = float(laser_point[1])
                                    geo_point.z = float(laser_point[2])
                                    detected_obj.points.append(geo_point)
                                
                                detected_objects_list.append(detected_obj)
                                self.get_logger().debug(f"Created DetectedObject: {label}, distance: {avg_distance:.2f}m, points: {len(detected_obj.points)}")
                                
                                # Draw rectangle around the detected object (using original box coordinates)
                                cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), color, 2)

                                # Annotate with label, distance, and confidence
                                annotation = f"{label}: {avg_distance:.2f}m ({confidence:.2f})"
                                (text_width, text_height), _ = cv2.getTextSize(annotation, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                                text_origin_x = box[0] + (box[2] - box[0]) // 2 - text_width // 2
                                text_origin_y = box[1] - 5 if box[1] - 5 > text_height else box[1] + text_height + 5
                                cv2.putText(img, annotation, (text_origin_x, text_origin_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                self.get_logger().warn(f"Malformed detection object from API: {det}")

        # Draw circles for all (filtered) laser points on the image
        if img_points_final_filtered.ndim == 2 and img_points_final_filtered.shape[0] > 0 and point_colors is not None:
            img_points_filtered_int = np.round(img_points_final_filtered).astype(np.int32)
            for i, point_coord_int in enumerate(img_points_filtered_int):
                if i < len(point_colors): # Ensure index is within bounds
                    cv2.circle(img, tuple(point_coord_int), self.laser_point_radius, point_colors[i], 1)
        
        self.pub.publish(self.bridge.cv2_to_imgmsg(img))
        img = None

        # --- New: Create and publish marked LaserScan ---
        marked_scan_msg = LaserScan()
        marked_scan_msg.header = scan.header # Use original scan's header for timestamp and frame_id
        marked_scan_msg.angle_min = scan.angle_min
        marked_scan_msg.angle_max = scan.angle_max
        marked_scan_msg.angle_increment = scan.angle_increment
        marked_scan_msg.time_increment = scan.time_increment
        marked_scan_msg.scan_time = scan.scan_time
        marked_scan_msg.range_min = scan.range_min
        marked_scan_msg.range_max = scan.range_max
        marked_scan_msg.ranges = list(scan.ranges) # Copy of original ranges
        marked_scan_msg.intensities = [0.0] * len(scan.ranges) # Default intensity

        object_label_to_intensity_value = {}
        next_intensity = 100.0 # Starting intensity for first object type
        intensity_increment = 50.0

        for obj_label, obj_indices in all_object_scan_indices:
            if not obj_indices: continue # Skip if no indices for this object
            
            intensity_to_assign = 0.0
            if obj_label not in object_label_to_intensity_value:
                object_label_to_intensity_value[obj_label] = next_intensity
                intensity_to_assign = next_intensity
                next_intensity += intensity_increment
            else:
                intensity_to_assign = object_label_to_intensity_value[obj_label]
            
            for original_idx in obj_indices:
                if 0 <= original_idx < len(marked_scan_msg.intensities):
                     marked_scan_msg.intensities[original_idx] = intensity_to_assign
        
        self.marked_scan_pub.publish(marked_scan_msg)
        # --- End New ---

        # --- New: Create and publish DetectedObjectArray ---
        detected_objects_msg = DetectedObjectArray()
        detected_objects_msg.header = scan.header  # Use original scan's header for timestamp and frame_id
        detected_objects_msg.objects = detected_objects_list
        
        self.detected_objects_pub.publish(detected_objects_msg)
        self.get_logger().debug(f"Published DetectedObjectArray with {len(detected_objects_list)} objects")
        # --- End New ---

    def destroy_node(self):
        self.get_logger().info("Cleaning up ZeroMQ resources...")
        if hasattr(self, 'image_push_socket'):
            self.image_push_socket.close()
        if hasattr(self, 'results_req_socket'):
            self.results_req_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()
        self.get_logger().info("ZeroMQ resources cleaned up.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ReprojectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()