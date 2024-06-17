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
        self.get_logger().debug("image timestamp: %d ns" % image.header.stamp.nanosec)
        self.get_logger().debug("scan timestamp: %d ns" % scan.header.stamp.nanosec)
        diff = abs(image.header.stamp.nanosec - scan.header.stamp.nanosec)
        self.get_logger().debug("diff: %d ns" % diff)
        
        img = self.bridge.compressed_imgmsg_to_cv2(image)
        cloud = self.lp.projectLaser(scan)
        points = pc2.read_points(cloud)
        objPoints = np.array(list(map(self.extract, points)))
        Z = self.get_z(self.q, objPoints, self.K)
        objPoints = objPoints[Z > 0]
        if self.lens == 'pinhole':
            img_points, _ = cv2.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        elif self.lens == 'fisheye':
            objPoints = np.reshape(objPoints, (1, objPoints.shape[0], objPoints.shape[1]))
            img_points, _ = cv2.fisheye.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        img_points = np.squeeze(img_points)
        for point in img_points:
            if np.isnan(point).any() or np.isinf(point).any():
                continue
            x, y = point[:2]
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            try:
                cv2.circle(img, (int(round(x)), int(round(y))), self.laser_point_radius, (0, 255, 0), 1)
            except Exception as e:
                continue
        self.pub.publish(self.bridge.cv2_to_imgmsg(img))

def main(args=None):
    rclpy.init(args=args)
    node = ReprojectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()