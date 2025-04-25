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
        
        self.LABELS_FILE = "/project/linorobot2_ws/src/linorobot2/linorobot2_vision/linorobot2_vision/coco.names"
        self.CONFIG_FILE = "/project/linorobot2_ws/src/linorobot2/linorobot2_vision/linorobot2_vision/yolov3.cfg"
        self.WEIGHTS_FILE = "/project/linorobot2_ws/src/linorobot2/linorobot2_vision/linorobot2_vision/yolov3.weights"
        self.CONFIDENCE_THRESHOLD = 0.3

        self.LABELS = open(self.LABELS_FILE).read().strip().split("\n")
        np.random.seed(4)
        self.COLORS = np.random.randint(0, 255, size = (len(self.LABELS), 3), dtype = "uint8")
        self.net = cv2.dnn.readNetFromDarknet(self.CONFIG_FILE, self.WEIGHTS_FILE)


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

    def drawBoxes (self, image, layerOutputs, H, W):
        boxes = []
        confidences = []
        classIDs = []

        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                if confidence > self.CONFIDENCE_THRESHOLD:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.CONFIDENCE_THRESHOLD, self.CONFIDENCE_THRESHOLD)

        # Ensure at least one detection exists
        if len(idxs) > 0:
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                color = [int(c) for c in self.COLORS[classIDs[i]]]

                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(self.LABELS[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def callback(self, scan, image):
        self.get_logger().debug("image timestamp: %d ns" % image.header.stamp.nanosec)
        self.get_logger().debug("scan timestamp: %d ns" % scan.header.stamp.nanosec)
        diff = abs(image.header.stamp.nanosec - scan.header.stamp.nanosec)
        self.get_logger().debug("diff: %d ns" % diff)

        img = self.bridge.compressed_imgmsg_to_cv2(image)
        cloud = self.lp.projectLaser(scan)
        points = pc2.read_points(cloud)
        objPoints = np.array(list(map(self.extract, points)))

        # Beispiel für Datenreduktion: Filtere Laserpunkte basierend auf Entfernung
        max_distance = 5.0  # Maximal zulässige Entfernung in Metern
        objPoints = objPoints[np.linalg.norm(objPoints, axis=1) <= max_distance]
        
        # Projiziere 3D-Punkte in 2D-Bildkoordinaten
        if self.lens == 'pinhole':
            img_points, _ = cv2.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        elif self.lens == 'fisheye':
            objPoints_reshaped = np.reshape(objPoints, (1, objPoints.shape[0], objPoints.shape[1]))
            img_points, _ = cv2.fisheye.projectPoints(objPoints_reshaped, self.rvec, self.tvec, self.K, self.D)
        img_points = np.squeeze(img_points)


        # Definiere Bildgrenzen
        img_height, img_width = img.shape[:2]
 
        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]

        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (416, 416), swapRB = True, crop = False)
        self.net.setInput(blob)
        layerOutputs = self.net.forward(ln)
        self.drawBoxes(img, layerOutputs, img_height, img_width)

        # Filtere Punkte, die außerhalb des Bildes liegen
        if len(img_points) > 0:
            valid_indices = np.where((img_points[:, 0] >= 0) & (img_points[:, 0] < img_width) &
                                     (img_points[:, 1] >= 0) & (img_points[:, 1] < img_height))[0]
            objPoints_filtered = objPoints[valid_indices]
            img_points_filtered = img_points[valid_indices]
        else:
            objPoints_filtered = np.array([])
            img_points_filtered = np.array([])

        # Z-Filterung nur auf gefilterte Punkte anwenden
        Z = self.get_z(self.q, objPoints_filtered, self.K)
        objPoints_filtered = objPoints_filtered[Z > 0]
        img_points_filtered = img_points_filtered[Z > 0]

        # Beispiel für Vektorisierung (ggf. ineffizient, je nach OpenCV-Version und Hardware)
        if len(img_points_filtered) > 0:
            img_points_filtered = np.round(img_points_filtered).astype(np.int32)
            for point in img_points_filtered:
                cv2.circle(img, tuple(point), self.laser_point_radius, (0, 255, 0), 1)
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