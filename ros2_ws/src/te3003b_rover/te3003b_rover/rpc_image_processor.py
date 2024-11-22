#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import grpc
from . import te3002b_pb2
from . import te3002b_pb2_grpc
import google.protobuf.empty_pb2

class RESTImageNode(Node):
    def __init__(self):
        super().__init__('rpc_image_node')
        self._addr="192.168.100.63" # running on WSL; normal address is 127.0.0.1
        self.channel=grpc.insecure_channel(self._addr+':7072')
        self.stub = te3002b_pb2_grpc.TE3002BSimStub(self.channel)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(String, 'aruco_crate_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cv_image = None 
        self.bridge = CvBridge()
        self.result=None
        self.subscription = self.create_subscription(Pose, '/rover_pose', self.listener_callback, qos_profile=qos_profile)
        self.datacmd=te3002b_pb2.CommandData()
        self.pose=[0,0,0,0,0,0]

        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        parameters = cv.aruco.DetectorParameters()
        self.aruco_detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

    def listener_callback(self, msg):
        pos=[msg.position.x,msg.position.y,msg.position.z]
        ang=self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.pose = pos + ang
        self.get_logger().info(f'Received Pose: position {pos} orientation {ang}')

    def timer_callback(self):
        req=google.protobuf.empty_pb2.Empty()
        self.result = self.stub.GetImageFrame(req)
        img_buffer=np.frombuffer(self.result.data, np.uint8)
        img=cv.imdecode(img_buffer, cv.IMREAD_COLOR)
        self.cv_image=img
        if self.cv_image is not None:
            gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
            cv.imwrite(f"/home/thecubicjedi/lidar/cam_frames/frame.png", gray)
            _, ids, _ = self.aruco_detector.detectMarkers(gray)
            if ids is not None:
                self.publisher_.publish(String(data=f"Detected marker with ID {ids[0]}"))
        self.get_logger().info('Publishing image')
        
        self.datacmd.linear.x=self.pose[2]
        self.datacmd.linear.y=self.pose[1]
        self.datacmd.linear.z=self.pose[0]
        self.datacmd.angular.x=self.pose[3]
        self.datacmd.angular.y=self.pose[4]
        self.datacmd.angular.z=self.pose[5]*2
        self.stub.SetCommand(self.datacmd)

    def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z] # in radians

def main(args=None):
    rclpy.init(args=args)
    image_publisher = RESTImageNode()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    image_publisher.c.close()
    rclpy.shutdown()

if __name__ == '__main__':
   main()

