import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import math

class ArucoMarkerFollower(Node):
    def __init__(self):
        super().__init__('aruco_marker_follower')

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.latest_depth_image = None  # latest_depth_image 초기화

        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)

        self.intrinsic_matrix = None
        self.distortion_coefficients = None

        # TF2 버퍼와 리스너 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg):
        # 카메라의 내재 매트릭스와 왜곡 계수를 가져옵니다.
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coefficients = np.array(msg.d)
        
    def depth_callback(self, msg):
        # depth 이미지를 변환하고 저장
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_image = np.array(depth_image, dtype=np.float32)

        # 최신 depth 이미지를 저장
        self.latest_depth_image = depth_image

    def rgb_callback(self, msg):
        # RGB 이미지를 처리할 때 호출됩니다.
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ArUco 마커를 감지하고 처리합니다.
        corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            # 감지된 마커의 좌표를 사용하여 미세 조정합니다.
            self.adjust_to_marker(corners, ids, rgb_image)

            # ArUco 마커를 감지하고 그리는 코드
            cv2.aruco.drawDetectedMarkers(rgb_image, corners, ids)
            cv2.imshow("RGB Image", rgb_image)
            cv2.waitKey(1)

    def adjust_to_marker(self, corners, ids, rgb_image):
        # 첫 번째로 감지된 마커의 중심을 가져옵니다.
        marker_corners = corners[0].reshape((4, 2))
        marker_id = ids[0][0]

        # 마커 중심 좌표 계산
        marker_center = marker_corners.mean(axis=0)

        # 마커 중심에 해당하는 depth 값을 가져옵니다.
        depth_image = self.latest_depth_image  # 최신 depth 이미지를 사용
        if depth_image is not None:
            depth_value = depth_image[int(marker_center[1]), int(marker_center[0])]

            # 마커의 실제 위치를 계산합니다 (Z 값은 depth에서 가져오고, X, Y는 화면 좌표에서 계산합니다)
            if self.intrinsic_matrix is not None:
                x = (marker_center[0] - self.intrinsic_matrix[0, 2]) * depth_value / self.intrinsic_matrix[0, 0]
                y = (marker_center[1] - self.intrinsic_matrix[1, 2]) * depth_value / self.intrinsic_matrix[1, 1]
                z = depth_value

                # 마커 ID에 따라 다른 동작을 수행
                self.handle_marker_by_id(marker_id, x, y, z)

    def handle_marker_by_id(self, marker_id, x, y, z):
        # ArUco 마커 ID에 따라 로봇의 동작을 다르게 설정
        cmd = Twist()

        if marker_id == 1:  # ID 1인 경우
            cmd.linear.x = 0.1  # 전진
            cmd.angular.z = 0.1  # 시계 방향 회전
        elif marker_id == 2:  # ID 2인 경우
            cmd.linear.x = -0.1  # 후진
            cmd.angular.z = -0.1  # 반시계 방향 회전
        elif marker_id == 3:  # ID 3인 경우
            cmd.linear.x = 0.2  # 빠르게 전진
            cmd.angular.z = 0.0  # 직진
        else:  # 기타 마커 ID에 대해 기본 동작
            cmd.linear.x = 0.1  # 전진
            cmd.angular.z = 0.05  # 약간의 시계 방향 회전

        # 로봇이 마커에 충분히 가까이 왔다면 멈춤
        if z < 0.005:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def quaternion_to_euler(self, q):
        """ tf2를 사용하여 쿼터니언을 오일러 각도로 변환 """
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """ tf2를 사용하여 오일러 각도를 쿼터니언으로 변환 """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    follower = ArucoMarkerFollower()

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
