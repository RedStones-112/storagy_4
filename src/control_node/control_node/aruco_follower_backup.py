import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
import math

class ArucoMarkerFollower(Node):
    def __init__(self):
        super().__init__('aruco_marker_follower')

        # OpenCV 및 ArUco 관련 초기화
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Depth 이미지 관련 변수 초기화
        self.latest_depth_image = None

        # ROS2 구독자 및 퍼블리셔 설정
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 목표 위치에 도달했는지 여부를 추적하는 변수 및 카운터 초기화
        self.target_reached_count = 0  # 목표 도달 횟수를 추적

        # 카메라 내부 파라미터 초기화
        self.intrinsic_matrix = None
        self.distortion_coefficients = None

        # TF2 버퍼 및 리스너 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg):
        """카메라 내부 파라미터를 설정하는 콜백 함수"""
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coefficients = np.array(msg.d)
        
    def depth_callback(self, msg):
        """깊이 이미지를 처리하고 최신 이미지로 업데이트하는 콜백 함수"""
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.latest_depth_image = np.array(depth_image, dtype=np.float32)

    def rgb_callback(self, msg):
        """RGB 이미지에서 ArUco 마커를 감지하고, 위치를 추정하여 제어하는 콜백 함수"""
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ArUco 마커 감지
        corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            # 감지된 마커에 대해 위치 조정 수행
            self.adjust_to_marker(corners, ids, rgb_image)
            cv2.aruco.drawDetectedMarkers(rgb_image, corners, ids)  # 마커를 이미지에 표시

        # 실시간으로 화면을 출력
        cv2.imshow("RGB Image", rgb_image)
        cv2.waitKey(1)  # 프레임을 갱신하기 위해 1ms 대기

    def adjust_to_marker(self, corners, ids, rgb_image):
        """감지된 ArUco 마커의 중심 좌표와 깊이 정보를 사용하여 위치를 추정하고 제어하는 함수"""
        marker_corners = corners[0].reshape((4, 2))
        marker_id = ids[0][0]
        marker_center = marker_corners.mean(axis=0)

        # 깊이 정보가 존재하는지 확인
        if self.latest_depth_image is not None:
            # 마커 중심의 깊이 값을 추출
            depth_value = self.latest_depth_image[int(marker_center[1]), int(marker_center[0])]

            if self.intrinsic_matrix is not None:
                # 카메라 파라미터를 사용하여 X, Y, Z 좌표를 계산
                x = (marker_center[0] - self.intrinsic_matrix[0, 2]) * depth_value / self.intrinsic_matrix[0, 0]
                y = (marker_center[1] - self.intrinsic_matrix[1, 2]) * depth_value / self.intrinsic_matrix[1, 1]
                z = depth_value

                # 마커의 회전 각도 계산 (코너 점들을 이용)
                vector1 = marker_corners[0] - marker_corners[1]
                vector2 = marker_corners[2] - marker_corners[1]
                angle = math.atan2(vector2[1], vector2[0])

                # 이미지에 마커 ID와 좌표 및 각도 정보를 표시
                text = f"ID: {marker_id}, X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, Angle: {math.degrees(angle):.2f}"
                cv2.putText(rgb_image, text, (int(marker_center[0]), int(marker_center[1])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                # 위치 정보 로그 출력
                self.get_logger().info(f'Marker ID: {marker_id}, X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, Angle: {math.degrees(angle):.2f}')

                # 마커에 대한 제어 명령 수행
                self.handle_marker_by_id(marker_id, x, y, z, angle)

    def handle_marker_by_id(self, marker_id, x, y, z, angle):
        """마커 ID에 따라 로봇의 위치 및 회전을 조정하는 함수"""
        cmd = Twist()

        # 각 marker_id에 대한 목표 X, Z 위치 및 허용 오차 설정
        target_positions = {
            1: {'x': -34.00, 'z': 728.00, 'angle': 0.0}, 
            # 추가적인 marker_id의 목표를 여기에 추가
        }

        # 2mm 오차 범위 및 각도 허용 오차 (예: 3도)
        tolerance = 2.0
        angle_tolerance = math.radians(3.0)

        # 목표 위치 및 각도 가져오기
        target_x = target_positions.get(marker_id, {}).get('x')
        target_z = target_positions.get(marker_id, {}).get('z')
        target_angle = target_positions.get(marker_id, {}).get('angle')

        if target_x is not None and target_z is not None and target_angle is not None:
            # X, Z 위치가 목표 위치에 도달했는지 확인
            if abs(x - target_x) <= tolerance and abs(z - target_z) <= tolerance:
                # 목표 위치에 도달한 후 각도 제어 수행
                if abs(angle - target_angle) > angle_tolerance:
                    # 목표 각도에 도달하지 않은 경우 각도 조정
                    self.get_logger().info(f"Marker ID {marker_id}: Target position reached, adjusting angle: X={x:.2f}, Z={z:.2f}, Angle={math.degrees(angle):.2f}")
                    if angle < target_angle:
                        cmd.angular.z = 0.02  # 시계 방향 회전
                    else:
                        cmd.angular.z = -0.02  # 반시계 방향 회전

                    cmd.linear.x = 0.0  # 위치 조정은 멈추고 각도만 조정

                else:
                    # 목표 위치와 각도 모두에 도달하면 로봇의 움직임 중지 및 상태 갱신
                    self.get_logger().info(f"Marker ID {marker_id}: Target position and angle reached: X={x:.2f}, Z={z:.2f}, Angle={math.degrees(angle):.2f}")
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                    self.target_reached_count += 1  # 목표 도달 횟수 증가

                    # 목표에 10번 이상 도달하면 프로그램 종료
                    if self.target_reached_count >= 10:
                        self.get_logger().info("Target reached 10 times. Shutting down.")
                        cv2.destroyAllWindows()
                        rclpy.shutdown()
                        return

            else:
                # 목표 위치에 도달하지 않았을 때, 목표에 따라 전/후진 및 회전 제어
                if z < target_z:
                    cmd.linear.x = -0.02  # 전진
                else:
                    cmd.linear.x = 0.02  # 후진

                if x < target_x:
                    cmd.angular.z = 0.02  # 시계 방향 회전
                else:
                    cmd.angular.z = -0.02  # 반시계 방향 회전

            # 제어 명령 퍼블리시
            self.cmd_vel_pub.publish(cmd)
        else:
            # target_x 또는 target_z 또는 target_angle 값이 None인 경우, 로봇을 중지합니다.
            self.get_logger().warn(f"Marker ID {marker_id} has no target position or angle set.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

    def quaternion_to_euler(self, q):
        """쿼터니언을 오일러 각도로 변환하는 함수"""
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각도를 쿼터니언으로 변환하는 함수"""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

def main(args=None):
    """메인 함수: 노드 초기화 및 실행"""
    rclpy.init(args=args)
    follower = ArucoMarkerFollower()

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 노드를 종료
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
