import rclpy as rp
from rclpy.node import Node
import numpy as np
import inspect
from control_node.config import ControlNodeConfig as config
import time

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult

from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from std_srvs.srv import Empty
# from aris_package.srv import SetSeatNumber
from team4_msgs.srv import PutOnIcecream
from threading import Thread
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

import math


class RobotPlanner(Node):
    """
    주변 상황에 따라 계획을 만들고, 
    계획에 따라 nav2에 요청하는 노드
    """
    def __init__(self):
        super().__init__("robot_planner")
        self.nav = BasicNavigator()

        # self.amcl_pose를 먼저 초기화
        self.amcl_pose = PoseWithCovarianceStamped()

        self.create_handles()
        
        self.init_pose_estimation()  
        self.reset_values()

        
    def create_handles(self): # sub, pub, client, server 선언 함수
        # amcl_pose sub
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, config.amcl_topic_name, self.pose_callback, config.stack_msgs_num)
        # nav_to_pose action_client 
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, config.pose_topic_name)
        # 아리스에게 이동하는 서비스
        self.goto_icecream_service = self.create_service(Empty, "/go_to_icecream", self.service_icecream)
        # 아리스에게 아이스크림을 모두 받았음을 전달받는 서비스
        self.complite_puton = self.create_service(PutOnIcecream, "set_seat_number", self.service_complite_puton)

        # cmd_vel publisher for rotating the robot 
        # 처음 시작했을때 360도 회전해서 자기 위치 추정
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 초기 위치 퍼블리시를 위한 publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    
    def init_pose_estimation(self):
        """로봇이 주위 환경을 스캔하며 위치를 초기화하는 함수"""
        try:
            # 초기 위치를 임의로 설정
            self.get_logger().info("Setting initial pose...")
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.header.frame_id = "map"

            # 초기 위치를 설정 (필요에 따라 값을 조정)
            # 맵 중앙으로 로봇 초기 위치 다시 설정
            initial_pose.pose.pose.position.x = -0.52
            initial_pose.pose.pose.position.y = 1.35
            initial_pose.pose.pose.orientation.z = 0.0
            initial_pose.pose.pose.orientation.w = 1.0  # w를 1.0으로 설정하여 초기 방향을 정렬

            # 초기 위치의 공분산 설정 (36개의 float 값)
            initial_pose.pose.covariance = [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]

            # 초기 위치를 퍼블리시
            self.pose_pub.publish(initial_pose)
            self.get_logger().info("Initial pose set and published.")
            time.sleep(2.0)  # 퍼블리시 후 잠시 대기

            # AMCL 전역 초기화 (지도 전체에서 위치를 추정하도록 파티클 초기화)
            self.get_logger().info("Initializing global localization for AMCL...")
            self.global_localization_service = self.create_client(Empty, '/reinitialize_global_localization')
            
            while not self.global_localization_service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for /reinitialize_global_localization service...")
            
            global_localization_request = Empty.Request()
            self.global_localization_service.call_async(global_localization_request)
            self.get_logger().info("Global localization initialized.")

            # 회전을 위한 Twist 메시지 생성
            twist_msg = Twist()
            twist_msg.angular.z = 0.25  # 회전 속도 설정

            # 목표 회전 각도 = 2π rad
            target_rotation = 2 * math.pi  # 360도

            # 한 번의 루프에서 회전하는 각도
            rotation_per_loop = twist_msg.angular.z * 0.3  # 0.25 rad/s * 0.3 s

            # 필요한 루프 횟수 계산
            num_loops = int(target_rotation / rotation_per_loop)

            # 360도 회전을 수행하면서 위치 추정
            for _ in range(num_loops):
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.3)  # 0.3초마다 메시지 전송 (속도 조정)

            # 회전 종료
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("Initial pose estimation done by rotating.")

            # AMCL에서 최종적으로 추정한 위치를 가져와 초기 위치로 설정
            if hasattr(self, 'amcl_pose'):
                final_pose = PoseWithCovarianceStamped()
                final_pose.header.stamp = self.get_clock().now().to_msg()
                final_pose.header.frame_id = "map"

                # AMCL이 추정한 최종 위치 및 자세를 설정
                final_pose.pose.pose.position.x = self.amcl_pose.pose.pose.position.x
                final_pose.pose.pose.position.y = self.amcl_pose.pose.pose.position.y
                final_pose.pose.pose.orientation = self.amcl_pose.pose.pose.orientation

                # 위치의 공분산 설정
                final_pose.pose.covariance = self.amcl_pose.pose.covariance
                self.get_logger().info("Final pose estimated by AMCL and published.")

                # 최종 추정된 위치를 로봇의 현재 위치로 업데이트
                self.amcl_pose = final_pose

            self.get_logger().info("AMCL localization and pose estimation completed.")

        except rp.exceptions.ROSInterruptException:
            self.get_logger().error("ROS Interrupt Exception caught. Stopping pose estimation.")
        
        except Exception as e:
            self.get_logger().error(f"Unexpected error during pose estimation: {str(e)}")
        
        finally:
            # 회전 중지 명령을 보내는 코드가 예외 발생 시에도 실행되도록 보장
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("Pose estimation process has been safely terminated.")


    def reset_values(self):  # 수치값들 초기화 해주는 함수
        self.task_list = []
        self.goal_distance = 0

        # 현재 amcl_pose 값을 안전하게 복사하여 인스턴스 변수로 저장
        self.current_amcl_pose = PoseWithCovarianceStamped()
        self.current_amcl_pose.pose = self.amcl_pose.pose

        self.storagy_state = config.robot_state_wait_task
        # self.location_name = config.base
        self.location_name = "initial_position"


        
    def start_run_thread(self): # run 함수 thread 시작하는 함수
        try:
            self.run_deamon = True
            self.run_thread = Thread(target=self.run)
            self.run_thread.start()
        except:
            print("control_node can't activate run_thread")


    def nav2_send_goal(self, x, y, z=0.0, yaw=0.0) -> bool:  # 해당 위치로 goal을 보내주는 함수 
        try:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            result = self.euler_to_quaternion(yaw)
            pose.pose.orientation.x = result.pop(0)
            pose.pose.orientation.y = result.pop(0)
            pose.pose.orientation.z = result.pop(0)
            pose.pose.orientation.w = result.pop(0)
            pose.header.frame_id = config.map_name
            pose.header.stamp = self.get_clock().now().to_msg()

            while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                print("'NavigateToPose' action server not available, waiting...")

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            print('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
            send_goal_future = self.nav_to_pose_client.send_goal(goal_msg)

            return True

        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            self.storagy_state = config.robot_state_wait_task
            time.sleep(config.wait_error)
            return False


    def euler_to_quaternion(self, yaw=config.euler_default_val, 
                            pitch=config.euler_default_val, roll=config.euler_default_val) -> list:  # roll pitch yaw를 쿼터니언으로 바꿔주는 함수
        try:
            qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
            qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
            qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
            qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

            return [qx, qy, qz, qw]
        except:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: unable input")
            return


    def distance_callback(self, msg):  # nav에서 목표와의 남은 거리를 받아올 때 사용할 함수
        self.goal_distance = msg.feedback


    def pose_callback(self, msg):  # pose 토픽을 가져와 값을 갱신해주는 함수
        self.amcl_pose = msg


    def service_icecream(self, req, res) -> Empty.Response:  # 아이스크림 주문 서비스 콜이 들어왔을 때 동작하는 함수
        self.task_list.append(config.robot_state_goto_icecream_robot)
        return res


    def service_complite_puton(self, req, res) -> PutOnIcecream.Response:  # aris가 아이스크림을 storagy에 모두 올려 뒀을때 동작하는 함수 테이블 번호를 받아오고 결과를 반환해준다.
        try:
            self.task_list.append(config.robot_state_goto_tables[req.seat_number - 1])
            self.storagy_state = config.robot_state_wait_task
            res.is_okay = True
        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            res.is_okay = False
        finally:
            return res


    def take_off_icecream(self):  # 일정시간 이상 대기장소가 아닌 장소에서 대기시 동작하는 함수 대기장소로 이동한다.
        self.go_to_station_now()
        self.set_state()


    def set_self_potion(self):  # 맵과 유추 위치가 틀린 경우 등에 사용할 자기유추 위치를 변경하는 함수 예정
        pass


    def patrol(self):  # 쓰래기 수거를 순찰을 계획에 추가해주는 함수
        self.task_list.append(config.robot_state_patrol)


    def go_to_station_now(self):  # 대기장소로 이동하는 행동을 계획의 최우선으로 추가해주는 함수
        self.task_list.insert(0, config.base)


    def set_state(self):  # 가장 우선도 높은 작업을 현재 상태로 지정해주는 함수
        try:
            self.storagy_state = self.task_list.pop(0)
        except:
            print("task_list is empty")


    def check_able_table_name(self):  # 현재 들어온 작업이 등록된 테이블 중에 있는지 확인하고 가능하면 출발시키는 함수
        try:
            for i, name in enumerate(config.robot_state_goto_tables):
                if name == self.task_list[0]:
                    self.set_state()
                    self.nav2_send_goal(config.goal_dict[config.tables[i]][config.str_x],
                                        config.goal_dict[config.tables[i]][config.str_y])
                    time.sleep(config.wait_time)  # 5초 대기 후 대기모드로 전환
                    self.storagy_state = config.robot_state_wait_task
                    self.location_name = config.tables[i]
                    break
        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            self.storagy_state = config.robot_state_wait_task

    def quaternion_to_yaw(self, qx, qy, qz, qw) -> float:
        """ 쿼터니언을 yaw(방위각) 각도로 변환하는 함수 """
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    #아리스 받는 위치
    def adjust_ice_cream_pose(self):
        # 설정된 목표 위치 및 자세
        target_pose = {
            "x": 0.01,
            "y": 0.05,
            "z": 0.0,
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        }

        # 오차 허용 범위
        position_tolerance = 0.01  # 1cm
        orientation_tolerance = 0.01  # 약 0.01 라디안 (0.57도)

        while True:
            # 현재 AMCL에서 받은 위치 및 자세
            current_pose = {
                "x": self.amcl_pose.pose.pose.position.x,
                "y": self.amcl_pose.pose.pose.position.y,
                "z": self.amcl_pose.pose.pose.position.z,
                "orientation": {
                    "x": self.amcl_pose.pose.pose.orientation.x,
                    "y": self.amcl_pose.pose.pose.orientation.y,
                    "z": self.amcl_pose.pose.pose.orientation.z,
                    "w": self.amcl_pose.pose.pose.orientation.w
                }
            }

            # 현재 위치와 목표 위치 간의 오차 계산
            position_error = math.sqrt(
                (target_pose["x"] - current_pose["x"])**2 +
                (target_pose["y"] - current_pose["y"])**2 +
                (target_pose["z"] - current_pose["z"])**2
            )

            # 현재 자세와 목표 자세 간의 오차 계산 (쿼터니언을 이용한 yaw 차이)
            target_yaw = self.quaternion_to_yaw(target_pose["orientation"]["x"],
                                                target_pose["orientation"]["y"],
                                                target_pose["orientation"]["z"],
                                                target_pose["orientation"]["w"])

            current_yaw = self.quaternion_to_yaw(current_pose["orientation"]["x"],
                                                current_pose["orientation"]["y"],
                                                current_pose["orientation"]["z"],
                                                current_pose["orientation"]["w"])

            orientation_error = abs(target_yaw - current_yaw)

            # 위치와 자세가 모두 허용 오차 이내이면 루프 종료
            if position_error < position_tolerance and orientation_error < orientation_tolerance:
                self.get_logger().info("Fine adjustment completed: Position and orientation are within tolerance.")
                break

            # 목표 위치와 yaw로 미세 조정
            self.get_logger().info("Adjusting position and orientation to AMCL pose")
            self.nav2_send_goal(target_pose["x"], target_pose["y"], target_pose["z"], target_yaw)
            
            # 조정을 반복하기 전에 잠시 대기
            time.sleep(1.0)

        self.get_logger().info("Fine adjustment done to AMCL pose")



    def run(self):  # 테스크 리스트를 확인하며 테스크를 수행하는 함수
        start_time = time.time()
        current_time = start_time
        while self.run_deamon:
            try:
                if current_time - start_time >= 10 and self.location_name != config.base:  # 10초간 어떤 작업도 수행하지 않으면 복귀
                    self.go_to_station_now()
                    start_time = time.time()
                    continue

                if self.storagy_state != config.robot_state_wait_task:  # 대기상태가 아니면 명령수행을 하지않음
                    print(f"state: {self.storagy_state}")
                    start_time = time.time()
                    time.sleep(config.wait_error)
                    continue

                if self.task_list == []:  # 테스크 리스트 비어있으면 대기
                    if self.location_name != config.base:
                        current_time = time.time()
                        print(f"waiting new task ... \nwait_time: {round(current_time - start_time, 2)}sec")
                    else:
                        print("waiting base")
                    time.sleep(config.wait_error)
                    continue

                if self.task_list[0] == config.robot_state_goto_icecream_robot:  # 테스크별 실행
                    self.set_state()
                    self.nav2_send_goal(config.goal_dict[config.before_icecream_robot][config.str_x],
                                        config.goal_dict[config.before_icecream_robot][config.str_y])
                    self.nav2_send_goal(config.goal_dict[config.icecream_robot_name][config.str_x],
                                        config.goal_dict[config.icecream_robot_name][config.str_y])
                    self.location_name = config.icecream_robot_name
                                        
                    # aris service call 들어갈 위치
                    start_time = time.time()
                elif self.task_list[0] == config.base:
                    print("go to base")
                    self.set_state()
                    self.nav2_send_goal(config.goal_dict[config.base][config.str_x],
                                        config.goal_dict[config.base][config.str_y])
                    self.location_name = config.base
                    self.storagy_state = config.robot_state_wait_task
                    start_time = time.time()
                elif self.task_list[0] in config.robot_state_goto_tables:
                    self.check_able_table_name()
                    start_time = time.time()
                else:
                    print(f"정의되지 않은 작업 : {self.task_list[0]}")

            except Exception as e:
                print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
                self.storagy_state = config.robot_state_wait_task


def main(args=None):
    rp.init(args=args)
    planner = RobotPlanner()

    try:
        # 초기화가 완료된 후에 run_thread를 시작
        planner.start_run_thread()

        # ROS2 노드 스핀 (이벤트 루프 실행)
        rp.spin(planner)

    except KeyboardInterrupt:
        print("KeyboardInterrupt stop")

    except Exception as e:
        print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")

    finally:
        # ROS2 종료 및 노드 파괴
        planner.destroy_node()
        rp.shutdown()


if __name__ == "__main__":
    main()
