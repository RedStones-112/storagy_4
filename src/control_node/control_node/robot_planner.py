import rclpy as rp
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
import inspect
from control_node.config import ControlNodeConfig as config
import time
import math
import subprocess
import os
# from src.navigation2.nav2_simple_commander.nav2_simple_commander.robot_navigator import BasicNavigator

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult

from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from std_srvs.srv import Empty
from std_msgs.msg import String
# from aris_package.srv import SetSeatNumber
from team4_msgs.srv import PutOnIcecream
from team4_msgs.msg import StoragyStatus

from threading import Thread
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist


class RobotPlanner(Node):
    """
    주변 상황에 따라 계획을 만들고, 
    계획에 따라 nav2에 요청하는 노드
    """
    def __init__(self):
        super().__init__("robot_planner")
        self.nav = BasicNavigator()
        self.create_handles()
        # self.init_pose_estimation()  
        self.reset_values()


        
    def create_handles(self): # sub, pub, client, server 선언 함수
        self.amcl_pose = PoseWithCovarianceStamped()
        # amcl_pose sub
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, config.amcl_topic_name, self.pose_callback, config.stack_msgs_num
        )
        # nav_to_pose action_client 
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, config.pose_topic_name)
        # 아리스에게 이동하는 서비스
        self.goto_icecream_service = self.create_service(
            Empty, config.call_aris_service_name, self.service_icecream
        )
        # 아리스에게 아이스크림을 모두 받았음을 전달받는 서비스
        self.complite_puton = self.create_service(
            PutOnIcecream, config.PutOnIcecream_name, self.service_complite_puton
        )
        # 스토리지의 상태를 뿌리는 펍
        self.state_pub = self.create_publisher(
            String, config.StoragyStatus_name, config.stack_msgs_num
        )

        
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", config.stack_msgs_num)
        
        self.create_timer(1, self.pub_state)

    def pub_state(self): # 상태 퍼블리쉬 하는 함수
        msg = String()
        msg.data = self.storagy_state
        self.state_pub.publish(msg=msg)
    
    def init_pose_estimation(self):
        """로봇이 주위 환경을 스캔하며 위치를 초기화하는 함수"""
        try:
            self.get_logger().info("Setting initial pose...")
            initial_pose = self.create_initial_pose()
            # 초기 위치를 퍼블리시
            self.pose_pub.publish(initial_pose)
            self.get_logger().info("Initial pose set and published.")
            time.sleep(2.0)

            # 초기 위치 추정하기 위한 AMCL 파라미터 변경
            self.declare_amcl_parameters()
            self.update_amcl_parameters_for_estimation()

            # AMCL 전역 초기화 (지도 전체에서 위치를 추정하도록 파티클 초기화)
            self.global_localization_init()
            # 360도 회전을 수행하면서 위치 추정
            self.perform_rotation()

            # AMCL에서 최종적으로 추정한 위치를 가져와 초기 위치로 설정
            if hasattr(self, 'amcl_pose'):
                final_pose = self.create_final_pose()
                self.amcl_pose = final_pose
                self.get_logger().info("Final pose estimated by AMCL and published.")

            self.get_logger().info("AMCL localization and pose estimation completed.")

        except rp.exceptions.ROSInterruptException:
            self.get_logger().error("ROS Interrupt Exception caught. Stopping pose estimation.")
        
        except Exception as e:
            self.get_logger().error(f"Unexpected error during pose estimation: {str(e)}")
        
        finally:
            # 회전 중지 명령을 보내는 코드가 예외 발생 시에도 실행되도록 보장
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("Pose estimation process has been safely terminated.")

    def declare_amcl_parameters(self):
        """AMCL 파라미터를 선언하는 함수"""
        self.declare_parameters(
            namespace='',
            parameters=[(key, value) for key, value in config.AMCL_PARAMS.items()]
        )

    def update_amcl_parameters_for_estimation(self):
        """위치 추정을 위해 AMCL 파라미터를 변경"""
        new_parameters = [
            Parameter('/amcl.alpha1', Parameter.Type.DOUBLE, config.AMCL_ESTIMATION_PARAMS['alpha1']),
            Parameter('/amcl.alpha2', Parameter.Type.DOUBLE, config.AMCL_ESTIMATION_PARAMS['alpha2']),
            Parameter('/amcl.alpha3', Parameter.Type.DOUBLE, config.AMCL_ESTIMATION_PARAMS['alpha3']),
            Parameter('/amcl.alpha4', Parameter.Type.DOUBLE, config.AMCL_ESTIMATION_PARAMS['alpha4']),
            Parameter('/amcl.do_beamskip', Parameter.Type.BOOL, config.AMCL_ESTIMATION_PARAMS['do_beamskip']),
            Parameter('/amcl.max_particles', Parameter.Type.INTEGER, config.AMCL_ESTIMATION_PARAMS['max_particles']),
            Parameter('/amcl.min_particles', Parameter.Type.INTEGER, config.AMCL_ESTIMATION_PARAMS['min_particles']),
        ]
        
        self.get_logger().info("Updating AMCL parameters for pose estimation...")
        try:
            self.set_parameters(new_parameters)
            self.get_logger().info("AMCL parameters updated successfully.")
        
        except Exception as e:
            self.get_logger().error(f"Failed to update AMCL parameters: {str(e)}")


    def restore_default_amcl_parameters(self):
        """위치 추정 후 AMCL 파라미터를 원래 상태로 복원"""
        original_parameters = [
            Parameter('/amcl.alpha1', Parameter.Type.DOUBLE, config.AMCL_DEFAULT_PARAMS['alpha1']),
            Parameter('/amcl.alpha2', Parameter.Type.DOUBLE, config.AMCL_DEFAULT_PARAMS['alpha2']),
            Parameter('/amcl.alpha3', Parameter.Type.DOUBLE, config.AMCL_DEFAULT_PARAMS['alpha3']),
            Parameter('/amcl.alpha4', Parameter.Type.DOUBLE, config.AMCL_DEFAULT_PARAMS['alpha4']),
            Parameter('/amcl.do_beamskip', Parameter.Type.BOOL, config.AMCL_DEFAULT_PARAMS['do_beamskip']),
            Parameter('/amcl.max_particles', Parameter.Type.INTEGER, config.AMCL_DEFAULT_PARAMS['max_particles']),
            Parameter('/amcl.min_particles', Parameter.Type.INTEGER, config.AMCL_DEFAULT_PARAMS['min_particles']),
        ]
        
        self.get_logger().info("Restoring AMCL parameters to defaults...")
        try:
            self.set_parameters(original_parameters)
            self.get_logger().info("AMCL parameters restored successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to restore AMCL parameters: {str(e)}")


    def create_initial_pose(self):
        """초기 위치 설정 함수 맵의 중앙에 위치 설정"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"

        # 초기 위치를 설정 (필요에 따라 값을 조정)
        initial_pose.pose.pose.position.x = -0.52
        initial_pose.pose.pose.position.y = 1.35
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0

        # 초기 위치의 공분산 설정 (36개의 float 값)
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]
        return initial_pose

    def global_localization_init(self):
        """AMCL 전역 초기화 함수"""
        self.get_logger().info("Initializing global localization for AMCL...")
        self.global_localization_service = self.create_client(Empty, '/reinitialize_global_localization')
        
        while not self.global_localization_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /reinitialize_global_localization service...")
        
        global_localization_request = Empty.Request()
        self.global_localization_service.call_async(global_localization_request)
        self.get_logger().info("Global localization initialized.")

    def perform_rotation(self):
        """로봇의 360도 회전 수행 함수"""
        twist_msg = Twist()
        twist_msg.angular.z = 0.2  # 회전 속도 설정

        # 목표 회전 각도 = 2π rad
        target_rotation = 2.5 * math.pi  # 360도

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

    def create_final_pose(self):
        """AMCL로부터 최종 위치를 가져와 초기 위치로 설정하는 함수"""
        final_pose = PoseWithCovarianceStamped()
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.header.frame_id = "map"

        # AMCL이 추정한 최종 위치 및 자세를 설정
        final_pose.pose.pose.position.x = self.amcl_pose.pose.pose.position.x
        final_pose.pose.pose.position.y = self.amcl_pose.pose.pose.position.y
        final_pose.pose.pose.orientation = self.amcl_pose.pose.pose.orientation

        # 위치의 공분산 설정
        final_pose.pose.covariance = self.amcl_pose.pose.covariance
        return final_pose

    def reset_values(self):  # 수치값들 초기화 해주는 함수
        self.task_list = []
        self.goal_distance = 0
        self.backup_distance = config.default_backup_distance
        self.distance_remaining = 0
        self.old_distance_remaining = 0
        self.go_to_pose_is_activate = False
        # 다시 초기 위치 추정하기 위해서 필요했던 AMCL 파라미터 값 기존으로 복원    
        # self.restore_default_amcl_parameters()

        # # 현재 amcl_pose 값을 안전하게 복사하여 인스턴스 변수로 저장
        # self.current_amcl_pose = PoseWithCovarianceStamped()
        # self.current_amcl_pose.pose = self.amcl_pose.pose

        self.storagy_state = config.robot_state_wait_task
        self.location_name = config.base
        # self.location_name = "initial_position"
        

    def start_run_thread(self): # run 함수 thread 시작하는 함수
        try:
            self.run_deamon = True
            self.run_thread = Thread(target=self.run)
            self.run_thread.start()
        except:
            print("control_node can't activate run_thread")


    def nav2_send_goal(self, x, y, z=0.0, yaw=None) -> bool:  # 해당 위치로 goal을 보내주는 함수 
        try:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            if yaw != None:
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
            
            self.nav_start_time = time.time()
            self.nav_current_time = time.time()
            self.go_to_pose_is_activate = True

            send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self.nav2_action_feedback_callback)
            send_goal_future.add_done_callback(self.goto_pose_done_callback)
            self.wait_goto_pose()
            return True

        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            self.storagy_state = config.robot_state_wait_task
            time.sleep(config.wait_error)
            return False
        

    def wait_goto_pose(self): # goto_pose 액션이 실행중이라면 추가로 액션을 요청하지 않도록 대기하게 하는 함수
        while self.go_to_pose_is_activate:
            self.nav_current_time = time.time()
            if self.old_distance_remaining != self.distance_remaining:
                self.nav_start_time = time.time()

            # if self.nav_current_time - self.nav_start_time >= config.nav2_wait_time:
            #     self.back_up(self.backup_distance)
            #     self.nav_start_time = time.time()
    
        
    def goto_pose_done_callback(self, future): # goto_pose 액션 콜백
        try:
            result_future = future.result().get_result_async()
            result_future.add_done_callback(self.goto_pose_result_callback)
        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            return 
        
    def goto_pose_result_callback(self, future): # goto_pose 액션이 끝나면 상태값 변환해주는 콜백함수
        try:
            result = future.result().result
            self.go_to_pose_is_activate = False
        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            self.go_to_pose_is_activate = False
            return 

    def nav2_action_feedback_callback(self, msg): # goto_pose 액션 피드백 콜백
        try:
            self.distance_remaining = msg.feedback.distance_remaining
            self.old_distance_remaining = self.distance_remaining
        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            return 
        
        
    def back_up(self, distance = 0): # 후진함수
        try:
            if distance == 0:
                return
            print(f"back_up {distance}")
            twist_msg = Twist()
            twist_msg.linear.x = config.back_up_speed  # 속도 설정
            time_to_move = distance / abs(config.back_up_speed)
            start_time = self.get_clock().now().seconds_nanoseconds()[0]

            while (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < time_to_move:
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.01)

            twist_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(twist_msg)
        except:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: unable input")
            return


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


    def go_to_station_now(self):  # 대기장소로 이동하는 행동을 계획의 최우선으로 추가해주는 함수
        self.task_list.insert(0, config.base)


    def set_state(self):  # 가장 우선도 높은 작업을 현재 상태로 지정해주는 함수
        try:
            self.storagy_state = self.task_list.pop(0)
        except:
            print("task_list is empty")

    def append_task_tables_patrol(self):
        for i in config.robot_state_goto_tables:
            self.task_list.append(i)
        self.task_list.append(config.base)


    def check_able_table_name(self):  # 현재 들어온 작업이 등록된 테이블 중에 있는지 확인하고 가능하면 출발시키는 함수
        try:
            for i, name in enumerate(config.robot_state_goto_tables):
                if name == self.task_list[0]:
                    self.back_up(distance=self.backup_distance)
                    self.backup_distance = config.default_backup_distance
                    self.set_state()
                    self.nav2_send_goal(config.goal_dict[config.tables[i]][config.str_x],
                                        config.goal_dict[config.tables[i]][config.str_y])
                    self.location_name = config.tables[i]
                    time.sleep(config.wait_time)  # 5초 대기 후 대기모드로 전환
                    self.storagy_state = config.robot_state_wait_task
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


    def run(self):  # 테스크 리스트를 확인하며 테스크를 수행하는 함수
        start_time = time.time()
        current_time = start_time
        while self.run_deamon:
            try:
                if current_time - start_time >= config.patrol_time:  # 지정된 시간동안 어떤 작업도 수행하지 않으면 순찰
                    # self.go_to_station_now()
                    self.append_task_tables_patrol()
                    self.storagy_state = config.base
                    start_time = time.time()
                    continue

                if not (self.storagy_state == config.robot_state_wait_task or self.storagy_state == config.base):  # 대기상태가 아니면 명령수행을 하지않음
                    print(f"state: {self.storagy_state}")
                    start_time = time.time()
                    time.sleep(config.wait_error)
                    continue

                if self.task_list == [] and self.storagy_state == config.robot_state_wait_task:  # 테스크 리스트 비어있고 로봇 상태가 대기모드이면 카운트
                    current_time = time.time()
                    print(f"waiting new task ... \nwait_time: {round(current_time - start_time, 2)}sec")
                    # if self.location_name != config.base:
                    #     current_time = time.time()
                    #     print(f"waiting new task ... \nwait_time: {round(current_time - start_time, 2)}sec")
                    # else:
                    #     print("waiting base")
                    time.sleep(config.wait_error)
                    continue

                if self.task_list[0] == config.robot_state_goto_icecream_robot:  # 테스크별 실행
                    self.back_up(distance=self.backup_distance)
                    self.set_state()
                    print(self.storagy_state)
                    self.nav2_send_goal(config.goal_dict[config.before_icecream_robot][config.str_x],
                                        config.goal_dict[config.before_icecream_robot][config.str_y], yaw=math.pi/2)
                    self.nav2_send_goal(config.goal_dict[config.icecream_robot_name][config.str_x],
                                        config.goal_dict[config.icecream_robot_name][config.str_y], yaw=math.pi/2)
                    self.backup_distance = config.back_up_distance
                    # ArUco 마커 팔로워 실행
                    # script_path = os.path.join(os.path.dirname(__file__), 'aruco_follower.py')
                    # subprocess.Popen(["python3", script_path])

                    self.location_name = config.icecream_robot_name
                    self.storagy_state = config.robot_state_wait_icecream_robot
                                        
                    # aris service call 들어갈 위치
                    start_time = time.time()

                elif self.task_list[0] == config.base:
                    print("go to base")
                    self.back_up(distance=self.backup_distance)
                    self.set_state()
                    self.nav2_send_goal(config.goal_dict[config.base][config.str_x],
                                        config.goal_dict[config.base][config.str_y])
                    self.location_name = config.base
                    self.storagy_state = config.robot_state_wait_task
                    start_time = time.time()

                elif self.task_list[0] in config.robot_state_goto_tables:
                    self.check_able_table_name()
                    if self.task_list == []:
                        self.task_list.append(config.base)
                    start_time = time.time()

                else:
                    print(f"정의되지 않은 작업 : {self.task_list[0]}")

            except Exception as e:
                print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
                self.storagy_state = config.robot_state_wait_task


def main(args=None):
    rp.init(args=args)
    planner = RobotPlanner()
    planner.start_run_thread()

    try:
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
