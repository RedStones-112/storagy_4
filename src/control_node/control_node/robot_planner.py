import rclpy as rp
from rclpy.node import Node
import numpy as np
import sys
from control_node import config
import time

# from navigation2.nav2_simple_commander.nav2_simple_commander.robot_navigator import BasicNavigator
# from navigation2.nav2_simple_commander.nav2_simple_commander.robot_navigator import TaskResult

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose

from threading import Thread
from rclpy.duration import Duration
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty


class RobotPlanner(Node):
    """
    주변 상황에 따라 계획을 만들고, 
    계획에 따라 로봇을 이동시키는 노드
    """
    def __init__(self):
        super().__init__("robot_planner")
        self.nav = BasicNavigator()
        self.reset_values()
        self.task_list = []

        # amcl_pose sub
        self.pose_sub           = self.create_subscription(PoseWithCovarianceStamped, config.amcl_topic_name, self.pose_callback, config.stack_msgs_num)
        # nav_to_pose action_client 
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 
        # self.go_to_wait_point   = self.create_service()
        # 
        # self.go_to_aris         = self.create_service()
        self.test_service = self.create_service(Empty, "/test_srv", self.service_icecream)
        
        # self.send_goal(-1.12, 5.2) # test 용도 지워도 상관 x

    def reset_values(self): # 수치값들 초기화 해주는 함수
        self.goal_distance = 0
        self.amcl_pose = PoseWithCovarianceStamped()
        self.storagy_state = config.robot_state_wait_task
        
    def start_run_thread(self):
        self.run_deamon = True
        self.run_thread = Thread(target=self.run)
        self.run_thread.start()

    def nav2_send_goal(self, x, y, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0, w = 0.0) -> bool: # 해당 위치로 목표를 정해주는 함수 
        pose = PoseStamped()
        try:
            pose.pose.position.x    = x
            pose.pose.position.y    = y
            pose.pose.position.z    = z
            pose.pose.orientation.x = roll
            pose.pose.orientation.y = pitch
            pose.pose.orientation.z = yaw
            pose.pose.orientation.w = w

            while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                print("'NavigateToPose' action server not available, waiting...")

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            
            print('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                    str(pose.pose.position.y) + '...')
            send_goal_future = self.nav_to_pose_client.send_goal(goal_msg)
            # rp.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()

            if not self.goal_handle.accepted:
                print('Goal to ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + ' was rejected!')
                return False
            
            return True

        except Exception as e:
            print(f"[{sys._getframe(1).f_code.co_name}]: Error Msg : {e}")
            self.storagy_state = config.robot_state_wait_task
            time.sleep(1)
            return False
        


    def euler_to_quaternion(self, yaw = 0, pitch = 0, roll = 0) -> list : # roll pitch yaw를 쿼터니언으로 바꿔주는 함수
        try:
            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            
            return [qx, qy, qz, qw]
        except:
            print(f"[{sys._getframe(1).f_code.co_name}]: unable input")
            return
    
    def distance_callback(self, msg):
        self.goal_distance = msg.feedback
        
    
    def pose_callback(self, msg): # pose 토픽을 가져와 값을 갱신해주는 함수
        self.amcl_pose = msg
         
    
    def service_icecream(self, req, res): # 아이스크림 주문이 들어왔을 때 동작하는 함수
        self.task_list.append(config.robot_state_goto_aris)
        return res

    def service_complite_puton(self, req, res): # aris가 아이스크림을 모두 올려 뒀을때 동작하는 함수
        return res

    def take_off_icecream(self): # 손님이 아이스크림을 가져갔을때 동작하는 함수
        pass

    def emergency_stop(self, msg): # 긴급 정지
        pass

    def set_self_potion(self): # 맵과 유추 위치가 틀린 경우 등에 사용할 자기유추 위치를 변경하는 함수
        pass

    def patrol(self): # 쓰래기 수거 신호가 올 시 순찰 시작 함수
        pass

    def go_to_station(self):
        pass

    def run(self): # 테스크 리스트를 확인하며 테스크를 수행하는 함수 
        while self.run_deamon:
            try:
                if self.task_list == []:
                    print("waiting new task ...")
                    time.sleep(1)
                    continue
                elif self.task_list[0] == config.robot_state_goto_aris:
                    self.task_list.pop(0)
                    self.storagy_state = config.robot_state_goto_aris
                    self.nav2_send_goal(config.goal_dict[config.icecream_robot_name]["x"],
                                   config.goal_dict[config.icecream_robot_name]["y"])
                    self.storagy_state = config.robot_state_wait_task

            except Exception as e:
                print(f"[{sys._getframe(1).f_code.co_name}]: Error Msg : {e}")
                self.storagy_state = config.robot_state_wait_task


def main(args = None) : 
    rp.init(args=args)
    planner = RobotPlanner()
    planner.start_run_thread()
    try : 
        rp.spin(planner)
        
    except Exception as e:
        print(f"[{sys._getframe(1).f_code.co_name}]: Error Msg : {e}")
        
    finally : 
        rp.shutdown()
        planner.destroy_node()


if __name__ == "__main__" : 
    main()

