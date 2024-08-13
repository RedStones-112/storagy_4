import rclpy as rp
from rclpy.node import Node
import numpy as np
import inspect
from control_node import config
import time

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
# from navigation2.nav2_simple_commander.nav2_simple_commander.robot_navigator import BasicNavigator
# from navigation2.nav2_simple_commander.nav2_simple_commander.robot_navigator import TaskResult

from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from std_srvs.srv import Empty
from team4_msgs.srv import PutOnIcecream


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

        # amcl_pose sub
        self.pose_sub           = self.create_subscription(PoseWithCovarianceStamped, config.amcl_topic_name, self.pose_callback, config.stack_msgs_num)
        # nav_to_pose action_client 
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, config.pose_topic_name)
        # 서비스 타입 미정
        # self.go_to_wait_point   = self.create_service()
        # 아리스에게 이동하는 서비스
        self.goto_icecream_service = self.create_service(Empty, "/go_to_icecream", self.service_icecream)
        # 아리스에게 아이스크림을 모두 받았음을 전달받는 서비스
        self.complite_puton = self.create_service(PutOnIcecream, "/complite_puton", self.service_complite_puton)

    # def for_test_def(self, req, res):

    #     return res

    def reset_values(self): # 수치값들 초기화 해주는 함수
        self.task_list = []
        self.goal_distance = 0
        self.amcl_pose = PoseWithCovarianceStamped()
        self.storagy_state = config.robot_state_wait_task
        self.location_name = config.base

        
    def start_run_thread(self):
        try:
            self.run_deamon = True
            self.run_thread = Thread(target=self.run)
            self.run_thread.start()
        except :
            print("control_node can't activate run_thread")


    def nav2_send_goal(self, x, y, z = 0.0, yaw = 0.0) -> bool: # 해당 위치로 목표를 정해주는 함수 
        pose = PoseStamped()
        try:
            pose.pose.position.x    = x
            pose.pose.position.y    = y
            pose.pose.position.z    = z

            result = self.euler_to_quaternion(yaw)
            pose.pose.orientation.x = result.pop(0)
            pose.pose.orientation.y = result.pop(0)
            pose.pose.orientation.z = result.pop(0)
            pose.pose.orientation.w = result.pop(0)
            pose.header.frame_id = 'map' 
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
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: unable input")
            return
        
    
    def distance_callback(self, msg) : # nav에서 목표와의 남은 거리를 받아올 때 사용할 함수
        self.goal_distance = msg.feedback
        
    
    def pose_callback(self, msg): # pose 토픽을 가져와 값을 갱신해주는 함수
        self.amcl_pose = msg
         
    
    def service_icecream(self, req, res) -> Empty.Response : # 아이스크림 주문 서비스 콜이 들어왔을 때 동작하는 함수
        self.task_list.append(config.robot_state_goto_icecream_robot)
        return res
    

    def service_complite_puton(self, req, res) -> PutOnIcecream.Response : # aris가 아이스크림을 storagy에 모두 올려 뒀을때 동작하는 함수 테이블 번호를 받아오고 결과를 반환해준다.
        try:
            self.task_list.append(config.robot_state_goto_tables[req.table_num - 1]) 
            self.storagy_state = config.robot_state_wait_task
            res.result = True  
        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            res.result = False  
        finally:
            return res
        

    def take_off_icecream(self): # 일정시간 이상 대기장소가 아닌 장소에서 대기시 동작하는 함수 대기장소로 이동한다.
        self.go_to_station_now() 
        self.set_state() 


    def set_self_potion(self): # 맵과 유추 위치가 틀린 경우 등에 사용할 자기유추 위치를 변경하는 함수
        pass


    def patrol(self): # 쓰래기 수거를 순찰을 계획에 추가해주는 함수
        self.task_list.append(config.robot_state_patrol)
 

    def go_to_station_now(self): # 대기장소로 이동하는 행동을 계획의 최우선으로 추가해주는 함수
        self.task_list.insert(0, config.base)


    def set_state(self): # 가장 우선도 높은 작업을 현재 상태로 지정해주는 함수
        try:
            self.storagy_state = self.task_list.pop(0)
        except :
            print("task_list is empty")


    def check_able_table_name(self): # 현재 들어온 작업이 등록된 테이블 중에 있는지 확인하고 가능하면 출발시키는 함수
        try:
            for i, name in enumerate(config.robot_state_goto_tables):
                if name == self.task_list[0]:
                    self.set_state()
                    self.nav2_send_goal(config.goal_dict[config.tables[i]][config.str_x],
                                        config.goal_dict[config.tables[i]][config.str_y])
                    time.sleep(5) # 5초 대기 후 대기모드로 전환
                    self.storagy_state = config.robot_state_wait_task
                    self.location_name = config.tables[i]
                    break
        except Exception as e:
            print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
            self.storagy_state = config.robot_state_wait_task
        

    def run(self): # 테스크 리스트를 확인하며 테스크를 수행하는 함수 
        start_time = time.time()
        current_time = start_time
        while self.run_deamon:
            try:
                if current_time - start_time >= 10 and self.location_name != config.base: # 10초간 어떤 작업도 수행하지 않으면 복귀
                    self.go_to_station_now()
                    start_time = time.time()
                    continue

                if self.storagy_state != config.robot_state_wait_task: # 대기상태가 아니면 명령수행을 하지않음
                    print(f"state: {self.storagy_state}")
                    start_time = time.time()
                    time.sleep(1)
                    continue

                if self.task_list == []: # 테스크 리스트 비어있으면 대기
                    if self.location_name != config.base:
                        current_time = time.time()
                        print(f"waiting new task ... \nwait_time: {round(current_time - start_time, 2)}sec")
                    else:
                        print("waiting base")
                    time.sleep(1)
                    continue

                if self.task_list[0] == config.robot_state_goto_icecream_robot: # 테스크별 실행
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
                
                
                
                # self.storagy_state = config.robot_state_wait_task # 테스크 끝나면 대기모드로 변경

            except Exception as e:
                print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
                self.storagy_state = config.robot_state_wait_task

    def a_star(self):
        pass


def main(args = None) : 
    rp.init(args=args)
    planner = RobotPlanner()
    planner.start_run_thread()
    try : 
        rp.spin(planner)

    except KeyboardInterrupt:
        print("KeyboardInterrupt stop")
        
    except Exception as e:
        print(f"[{inspect.currentframe().f_back.f_code.co_name}]: Error Msg : {e}")
        
    finally : 
        rp.shutdown()
        planner.destroy_node()


if __name__ == "__main__" : 
    main()

