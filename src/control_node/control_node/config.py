


amcl_topic_name = "/amcl_pose"
pose_topic_name = "navigate_to_pose"

stack_msgs_num = 1


icecream_robot_name = "aris"
before_icecream_robot = "before_aris"
tables  = ["table_1", "table_2", "table_3"]
base = "base"

str_x = "x"
str_y = "y"
str_z = "z"


goal_dict = {tables[0]              : {"x" : 1.6,   "y" : 3.25, "z" : 0.0},
             tables[1]              : {"x" : 1.6,   "y" : 2.5,  "z" : 0.0},
             tables[2]              : {"x" : 1.1,   "y" : 3.25, "z" : 0.0},
             icecream_robot_name    : {"x" : 1.6,   "y" : 0.2,  "z" : 0.0},
             before_icecream_robot  : {"x" : 0.18,  "y" : 0.2,  "z" : 0.0},
             base                   : {"x" : 0.0,   "y" : 0.0,  "z" : 0.0}

}



robot_state_wait_task   = "wait_task"
robot_state_wait_icecream_robot   = f"wait_{icecream_robot_name}"
robot_state_goto_icecream_robot   = f"goto_{icecream_robot_name}"
robot_state_goto_tables = []
for i in tables:
    robot_state_goto_tables.append(f"goto_{i}")

robot_state_wait_client = "wait_client"
robot_state_patrol      = "patrol"
robot_state_wait_task   = "wait_task"
