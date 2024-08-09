


amcl_topic_name = "/amcl_pose"
pose_topic_name = "navigate_to_pose"

stack_msgs_num = 1


icecream_robot_name = "aris"
table_1 = "table_1"
table_2 = "table_2"
table_3 = "table_3"
base = "base"

str_x = "x"
str_y = "y"
str_z = "z"


goal_dict = {table_1                : {"x" : -1.9, "y" : 0.13, "z" : 0.0},
             table_2                : {"x" : -1.3, "y" : 0.13, "z" : 0.0},
             table_3                : {"x" : -1.9, "y" : -0.3, "z" : 0.0},
             icecream_robot_name    : {"x" : 0.65, "y" : 0.5, "z" : 0.0},
             base                   : {"x" : 2.0, "y" : 0.1, "z" : 0.0}
}



robot_state_wait_task   = "wait_task"
robot_state_wait_icecream_robot   = "wait_aris"
robot_state_goto_icecream_robot   = "goto_aris"
robot_state_goto_table_1  = f"goto_{table_1}"
robot_state_goto_table_2  = f"goto_{table_2}"
robot_state_goto_table_3  = f"goto_{table_3}"
robot_state_wait_client = "wait_client"
robot_state_patrol      = "patrol"
robot_state_wait_task   = "wait_task"
