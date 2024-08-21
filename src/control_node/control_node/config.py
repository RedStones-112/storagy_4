

class ControlNodeConfig:
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
    map_name = "map"

    goal_dict = {tables[0]             : {"x" : 0.24,   "y" : 3.06, "z" : 0.0},
                tables[1]              : {"x" : 0.40,   "y" : 1.97,  "z" : 0.0},
                tables[2]              : {"x" : -0.20,   "y" : 3.08, "z" : 0.0},
                icecream_robot_name    : {"x" : -0.024,   "y" : 0.06, "z" : 0.0},
                before_icecream_robot  : {"x" : 0.18,  "y" : 0.2,  "z" : 0.0},
                base                   : {"x" : -1.154768375146601,   "y" : 3.0310363309456543,  "z" : 0.0}

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
    wait_time = 5
    wait_error = 1
    euler_default_val = 0
    location_icecream_robot = [1.125, -0.30]