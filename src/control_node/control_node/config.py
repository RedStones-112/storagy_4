

class ControlNodeConfig:
    amcl_topic_name = "/amcl_pose"
    pose_topic_name = "navigate_to_pose"
    stack_msgs_num = 1
    icecream_robot_name = "aris"
    before_icecream_robot = "before_aris"
    tables  = ["table_1", "table_2", "table_3"]
    base = "base"

    # 기존 AMCL 파라미터
    AMCL_PARAMS = {
        '/amcl.alpha1': 0.001,
        '/amcl.alpha2': 0.01,
        '/amcl.alpha3': 0.01,
        '/amcl.alpha4': 0.01,
        '/amcl.alpha5': 0.02,
        '/amcl.base_frame_id': 'base_footprint',
        '/amcl.beam_skip_distance': 0.5,
        '/amcl.beam_skip_error_threshold': 0.9,
        '/amcl.beam_skip_threshold': 0.3,
        '/amcl.do_beamskip': False,
        '/amcl.global_frame_id': 'map',
        '/amcl.lambda_short': 0.1,
        '/amcl.laser_likelihood_max_dist': 3.0,
        '/amcl.laser_max_range': 100.0,
        '/amcl.laser_min_range': -1.0,
        '/amcl.laser_model_type': 'likelihood_field',
        '/amcl.max_beams': 60,
        '/amcl.max_particles': 2000,
        '/amcl.min_particles': 500,
        '/amcl.odom_frame_id': 'odom',
        '/amcl.pf_err': 0.05,
        '/amcl.pf_z': 0.99,
        '/amcl.recovery_alpha_fast': 0.0,
        '/amcl.recovery_alpha_slow': 0.0,
        '/amcl.resample_interval': 1,
        '/amcl.robot_model_type': 'nav2_amcl::DifferentialMotionModel',
        '/amcl.save_pose_rate': 0.5,
        '/amcl.scan_topic': 'scan',
        '/amcl.set_initial_pose': True,
        '/amcl.sigma_hit': 0.1,
        '/amcl.tf_broadcast': True,
        '/amcl.transform_tolerance': 1.0,
        '/amcl.update_min_a': 0.1,
        '/amcl.update_min_d': 0.1,
        '/amcl.use_sim_time': False,
        '/amcl.z_hit': 0.9,
        '/amcl.z_rand': 0.5,
        '/amcl.z_short': 0.05,
    }

    AMCL_ESTIMATION_PARAMS = {
    'alpha1': 0.1,
    'alpha2': 0.1,
    'alpha3': 0.1,
    'alpha4': 0.1,
    'do_beamskip': True,
    'max_particles': 3000,
    'min_particles': 1000
    }

    AMCL_DEFAULT_PARAMS = {
        'alpha1': 0.001,
        'alpha2': 0.01,
        'alpha3': 0.01,
        'alpha4': 0.01,
        'do_beamskip': False,
        'max_particles': 2000,
        'min_particles': 500
        }
    
    str_x = "x"
    str_y = "y"
    str_z = "z"
    map_name = "map"

    goal_dict = {tables[0]             : {"x" : 3.70,   "y" : -0.43, "z" : 0.0},
                tables[1]              : {"x" : 2.70,   "y" : -0.43, "z" : 0.0},
                tables[2]              : {"x" : 3.70,   "y" : -0.05, "z" : 0.0},
                icecream_robot_name    : {"x" : 0.71,   "y" : 1.4,   "z" : 0.0},
                before_icecream_robot  : {"x" : 0.68,   "y" : 0.00,  "z" : 0.0},
                base                   : {"x" : 0.0,    "y" : 0.0,   "z" : 0.0}

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
    patrol_time = 20
    nav2_wait_time = 10
    wait_error = 1
    euler_default_val = 0
    back_up_speed = -0.1
    back_up_distance = 1.0
    default_backup_distance = 0.2
    location_icecream_robot = [1.125, -0.30]

    call_aris_service_name = "/go_to_icecream"
    PutOnIcecream_name = "set_seat_number"
    StoragyStatus_name = "/storagy_state"


