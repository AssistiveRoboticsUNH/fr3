"""Teleoperating robot arm with a SpaceMouse to collect demonstration data"""

import argparse
import json
import os
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
# from rpl_vision_utils.networking.camera_redis_interface import \
#     CameraRedisSubInterface
from oculus_reader.reader import OculusReader

from deoxys_vision.networking.camera_redis_interface import CameraRedisSubInterface
from deoxys_vision.camera.rs_interface import RSInterface
from deoxys_vision.utils.img_utils import preprocess_color, preprocess_depth
from deoxys_vision.utils.camera_utils import assert_camera_ref_convention, get_camera_info

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.config_utils import robot_config_parse_args
from deoxys.utils.input_utils import input2action
# from deoxys.k4a_interface import K4aInterface
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger
import cv2 
from threading import Thread
from 

logger = get_deoxys_example_logger()


import beepy as beep
beep_start = lambda : beep.beep('coin')
beep_end= lambda : beep.beep('ready')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--folder", type=Path, default="/home/franka_deoxys/data_franka/imgsd_demo/")
    parser.add_argument("--controller-type", type=str, default="OSC_POSE")
    parser.add_argument(
        "--controller-cfg", type=str, default="osc-position-controller.yml"
    )
    
    parser.add_argument("--use-depth", action="store_true", 
                        help="increase output verbosity") 

    parser.add_argument(
        "--horizon",
        type=int,
        default=1000,
    )
    parser.add_argument(
        "--vendor_id",
        type=int,
        default=9583,
    )
    parser.add_argument(
        "--product_id",
        type=int,
        default=50734,
    )
    # robot_config_parse_args(parser)
    return parser.parse_args()

def input_to_action(state, last_state):
    state_pose_data, state_input_data = state
    last_pose_data, last_input_data = last_state

    # Skip if either part is empty
    if not state_pose_data or not last_pose_data:
        return None

    pose1 = last_pose_data['r']
    pose2 = state_pose_data['r']
    delta_pose = np.linalg.inv(pose1) @ pose2
    translation = delta_pose[:3, 3]
    rotation_matrix = delta_pose[:3, :3]
    rotation = R.from_matrix(rotation_matrix).as_euler('xyz')

    # joystick_delta = np.array(state_input_data['rightJS']) - np.array(last_input_data['rightJS'])
    # trigger_delta = np.array(state_input_data['rightTrig']) - np.array(last_input_data['rightTrig'])
    
    if(state_input_data['rightTrig'][0] > 0):
        trigger_delta = 1
    else:
        trigger_delta = -1

    if(state_input_data['rightGrip'][0] > 0):
        grip_delta = 1
    else:
        grip_delta = -1

    return {
        "delta_pos": translation,
        "delta_rot": rotation,
        # "joystick": joystick_delta,
        "trigger": [trigger_delta],
        "grip": [grip_delta]
    }, (translation + rotation + [trigger_delta])

def main():
    args = parse_args()

    use_depth = args.use_depth
    horizon = args.horizon 

    print(f'-----------collecting data with {horizon} max steps')

    args.folder.mkdir(parents=True, exist_ok=True)

    experiment_id = 0

    logger.info(f"Saving to {args.folder}")

    # Create a folder that saves the demonstration raw states.
    for path in args.folder.glob("run*"):
        if not path.is_dir():
            continue
        try:
            folder_id = int(str(path).split("run")[-1])
            if folder_id > experiment_id:
                experiment_id = folder_id
        except BaseException:
            pass
    experiment_id += 1
    folder = str(args.folder / f"run{experiment_id}")

    oculus_reader = OculusReader()

    cfg_file = os.path.join(config_root, args.interface_cfg)
    print('cfg_file=', cfg_file)

    # Franka Interface
    robot_interface = FrankaInterface(cfg_file)

    camera_ids = [0, 1]
    # camera_ids = [0]
    cr_interfaces = {}




    for camera_id in camera_ids:
        camera_ref=f"rs_{camera_id}"
        assert_camera_ref_convention(camera_ref)
        camera_info = get_camera_info(camera_ref)
        print('---------****-----------')
        print(camera_info)
        print('--------------------------')
        # camera_info=  {'camera_id': 0, 'camera_type': 'rs', 'camera_name': 'camera_rs_0'}
    # 

        cr_interface = CameraRedisSubInterface(camera_info=camera_info, use_depth=use_depth, redis_host='127.0.0.1')
        cr_interface.start()
        cr_interfaces[camera_id] = cr_interface

    controller_cfg = YamlConfig(
        os.path.join(config_root, args.controller_cfg)
    ).as_easydict()

    # demo_file = h5py.File(demo_file_name, "w")
    controller_type = args.controller_type

    data = {"action": [], "ee_states": [], "joint_states": [], "gripper_states": []}
    for camera_id in camera_ids:
        data[f"camera_{camera_id}"] = [] 
        data[f"camera_{camera_id}_color"] = []
        data[f"camera_{camera_id}_depth"] = []



    i = 0
    start = False

    previous_state_dict = None


    # beep_start()
    time.sleep(2)

    print('------------------------record--------now----------------------')
    tb=Thread(target=beep_start)
    tb.start()
    last_state = None


    # empty_action = {
    #     "delta_pos": [0.0,0.0,0.0],
    #     "delta_rot": [0.0,0.0,0.0],
    #     "trigger": [0.0],
    #     "grip": [0.0]
    # }
    empty_action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    while i < horizon:
        i += 1
        start_time = time.time_ns()
        
        state = oculus_reader.get_transformations_and_buttons()
        action = empty_action
        if last_state is not None:
            input_action_data, input_action = input_to_action(state, last_state)
            if input_action_data is not None:
                if input_action_data['grip'][0] > 0:
                    action = input_action
        last_state = state
        grasp = input_action_data['trigger']

        if action is None:
            break

        # set unused orientation dims to 0
        if controller_type == "OSC_YAW":
            action[3:5] = 0.0
        elif controller_type == "OSC_POSITION":
            action[3:6] = 0.0

        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

        if len(robot_interface._state_buffer) == 0:
            continue
        last_state = robot_interface._state_buffer[-1]
        last_gripper_state = robot_interface._gripper_state_buffer[-1]
        if np.linalg.norm(action[:-1]) < 1e-3 and not start:
            continue

        start = True
        # print(action.shape)
        # Record ee pose,  joints

        data["action"].append(action)

        state_dict = {
            "ee_states": np.array(last_state.O_T_EE),
            "joint_states": np.array(last_state.q),
            "gripper_states": np.array(last_gripper_state.width),
        }

        if previous_state_dict is not None:
            for proprio_key in state_dict.keys():
                proprio_state = state_dict[proprio_key]
                if np.sum(np.abs(proprio_state)) <= 1e-6:
                    proprio_state = previous_state_dict[proprio_key]
                state_dict[proprio_key] = np.copy(proprio_state)
        for proprio_key in state_dict.keys():
            data[proprio_key].append(state_dict[proprio_key])

        previous_state_dict = state_dict
        # data["ee_states"].append(np.array(last_state.O_T_EE))
        # joints = np.array(last_state.q)
        # if np.sum(np.abs(joints)) < 1e-6:
        #     print("Joints missing!!!!")
        # data["joint_states"].append(np.array(last_state.q))
        # data["gripper_states"].append(np.array(last_gripper_state.width))
        # Get img info

        for camera_id in camera_ids:
            img_info = cr_interfaces[camera_id].get_img_info()
            data[f"camera_{camera_id}"].append(img_info)

            imgs = cr_interfaces[camera_id].get_img()
            # print('keys: ', imgs.keys())
            color_img = imgs["color"][..., ::-1]
            color_img = cv2.resize(color_img, None, fx=0.5, fy=0.5)
            data[f"camera_{camera_id}_color"].append(color_img)

            if use_depth:
                depth_img = imgs["depth"]
                depth_img = cv2.resize(depth_img, None, fx=0.5, fy=0.5)
                data[f"camera_{camera_id}_depth"].append(depth_img)
            # cv2.imshow(f'color_img_{camera_id}', color_img)
            # cv2.waitKey(1)

            # color_img = cv2.resize(color_img, (224, 224))  
            # depth_img = cv2.resize(depth_img, (224, 224))

            # scale by half.
            
        

        # TODO: Test if we can directly save img (probably not)
        # img = cr_interface.get_img() 


        end_time = time.time_ns()
        print(f"Time profile: {(end_time - start_time) / 10 ** 9} steps: {i}/{horizon}")

    # cv2.destroyAllWindows()
        
    os.makedirs(folder, exist_ok=True)
    with open(f"{folder}/config.json", "w") as f:
        config_dict = {
            "controller_cfg": dict(controller_cfg),
            "controller_type": controller_type,
        }
        json.dump(config_dict, f)
        np.savez(f"{folder}/testing_demo_action", data=np.array(data["action"]))
        np.savez(f"{folder}/testing_demo_ee_states", data=np.array(data["ee_states"]))
        np.savez(
            f"{folder}/testing_demo_joint_states", data=np.array(data["joint_states"])
        )
        np.savez(
            f"{folder}/testing_demo_gripper_states",
            data=np.array(data["gripper_states"]),
        )

    for camera_id in camera_ids:
        np.savez(
            f"{folder}/testing_demo_camera_{camera_id}_color",
            data=np.array(data[f"camera_{camera_id}_color"]),
        )
        if use_depth:
            np.savez(
                f"{folder}/testing_demo_camera_{camera_id}_depth",
                data=np.array(data[f"camera_{camera_id}_depth"]),
            )


    for camera_id in camera_ids:
        np.savez(
            f"{folder}/testing_demo_camera_{camera_id}",
            data=np.array(data[f"camera_{camera_id}"]),
        )
        cr_interfaces[camera_id].stop()
    robot_interface.close()
    print("Total length of the trajectory: ", len(data["action"]))
    # beep_end()
    tb=Thread(target=beep_end)
    tb.start()
    

if __name__ == "__main__":
    main()
