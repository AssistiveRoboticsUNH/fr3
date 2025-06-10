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

from deoxys.experimental.motion_utils import reset_joints_to

logger = get_deoxys_example_logger()


import beepy as beep
beep_start = lambda : beep.beep('coin')
beep_end= lambda : beep.beep('ready')

beep_start_again= lambda : beep.beep('error')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--folder", type=Path, default="/home/franka_deoxys/data_franka/imgsd_demo/")
    parser.add_argument("--controller-type", type=str, default="OSC_POSE")
    parser.add_argument(
        "--controller-cfg", type=str, default="osc-pose-controller.yml"
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
        default=50746.,
    )
    # robot_config_parse_args(parser)
    return parser.parse_args()


def main():
    args = parse_args()

    use_depth = args.use_depth
    horizon = args.horizon 

    print(f'-----------collecting data with {horizon} max steps')

    args.folder.mkdir(parents=True, exist_ok=True)

    experiment_id = 0

    

    

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
    # reset_joint_positions = [0.034,0.109,-0.012,-1.63,0.005,1.776,0.696] # candy 60,80
    reset_joint_positions = [-0.048,0.07,0.007,-1.429,-0.007,1.548,0.72 ]
    reset_joints_to(robot_interface, reset_joint_positions, gripper_open=True)


    # beep_start()
    tb=Thread(target=beep_start_again)
    tb.start()

    while True:
        i = 0
        # start_collecting_data = 0
        # device.__init__(vendor_id=args.vendor_id, product_id=args.product_id)
        device = SpaceMouse(vendor_id=args.vendor_id, product_id=args.product_id)
        device.start_control()
        

        data = {"action": [], "ee_states": [], "joint_states": [], "gripper_states": []}
        for camera_id in camera_ids:
            data[f"camera_{camera_id}"] = [] 
            data[f"camera_{camera_id}_color"] = []
            data[f"camera_{camera_id}_depth"] = []

        while True:
            state = device.get_controller_state()
            start_collecting_data = bool(state["grasp"])
            print(start_collecting_data)
            if start_collecting_data:
                print("Starting Record.....")
                break
            time.sleep(0.1)
                
        time.sleep(0.5)
        ## ==========  Start a trajectory recording =============

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
        
        print('------------------------record--------now----------------------')
        tb=Thread(target=beep_start)
        tb.start()
        
        while i < horizon:
            i += 1
            start_time = time.time_ns()
            action, grasp = input2action(
                device=device,
                controller_type=controller_type,
            )
            if action is None:
                # print("noneeeeeeeeeeeeeeeeeeeee")
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
            # try:
            #     data["action"] == []
            #     data["action"] = action
            # except:
            data["action"].append(action)
                
            # if data["action"] == []:
            #     data["action"] = action

            # else:
            #     data["action"].append(action)

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
                try:
                    img_info = cr_interfaces[camera_id].get_img_info()
                    data[f"camera_{camera_id}"].append(img_info)

                    imgs = cr_interfaces[camera_id].get_img()
                    color_img = imgs["color"][..., ::-1]
                    # color_img = cv2.resize(color_img, None, fx=0.5, fy=0.5)
                    data[f"camera_{camera_id}_color"].append(color_img)

                    if use_depth:
                        depth_img = imgs["depth"]
                        # depth_img = cv2.resize(depth_img, None, fx=0.5, fy=0.5)
                        data[f"camera_{camera_id}_depth"].append(depth_img)
                except Exception as e:
                    logger.error(f"Error with camera {camera_id}: {e}")
                    continue


            # for camera_id in camera_ids:
            #     img_info = cr_interfaces[camera_id].get_img_info()
            #     data[f"camera_{camera_id}"].append(img_info)

            #     imgs = cr_interfaces[camera_id].get_img()
            #     # print('keys: ', imgs.keys())
            #     color_img = imgs["color"][..., ::-1]
            #     color_img = cv2.resize(color_img, None, fx=0.5, fy=0.5)
            #     data[f"camera_{camera_id}_color"].append(color_img)

            #     if use_depth:
            #         depth_img = imgs["depth"]
            #         depth_img = cv2.resize(depth_img, None, fx=0.5, fy=0.5)
            #         data[f"camera_{camera_id}_depth"].append(depth_img)
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

        tb=Thread(target=beep_end)
        tb.start()

        # Reset Robot Joints before closing
        
        # reset_joint_positions = [-0.048,0.07,0.007,-1.429,-0.007,1.548,0.72 ]
        reset_joints_to(robot_interface, reset_joint_positions, gripper_open=True)

        
        print("Total length of the trajectory: ", len(data["action"]))

        tb=Thread(target=beep_start_again)
        tb.start()
        
        device.stop()
        device.device.close()
        # del device

    robot_interface.close()
    
    
    

if __name__ == "__main__":
    main()
