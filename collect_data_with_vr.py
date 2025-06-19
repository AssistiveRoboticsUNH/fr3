import argparse
import time
import os

from util_vr import read_vr_action

from deoxys_vision.utils.camera_utils import assert_camera_ref_convention, get_camera_info
from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.log_utils import get_deoxys_example_logger
import matplotlib.pyplot as plt
import numpy as np
from oculus_reader.reader import OculusReader
from deoxys_vision.networking.camera_redis_interface import CameraRedisSubInterface
from threading import Thread
import json

logger = get_deoxys_example_logger()

import beepy as beep
beep_start = lambda : beep.beep('coin')
beep_end= lambda : beep.beep('ready')

###########################################################
# Funtion read read parse and return command line arguemnts
###########################################################
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
        default=50746,   #  50770.,  50746
    )
    # robot_config_parse_args(parser)
    return parser.parse_args()


###########################################################
# Main
############################################################

def main():

    args = parse_args()
    use_depth = args.use_depth
    horizon = args.horizon

    print(f'-----------collecting data with {horizon} max steps')

    args.folder.mkdir(parents=True, exist_ok=True)

    experiment_id = 0

    ###########################################################
    # Create a folder that saves the demonstration raw states.
    ###########################################################
    
    logger.info(f"Saving to {args.folder}")
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

    ###########################################################
    # Setup Franka interface
    ###########################################################

    cfg_file = os.path.join(config_root, args.interface_cfg)
    print('cfg_file=', cfg_file)

    robot_interface = FrankaInterface(cfg_file)
    robot_interface._state_buffer = []

    ###########################################################
    # Setup Cameras
    ###########################################################
    
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

    controller_type = args.controller_type
    controller_cfg = get_default_controller_config(controller_type=controller_type)
    # controller_cfg = YamlConfig("config/osc-pose-controller.yml").as_easydict()

    data = {"action": [], "ee_states": [], "joint_states": [], "gripper_states": []}
    for camera_id in camera_ids:
        data[f"camera_{camera_id}"] = [] 
        data[f"camera_{camera_id}_color"] = []
        data[f"camera_{camera_id}_depth"] = []


    ###########################################################
    # Demo Collection
    ###########################################################
    i = 0

    previous_state_dict = None
    beep_start()
    time.sleep(2)

    print('------------------------record--------now----------------------')
    tb=Thread(target=beep_start)
    tb.start()

    ###########################################################
    # Move Robot
    ###########################################################
    
    oculus_reader = OculusReader()
    last_state = None
    last_trigger = -1.0
    while i < horizon:
        i += 1
        start_time = time.time_ns()
        action, last_state, last_trigger = read_vr_action(oculus_reader, last_state, last_trigger)

        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

        ###########################################################
        # Record states
        ###########################################################

        data["action"].append(action)
        state_dict = {
            "ee_states": np.array(last_state.O_T_EE),
            "joint_states": np.array(last_state.q),
            "gripper_states": np.array(last_trigger),
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

        for camera_id in camera_ids:
            try:
                img_info = cr_interfaces[camera_id].get_img_info()
                data[f"camera_{camera_id}"].append(img_info)

                imgs = cr_interfaces[camera_id].get_img()
                color_img = imgs["color"][..., ::-1]
                color_img = cv2.resize(color_img, None, fx=0.5, fy=0.5)
                data[f"camera_{camera_id}_color"].append(color_img)

                if use_depth:
                    depth_img = imgs["depth"]
                    depth_img = cv2.resize(depth_img, None, fx=0.5, fy=0.5)
                    data[f"camera_{camera_id}_depth"].append(depth_img)
            except Exception as e:
                logger.error(f"Error with camera {camera_id}: {e}")
                continue

        end_time = time.time_ns()
        print(f"Time profile: {(end_time - start_time) / 10 ** 9} steps: {i}/{horizon}")

    ###########################################################
    # Save recorded states to file
    ###########################################################
    
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

    ###########################################################
    # Clean up
    ###########################################################
    
    print("Total length of the trajectory: ", len(data["action"]))
    # beep_end()
    tb=Thread(target=beep_end)
    tb.start()














if __name__ == "__main__":
    main()
