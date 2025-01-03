import argparse
import os
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
# from deoxys.camera_redis_interface import CameraRedisSubInterface
from deoxys.franka_interface import FrankaInterface 
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse


import numpy as np
from PIL import Image
 
from matplotlib import pyplot as plt
 
import scipy.io as sio
 
import numpy as np
import cv2 
import pyrealsense2 as rs
from threading import Thread
import time

import sys 
sys.path.append("/home/franka_deoxys/deoxys_control/deoxys/examples/")
from camera_utils import CVCamera, MyRealSense



import beepy as beep
beep_start = lambda : beep.beep('coin')
beep_end= lambda : beep.beep('ready')


# python3 examples/demo_collection/data_collection2_imgs.py --controller-type OSC_POSITION




def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    # parser.add_argument(
    #     "--controller-cfg", type=str, default="osc-position-controller.yml"
    # )
    parser.add_argument("--folder", type=Path, default="/home/franka_deoxys/data_franka/imgs_demo/")


    parser.add_argument("--controller-type", type=str, default="OSC_POSE")
    parser.add_argument(
        "--controller-cfg", type=str, default="osc-position-controller.yml"
    )


    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    args.folder.mkdir(parents=True, exist_ok=True)

    experiment_id = 0
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
    os.makedirs(folder, exist_ok=True)

    device = SpaceMouse(vendor_id=9583, product_id=50734)
    device.start_control()

    # robot_interface = FrankaInterface(config_root + f"/{args.interface_cfg}")
    #   /home/franka_deoxys/deoxys_control/deoxys/deoxys/../config/alice.yml
    # /home/franka_deoxys/deoxys_control/deoxys/deoxys/../config/charmander.yml
    config_file = os.path.join(config_root, args.interface_cfg)
    print('config_file: ', config_file)

    robot_interface = FrankaInterface(config_file)

    # Initialize camera interfaces. The example uses two cameras. You
    # need to specify camera id in camera_node script from rpl_vision_utils
    camera_ids = [0, 1]
    cr_interfaces = {}
    # for camera_id in camera_ids:
    #     cr_interface = CameraRedisSubInterface(camera_id=camera_id)
    #     cr_interface.start()
    #     cr_interfaces[camera_id] = cr_interface


    cam_wrist=MyRealSense()
    cam_front=CVCamera(0)


    tb1=Thread(target=cam_wrist.run)
    tb1.start()
    tb2=Thread(target=cam_front.run)
    tb2.start()


    cr_interfaces[0]=cam_wrist
    cr_interfaces[1]=cam_front
 

    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()
    # controller_type = "OSC_POSITION"
    # controller_type = "OSC_POSE"
    controller_type = args.controller_type

    data = {
        "action": [],
        "proprio_ee": [],
        "proprio_joints": [],
        "proprio_gripper_state": [],
    }

    for camera_id in camera_ids:
        data[f"camera_{camera_id}"] = []
    i = 0
    start = False


    tb=Thread(target=beep_start)
    tb.start()
    print('-------------------------------------------')
    print('-----------Main Loop Started---------------')
    print('-------------------------------------------')


    


    while i < 2000:
        i += 1
        start_time = time.time_ns()
        action, grasp = input2action(
            device=device,
            controller_type=controller_type,
        )
        if action is None:
            break
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

        if len(robot_interface._state_buffer) == 0:
            continue

        last_state = robot_interface._state_buffer[-1]
        last_gripper_state = robot_interface._gripper_state_buffer[-1]
        # if np.linalg.norm(action[:-1]) < 1e-3 and not start:
        #     continue

        start = True
        print(action)
        # Record ee pose,  joints

        data["action"].append(action)
        data["proprio_ee"].append(np.array(last_state.O_T_EE))
        data["proprio_joints"].append(np.array(last_state.q))
        data["proprio_gripper_state"].append(np.array(last_gripper_state.width))
        # Get img info

        for camera_id in camera_ids:
            # img_info = cr_interfaces[camera_id].get_img_info()
            # data[f"camera_{camera_id}"].append(img_info)
            
            # imagei=cr_interfaces[camera_id].get_current_frame(scale=0.5)
            imagei=cr_interfaces[camera_id].current_frame
            imagei = cv2.resize(imagei, (128, 128))  
            data[f"camera_{camera_id}"].append(imagei)


        # TODO: Test if we can directly save img (probably not)
        # img = cr_interface.get_img()

        end_time = time.time_ns()
        print(f"Time profile: {(end_time - start_time) / 10 ** 9}")


    np.savez(f"{folder}/testing_demo_action", data=np.array(data["action"]))
    np.savez(f"{folder}/testing_demo_proprio_ee", data=np.array(data["proprio_ee"]))
    np.savez(
        f"{folder}/testing_demo_proprio_joints", data=np.array(data["proprio_joints"])
    )
    np.savez(
        f"{folder}/testing_demo_proprio_gripper_state",
        data=np.array(data["proprio_gripper_state"]),
    )

    for camera_id in camera_ids:
        np.savez(
            f"{folder}/testing_demo_camera_{camera_id}",
            data=np.array(data[f"camera_{camera_id}"]),
        )
        cr_interfaces[camera_id].stop()
    robot_interface.close()

    # save = input("Save or not? (enter 0 or 1)")
    # save = bool(int(save))

    # if not save:
    #     import shutil 
    #     shutil.rmtree(f"{folder}")

    print('saved to ', folder)
    tb=Thread(target=beep_end)
    tb.start()
    
if __name__ == "__main__":
    main()
