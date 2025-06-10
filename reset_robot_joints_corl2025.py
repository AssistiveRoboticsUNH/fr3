"""Moving robot joint positions to initial pose for starting new experiments."""
import argparse
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()





def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--controller-cfg", type=str, default="joint-position-controller.yml")
    parser.add_argument("--folder", type=Path, default="data_collection_example/example_data")

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()

    controller_type = "JOINT_POSITION"

    # Golden resetting joints
    # reset_joint_positions = [
    #     0.032,
    #     -1.159,
    #     0.036,
    #     -2.384,
    #     -0.006,
    #     1.546,
    #     0.888
    # ]
    


    reset_joint_positions = [
        0.037,-1.655,0.099,-2.993,-0.074,2.321,0.921]
    
    # reset_joint_positions_0=[-0.086,0.747,0.118,-1.295,
    #                           -0.069,  3.68,   0.901]
    

    reset_joint_positions_0 = [-0.03,0.244,-0.03,
                               -1.673,0.134,1.905,0.665]
    

    # This is for varying initialization of joints a little bit to
    # increase data variation.
    reset_joint_positions = [
        e + np.clip(np.random.randn() * 0.005, -0.005, 0.005)
        for e in reset_joint_positions
    ]
    reset_joint_positions_0 = [
        e + np.clip(np.random.randn() * 0.005, -0.005, 0.005)
        for e in reset_joint_positions_0
    ]
    action = reset_joint_positions + [-1.0]
    action_0 = reset_joint_positions_0 + [-1.0]


    

    while True:
        if len(robot_interface._state_buffer) > 0:
            logger.info(f"Current Robot joint: {np.round(robot_interface.last_q, 3)}")
            logger.info(f"Desired Robot joint: {np.round(robot_interface.last_q_d, 3)}")

            if (
                np.max(
                    np.abs(
                        np.array(robot_interface._state_buffer[-1].q)
                        - np.array(reset_joint_positions_0)
                    )
                )
                < 1e-3
            ):
                break
        robot_interface.control(
            controller_type=controller_type,
            action=action_0,
            controller_cfg=controller_cfg,
        )

    
    
    robot_interface.close()


if __name__ == "__main__":
    main()
