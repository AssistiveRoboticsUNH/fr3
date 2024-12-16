import argparse
import os
import pickle
import threading
import time
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root 
from deoxys.franka_interface import FrankaInterface 
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="osc-position-controller.yml"
    )
    parser.add_argument("--file", type=Path, default="hdf5 file path")
    parser.add_argument("--episode", type=Path, default="demo_0")

    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    # In this example, we replay the first trajectory in
    # {args.folder}/demo.hdf5 
    # Initialize robot interface
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()
    controller_type = "OSC_POSITION"

    robot_interface = FrankaInterface(config_root + f"/{args.interface_cfg}")

    demo_file_name = str(args.file)
    demo = h5py.File(demo_file_name, "r")

    episode = demo[f"data/{args.episode}"]

    actions = episode["actions"][()]

    for action in actions:
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

    robot_interface.close()


if __name__ == "__main__":
    main()


# python3 examples/replay_hdf5.py --file /home/franka_deoxys/data_franka/imgs_demo/franka_block_gonb_50.hdf5 --episode demo_3
