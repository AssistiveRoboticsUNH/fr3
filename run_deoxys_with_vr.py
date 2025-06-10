import argparse
import time

from deoxys.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger
import matplotlib.pyplot as plt
import numpy as np
from oculus_reader.reader import OculusReader
from scipy.spatial.transform import Rotation as R

logger = get_deoxys_example_logger()

def input_to_action(state, last_state):
    state_pose_data, state_input_data = state
    last_pose_data, last_input_data = last_state

    # Skip if either part is empty
    if not state_pose_data or not last_pose_data:
        return None, None

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
    print("RAW: ", translation, rotation)
    print("CONCATTED: ", [*translation, *rotation])
    return ({
        "delta_pos": translation,
        "delta_rot": rotation,
        # "joystick": joystick_delta,
        "trigger": [trigger_delta],
        "grip": [grip_delta]
    # }, np.array([*translation, *rotation, trigger_delta]))
    }, np.array([*rotation, *translation, trigger_delta]))

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="config/charmander.yml")
    parser.add_argument("--controller-type", type=str, default="OSC_POSE")

    parser.add_argument("--vendor-id", type=int, default=9583)
    parser.add_argument("--product-id", type=int, default=50770) #50734 

    args = parser.parse_args()

    oculus_reader = OculusReader()

    robot_interface = FrankaInterface(args.interface_cfg, use_visualizer=False)

    controller_type = args.controller_type
    controller_cfg = get_default_controller_config(controller_type=controller_type)
    # controller_cfg = YamlConfig("config/osc-pose-controller.yml").as_easydict()

    robot_interface._state_buffer = []

    empty_action_data = {
        "delta_pos": [0.0,0.0,0.0],
        "delta_rot": [0.0,0.0,0.0],
        "trigger": [0.0],
        "grip": [0.0]
    }
    empty_action = np.array([0., 0., 0., 0., 0., 0., -1.0])
    last_state = None
    for i in range(30_000):
        start_time = time.time_ns()
        state = oculus_reader.get_transformations_and_buttons()
        # action, grasp = input2action(
        #     device=device,
        #     controller_type=controller_type,
        # )
        # print("STATE: ", state)
        action = empty_action
        input_action_data = empty_action_data
        if last_state is not None:
            input_action_data, input_action = input_to_action(state, last_state)
            # print("DATA1: ", input_action_data)
            # print("DATA2: ", input_action)
            if input_action_data is not None:
                # print("NOT NONE")
                if input_action_data['grip'][0] > 0:
                    action = input_action
                    pass
        last_state = state
        # grasp = input_action_data['trigger']
        # print("ROBOT ACTION1: ", action)
        action = action * 7
        # action = [0.0,0.0,0.0,0.0,0.0,0.0,action[6]]
        # print("ROBOT ACTION2: ", action)

        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        end_time = time.time_ns()
        logger.debug(f"Time duration: {((end_time - start_time) / (10**9))}")
        logger.info(f"Current Robot joint: {np.round(robot_interface.last_q, 3)}")

    robot_interface.control(
        controller_type=controller_type,
        action=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [1.0],
        controller_cfg=controller_cfg,
        termination=True,
    )

    robot_interface.close()

    # Check if there is any state frame missing
    for (state, next_state) in zip(
        robot_interface._state_buffer[:-1], robot_interface._state_buffer[1:]
    ):
        if (next_state.frame - state.frame) > 1:
            print(state.frame, next_state.frame)


if __name__ == "__main__":
    main()
