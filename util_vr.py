import numpy as np
from scipy.spatial.transform import Rotation as R

###########################################################
# Function to parse the oculus reader state inputs to the
# 7 dimensional action input for the franka
###########################################################

def adjust_x_axis(translation):
    correction_rot = R.from_euler('x', -60, degrees=True) # adjust axis and angle as needed translation = correction_rot.apply(translation)
    translation = correction_rot.apply(translation)
    return translation

def compensate_for_z_drift(translation, rotation):
    forward_movement = translation[0]  # robot forward/backward axis
    rotation_drift_correction_scale = -0.08  # Tune this value experimentally

    # Apply a pitch correction (rotation[1] is pitch in your mapping)
    rotation[1] += -forward_movement * rotation_drift_correction_scale

    return rotation

def smoothing(translation, rotation, alpha = 0.2, prev_smooth=None):
    translation_np = np.array(translation)
    rotation_np = np.array(rotation)

    alpha = 0.2

    if prev_smooth is None:
        prev_smooth = {
            "pos": translation_np,
            "rot": rotation_np
        }

    smoothed_translation = alpha * translation_np + (1 - alpha) * prev_smooth["pos"]
    smoothed_rotation = alpha * rotation_np + (1 - alpha) * prev_smooth["rot"]

    translation = smoothed_translation.tolist()
    rotation = smoothed_rotation.tolist()

    return translation, smoothed_translation, rotation, smoothed_rotation, prev_smooth

def input_to_action(state, last_state, prev_smooth = None):
    state_pose_data, state_input_data = state
    last_pose_data, last_input_data = last_state

    # Skip if either part is empty
    if not state_pose_data or not last_pose_data:
        return None, None, None

    pose1 = last_pose_data['r']
    pose2 = state_pose_data['r']

    np.set_printoptions(precision=2)

    delta_pose = np.linalg.inv(pose1) @ pose2

    translation = delta_pose[:3, 3]
    rotation_matrix = delta_pose[:3, :3]

    # velocity = np.linalg.norm(translation)
    # translation = add_x_rotation_offset(translation, velocity, -300) 

    translation = adjust_x_axis(translation)



    rotation = R.from_matrix(rotation_matrix).as_euler('xyz')

    if(state_input_data['leftGrip'][0] > 0):
        enable_delta = True
    else:
        enable_delta = False


    if(state_input_data['rightTrig'][0] > 0):
        trigger_delta = 1
    else:
        trigger_delta = -1

    if(state_input_data['rightGrip'][0] > 0):
        grip_delta = 1
    else:
        grip_delta = -1

    grip_delta = 1

    A_button = state_input_data['A']

    translation, smoothed_translation, rotation, smoothed_rotation, prev_smooth = smoothing(translation, rotation, 0.2, prev_smooth)

    rotation_scale = 8

    flipped = False

    flippedConst = 1
    if flipped:
        flippedConst = -1

    rotation = [
        rotation[1] * flippedConst * rotation_scale, # Z rotation
        rotation[0] * flippedConst * rotation_scale * 0.8, # X rotation
        -rotation[2] * rotation_scale # Y rotation
    ]

    translation_scale = 50
    translation_sum = abs(translation[0]) + abs(translation[1]) + abs(translation[2])

    if translation_sum < 0.8:
        translation_scale = 150

    translation = [
        translation[1] * flippedConst * translation_scale / 2 * 5 , # Z / forward backward
        translation[0] * flippedConst * translation_scale * 1.2,  # X / left right
        -translation[2] * translation_scale # Y / up down
    ]


    rotation = compensate_for_z_drift(translation, rotation)

    delta = {
        "delta_pos": translation,
        "delta_rot": rotation,
        # "joystick": joystick_delta,
        "trigger": [trigger_delta],
        "grip": [grip_delta],
        "A_button": A_button,
        "enable": enable_delta
    }

    data = np.array([*translation, *rotation, trigger_delta])

    prev_smooth = {
        "pos": smoothed_translation,
        "rot": smoothed_rotation
    }

    return delta, data, prev_smooth

###########################################################
# Read vr controller state and perform logic controls
# and state manipulation logic
############################################################

def read_vr_action(oculus_reader, last_state, last_trigger, last_prev_smooth):
    empty_action_data = {
        "delta_pos": [0.0,0.0,0.0],
        "delta_rot": [0.0,0.0,0.0],
        "trigger": [0.0],
        "grip": [0.0],
        "A_button": False,
        "enable": False
    }
    empty_action = np.array([0., 0., 0., 0., 0., 0., -1.0])


    empty_action[6] = last_trigger
    
    state = oculus_reader.get_transformations_and_buttons()
    prev_smooth = None
    action = empty_action
    A_button = False
    enable = False
    input_action_data = empty_action_data
    if last_state is not None:
        input_action_data, input_action, prev_smooth = input_to_action(state, last_state, last_prev_smooth)
        if input_action_data is not None:
            A_button = input_action_data['A_button']
            
            enable = input_action_data['enable']
            # enable = True

            if input_action_data['grip'][0] > 0 and enable:
                action = input_action
                last_trigger = input_action[6]
                # pass
    last_state = state

    return action, last_state, last_trigger, A_button, prev_smooth
