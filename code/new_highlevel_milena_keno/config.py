REC_JOINTS = {  # 0 = not recorded, 1 = recorded
        0: 0,   # left_hip_pitch
        1: 0,   # left_hip_roll
        2: 0,   # left_hip_yaw
        3: 0,   # left_knee
        4: 0,   # left_ankle_pitch
        5: 0,   # left_ankle_roll
        6: 0,   # right_hip_pitch
        7: 0,   # right_hip_roll
        8: 0,   # right_hip_yaw
        9: 0,   # right_knee
        10: 0,  # right_ankle_pitch
        11: 0,  # right_ankle_roll
        12: 0,  # waist_yaw
        13: 0,  # waist_roll
        14: 0,  # waist_pitch
        ############################# ARMS #####################
        15: 1,  # left_shoulder_pitch
        16: 1,  # left_shoulder_roll
        17: 1,  # left_shoulder_yaw
        18: 1,  # left_elbow
        19: 1,  # left_wrist_roll
        20: 1,  # left_wrist_pitch
        21: 1,  # left_wrist_yaw
        22: 1,  # right_shoulder_pitch
        23: 1,  # right_shoulder_roll
        24: 1,  # right_shoulder_yaw
        25: 1,  # right_elbow
        26: 1,  # right_wrist_roll
        27: 1,  # right_wrist_pitch
        28: 1,  # right_wrist_yaw
    }

NETWORK_CARD_NAME = "enp0s31f6"
ROUND_WAIT = .5
RECORDINGS_DIR = "../new_highlevel_milena_keno/recordings/positions/"