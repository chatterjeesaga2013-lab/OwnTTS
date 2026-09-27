"""
Author: Milena, Keno
Description: Read out joint data from the G1 robot once (in one position) save to file. For recording full movements use record_joint_data.py

How to: 
1. change which joints to record in config.py
2. run this file
3. json is saved in recordings folder
4. list of recorded joints is copied to clipboard
"""

import time
import json

import pyperclip

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from config import REC_JOINTS, NETWORK_CARD_NAME, RECORDINGS_DIR

G1_JOINT_NAMES = [
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
]

LEG_AND_WAIST_JOINTS = list(range(15))
ARM_JOINTS = list(range(15, 29))


class JointDataRecorder:
    def __init__(self, joints): 
        self.joints = joints 
        self.rec = []
        self.rec2 = []
        self.recording_number = 1
        self.recording = True

        """
        # this doesn't work yet, ReleaseMode() is not enough
        # can't get to zero torque yet
        # change to mode that is easily manipulated
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        
        status, result = self.msc.CheckMode()
        print(result["name"])
        self.msc.ReleaseMode()
        print(result["name"])"""

        

    def low_state_handler(self, msg: LowState_):
        if not self.recording:
            return
        
        for joint in self.joints:
            motorstate = msg.motor_state[joint]
            if self.recording_number == 1:
                self.rec.append(float(motorstate.q))
            else:
                self.rec2.append(float(motorstate.q))
        self.recording = False


if __name__ == '__main__':
    
    ChannelFactoryInitialize(0, NETWORK_CARD_NAME)

    # get joints (from config file) and their names 
    joints = [joint for joint, value in REC_JOINTS.items() if value == 1]
    print("----- Recording the following joints: -----")
    joint_names = []
    for joint_index in joints:
        print(G1_JOINT_NAMES[joint_index])
        joint_names.append(G1_JOINT_NAMES[joint_index])

    action_mode_undetermined = True
    while action_mode_undetermined:
        mode = input("Are you recording one static position (1) or two positions for a dynamic movement (2)? [Type 1 or 2] ")
        if mode == "1":
            action_type = "static"
            action_mode_undetermined = False
        elif mode == "2":
            action_type = "dynamic"
            action_mode_undetermined = False
        else:
            print("Please type either the number 1 or the number 2.")
    
    recorder = JointDataRecorder(joints = joints) 
    low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)

    print("Put the joints in the desired position now.")
    input("Hit enter to start recording")

    low_state_subscriber.Init(recorder.low_state_handler)

    # stop the recorder from over-recording
    while recorder.recording:
        time.sleep(0.25)

    # record second position if needed
    if action_type == "dynamic":
        print("Move the robot to a new position now")
        input("Hit enter to start recording")
        recorder.recording_number = 2
        recorder.recording = True

        while recorder.recording:
            time.sleep(0.25)

    # Saving to file
    filename = input("filename: ")
    try:
        with open(RECORDINGS_DIR + filename + ".json", "w", encoding="utf-8") as f:
            json.dump([action_type, REC_JOINTS, recorder.rec, recorder.rec2], f, ensure_ascii=False, indent=2)
    except OSError as e:
        print(f"Could not write file: {e}")

    # copy to clipboard for convenience 
    #pyperclip.copy(recorder.rec)


        






    

