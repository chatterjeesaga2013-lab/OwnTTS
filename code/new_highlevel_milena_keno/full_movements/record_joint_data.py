"""
Author: Milena, Keno
Description: Read out joint data from the G1 robot, save to file

How to: 
1. change which joints to record in config.py
    only record waist joints if you specifically want movement there!
2. run this file
3. json is saved in recordings folder
"""

import time
import json

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from config import REC_JOINTS, NETWORK_CARD_NAME

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
    def __init__(self, joints, rec_hz=500.0): 
        self._msg_count = 0
        self._period = 1.0 / rec_hz
        self._last_print_time = 0.0
        self.joints = joints 
        self.recs = {
            "dt" : 1/rec_hz,
            "steps" : []
        } 

    def low_state_handler(self, msg: LowState_):

        step = {} 
        now = time.time()
        if now - self._last_print_time < self._period:
            return

        self._last_print_time = now
        self._msg_count += 1
        for joint in self.joints:
            motorstate = msg.motor_state[joint]
            step[str(joint)] = float(motorstate.q)
        
        self.recs["steps"].append(step)



if __name__ == '__main__':
    
    ChannelFactoryInitialize(0, NETWORK_CARD_NAME)

    joints = [joint for joint, value in REC_JOINTS.items() if value == 1]

    print("----- Recording the following joints: -----")
    for joint_index in joints:
        print(G1_JOINT_NAMES[joint_index])

    print("Use Ctrl-C to stop recording.")

    input("Hit enter to start recording")
    
    # only change the recording hz if you know what you're doing!
    recorder = JointDataRecorder(joints = joints) 
    low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    low_state_subscriber.Init(recorder.low_state_handler, 10)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped recording.")
    finally:
        recs = recorder.recs
        hz = recorder._period
        filename = input("filename: ")
        
        try:
            with open("recordings/" + filename + ".json", "w", encoding="utf-8") as f:
                json.dump({**recs}, f, ensure_ascii=False, indent=2)
        except OSError as e:
            print(f"Could not write file: {e}")


        






    

