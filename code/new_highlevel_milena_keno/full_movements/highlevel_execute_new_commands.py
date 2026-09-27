"""
NOTE: Doesn't work at the moment!


Author: Milena, Keno
Description: Execute movements from recorded joint data on G1 robot

How to: 
1. add filename in terminal (e.g. python execute_rec_movement.py <filename>) or as input 
2. stand clear!
"""


import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np
import json


sys.path.insert(1, "../full_movements")
from config import NETWORK_CARD_NAME


G1_NUM_MOTOR = 29

Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms 
]

class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Executer:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        self.done = False


    def Init(self):

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher_.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        

    def Start(self, filename):
        self.done = False
        self.update_mode_machine_ = False
        # open recordings file
        try:
            with open("recordings/" + filename + ".json", "r", encoding="utf-8") as f:
                data = json.load(f)
        except OSError as e:
            print(f"Could not read file: {e}")

        self.rec_dt = data["dt"]
        self.steps = data["steps"]
        self.joints = [int(k) for k in self.steps[0].keys()]
        self.num_steps = len(self.steps)
    

        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0
            #print(self.low_state.imu_state.rpy)

    def LowCmdWrite(self):

        self.time_ += self.control_dt_
        step_index = int(self.time_ / self.rec_dt)
        #print(step_index)
        if step_index >= self.num_steps:
            self.done = True
            return 
        
        step = self.steps[step_index]

        #if self.time_ < self.duration_ * (i+1): 
        """
        for all other joints:
            stay where you are
        """
        for joint in self.joints:
            if step_index == 0:
                ratio = np.clip(self.time_ /self.rec_dt, 0.0, 1.0)
            else:
                ratio = np.clip((self.time_ - self.rec_dt* step_index ) / (self.rec_dt * step_index ), 0.0, 1.0)
            
            self.low_cmd.motor_cmd[joint].mode =  1 # 1:Enable, 0:Disable
            self.low_cmd.motor_cmd[joint].tau = 0.
            self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * step[str(joint)]
            self.low_cmd.motor_cmd[joint].q = step[str(joint)]
            self.low_cmd.motor_cmd[joint].dq = 0. 
            self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
            self.low_cmd.motor_cmd[joint].kd = Kd[joint]
        #else:
            #self.done = True


        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running movements")
    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, NETWORK_CARD_NAME)

    if len(sys.argv)>1:
        filename = sys.argv[1]
    else:
        filename = input("Filename (without '.json'): ")


    Executer = Executer()
    Executer.Init()
    Executer.Start(filename=filename)

    while True:        
        time.sleep(1)
        if Executer.done: 
           print("Done!")
           sys.exit(-1)
