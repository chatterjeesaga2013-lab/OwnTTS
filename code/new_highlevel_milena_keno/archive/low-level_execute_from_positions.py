import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np
import json

from config import REC_JOINTS

G1_NUM_MOTOR = 29
ROUND_WAIT = 1

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

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof


class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Executer:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]
        self.duration_dyn_ = 3.0 + ROUND_WAIT
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()

        # movements
        self.done = False
        self.joints = []
        self.action_type = "static"
        
        # base target is a neutral position
        self.target = [0.045048702508211136, 0.07023954391479492, 0.17014004290103912, 1.2804545164108276, 0.4430568218231201, 0.17042766511440277, 0.014584802091121674, 0.16109195351600647, -0.1435350626707077, 0.2588832378387451, 1.1212440729141235, 0.23610122501850128, 0.11965050548315048, -0.22583073377609253]
        self.target2 = []


    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)


    def set_target_position(self, action):
        self.done = False
        try:
            with open("../new_highlevel_milena_keno/recordings/positions/" + action + ".json", "r", encoding="utf-8") as f:
                data = json.load(f)
        except OSError as e:
            print(f"Could not read file: {e}")

        self.action_type = data[0]
        self.joints = [int(joint) for joint, value in data[1].items() if value == 1]
        self.target = data[2]
        self.target2 = data[3]

        self.time_ = 0.0


    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
        
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        if self.action_type == "static":
            if self.time_ < self.duration_ :
                for j, i in enumerate(self.joints):
                    ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                    self.low_cmd.mode_pr = Mode.PR
                    self.low_cmd.mode_machine = self.mode_machine_
                    self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                    self.low_cmd.motor_cmd[i].tau = 0.
                    self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q + ratio * self.target[j]
                    self.low_cmd.motor_cmd[i].dq = 0. 
                    self.low_cmd.motor_cmd[i].kp = Kp[i] 
                    self.low_cmd.motor_cmd[i].kd = Kd[i]
            elif self.time_ < self.duration_ + ROUND_WAIT:
                pass 
            else:
                self.done = True # still needed?
            
            
        else:
            # maybe add minimum time to avoid snapping?
            # 3 is just a decision that the first third of the interval 
            # is used to get into position, the other portion is for the movement
            if self.time_ < self.duration_dyn_/ 3: 
                for j, i in enumerate(self.joints):
                    ratio = np.clip(self.time_ / self.duration_dyn_, 0.0, 1.0)
                    self.low_cmd.mode_pr = Mode.PR
                    self.low_cmd.mode_machine = self.mode_machine_
                    self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                    self.low_cmd.motor_cmd[i].tau = 0.
                    self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q + ratio * self.target[j]
                    self.low_cmd.motor_cmd[i].dq = 0. 
                    self.low_cmd.motor_cmd[i].kp = Kp[i] 
                    self.low_cmd.motor_cmd[i].kd = Kd[i]

            elif self.time_ < self.duration_dyn_:
                reps = 8
                target_list = [self.target2, self.target] * int(reps / 2)

                remaining = self.time_ - self.duration_dyn_ / 3
                interval_len = ( 2 * self.duration_dyn_ / 3) / reps
                interval = int(remaining / interval_len)

                # change targets
                current_target = target_list[interval]

                # remaining time of the interval
                interval_remaining = remaining - interval_len * interval 
                for j, i in enumerate(self.joints):
                    ratio = np.clip(interval_remaining / interval_len, 0.0, 1.0)
                    self.low_cmd.mode_pr = Mode.PR
                    self.low_cmd.mode_machine = self.mode_machine_
                    self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                    self.low_cmd.motor_cmd[i].tau = 0.
                    self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q + ratio * current_target[j]
                    self.low_cmd.motor_cmd[i].dq = 0. 
                    self.low_cmd.motor_cmd[i].kp = Kp[i] 
                    self.low_cmd.motor_cmd[i].kd = Kd[i]
            else:
                self.done = True
        

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    Executer = Executer()
    Executer.Init()
    Executer.Start()
Mode
    
    Executer.set_target_position("salute")
    while True:        
        time.sleep(3)
        if Executer.done: 
           print("Done!")
           sys.exit(-1)