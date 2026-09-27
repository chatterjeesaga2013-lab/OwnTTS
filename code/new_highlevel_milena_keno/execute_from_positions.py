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

G1_NUM_MOTOR = 29
from config import ROUND_WAIT, RECORDINGS_DIR

Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    100, 100, 100,                # waist, set higher than default (60, 40, 40) for better control
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    2, 2, 2,              # waist, set higher than default (1,1,1)
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

    kNotUsedJoint = 29 # NOTE: Weight


def calc_waist(shoulder_l, shoulder_r, elbow_l, elbow_r):
    """corrects for a rotation due to asymetrical arm movements"""
    w1_yaw = 0 # tbd, should be positive, around -.1 for drumming
    w2_yaw = 0
    yaw = np.clip(w1_yaw * (shoulder_l - shoulder_r) + w2_yaw * (elbow_l - elbow_r), -2.6, 2.6)

    """correct for leaning to the right / left due to asymetrical arm movements"""
    w1_roll = 0 # tbd
    w2_roll = 0
    roll = np.clip(w1_roll * (shoulder_r - shoulder_l) + w2_roll * (elbow_r - elbow_l), -.52, .52)

    """corrects for a forward / backwards leaning position
    due to the weight of the arms """
    w1_pitch = 0.1222 # used to be 0.222
    w2_pitch = -0.018
    pitch = np.clip(w1_pitch * (shoulder_r + shoulder_l) + w2_pitch * (elbow_r + elbow_l), -.52, .52)

    return ([-0.001, 0.0001, pitch]) # replace values with yaw and roll once working!


class Executer:
    def __init__(self):
        # timings 
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s] 
        self.duration_dyn_ = 3.0 + ROUND_WAIT
        if self.duration_dyn_ < 2.5:
            sys.exit("""The duration for dynamic movements is too short (< 2.5 seconds), 
            please check the script to avoid snapping!
            Maybe the ROUND_WAIT variable is negative?""")

        self.counter_ = 0

        # unitree stuff
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        self.first_update_low_state = False

        # movements
        self.script_control = True # script control via arm_sdk 
        self.done = False
        self.position = ["", 0] # current target position and ratio achieved
        self.joints = []    # which joints are controlled by the script
        self.action_type = "static"
        
        # base target is a neutral position
        self.target = [0.045048702508211136, 0.07023954391479492, 0.17014004290103912, 1.2804545164108276, 0.4430568218231201, 0.17042766511440277, 0.014584802091121674, 0.16109195351600647, -0.1435350626707077, 0.2588832378387451, 1.1212440729141235, 0.23610122501850128, 0.11965050548315048, -0.22583073377609253]
        self.target2 = []
        

    def Init(self):
        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher_.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def set_target_position(self, action):
        # reset to start
        self.time_ = 0.0
        self.done = False
        self.position = [action, 0]

        # open new target data
        try:
            with open(RECORDINGS_DIR + action + ".json", "r", encoding="utf-8") as f:
                data = json.load(f)
        except OSError as e:
            sys.exit(f"Could not read file: {e}")

        # set parameters of movement
        self.action_type = data[0]
        self.joints = [int(joint) for joint, value in data[1].items() if value == 1]
        self.target = data[2]
        self.target2 = data[3]
    
        

    def Start(self):
        """ Start up the Executer with recurrent threads """
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.first_update_low_state == False:
            time.sleep(1)
        
        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0

            
        if self.first_update_low_state == False:
                    self.first_update_low_state = True

    def LowCmdWrite(self):
        """ 
        This is where the magic happens :) 
        All the movement definitions are in this function
        """
        self.time_ += self.control_dt_

        # to end the script and remove control from the arm_sdk / script
        if not self.script_control:
            if self.time_ < 1.0:
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  np.clip(1 - self.time_, 0.0, 1.0) # 1:Enable arm_sdk, 0:Disable arm_sdk
            else:            
                self.done = True

        
        if self.action_type == "static":
            # handing over control from robot to script within the first second
            # this only happens if the the control is not with the arm_sdk / script yet
            # i.e. first time this function is executed
            if self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q < 1:
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  np.clip(self.time_, 0.0, 1.0) # 1:Enable arm_sdk, 0:Disable arm_sdk
            else:             
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1


            # control waist
            waist_correction = calc_waist(
                self.low_state.motor_state[15].q, 
                self.low_state.motor_state[22].q, 
                self.low_state.motor_state[18].q, 
                self.low_state.motor_state[25].q
                )
            # this has to be separate from the other joints due to to the quicker correction
            for i, joint in enumerate([12, 13, 14]):
                ratio = np.clip(self.time_ / 0.5, 0.0, 1.0)
                self.low_cmd.mode_pr = 0 # pitch/roll joints (0)
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * waist_correction[i]
                self.low_cmd.motor_cmd[joint].dq = 0. 
                self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                self.low_cmd.motor_cmd[joint].kd = Kd[joint]

            # operate the rest of the joints
            for i, joint in enumerate(self.joints):
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.position[1] = ratio
                self.low_cmd.mode_pr = 1 # AB joints (1)
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * self.target[i]
                self.low_cmd.motor_cmd[joint].dq = 0. 
                self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                self.low_cmd.motor_cmd[joint].kd = Kd[joint]
                    
            
            if self.time_ >= self.duration_ + ROUND_WAIT:
                self.position[1] = 1
                self.done = True 


            
        else: # dynamic movements
            # 3 is just a decision that the first third of the interval 
            # is used to get into position, the other portion is for the movement
            self.position[1] = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            if self.time_ < self.duration_dyn_/ 3: 

                # handing over control from robot to script within the first second
                # this only happens if the the control is not with the arm_sdk / script yet
                # i.e. first time this function is executed
                if self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q < 1:
                    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  np.clip(self.time_, 0.0, 1.0) # 1:Enable arm_sdk, 0:Disable arm_sdk
                else:             
                    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

                # control waist
                waist_correction = calc_waist(
                    self.low_state.motor_state[15].q, 
                    self.low_state.motor_state[22].q, 
                    self.low_state.motor_state[18].q, 
                    self.low_state.motor_state[25].q
                    )
                # this has to be separate from the other joints due to to the quicker correction
                for i, joint in enumerate([12, 13, 14]):
                    ratio = np.clip(self.time_ / 0.5, 0.0, 1.0)
                    self.low_cmd.mode_pr = 0 # pitch/roll joints (0)
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * waist_correction[i]
                    self.low_cmd.motor_cmd[joint].dq = 0. 
                    self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                    self.low_cmd.motor_cmd[joint].kd = Kd[joint]

                # operate the rest of the joints
                for i, joint in enumerate(self.joints):
                    ratio = np.clip(self.time_ / self.duration_dyn_, 0.0, 1.0)
                    self.low_cmd.mode_pr = 1 # AB joints (1)
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * self.target[i]
                    self.low_cmd.motor_cmd[joint].dq = 0. 
                    self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                    self.low_cmd.motor_cmd[joint].kd = Kd[joint]

            elif self.time_ < self.duration_dyn_:
                """ targets """
                reps = 8
                target_list = [self.target2, self.target] * int(reps / 2)

                remaining = self.time_ - self.duration_dyn_ / 3
                interval_len = ( 2 * self.duration_dyn_ / 3) / reps
                interval = int(remaining / interval_len)

                # change targets
                current_target = target_list[interval]

                # remaining time of the interval
                interval_remaining = remaining - interval_len * interval 

                """control"""
                # make sure the arm_sdk keeps control
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

                # control waist
                waist_correction = calc_waist(
                    self.low_state.motor_state[15].q, 
                    self.low_state.motor_state[22].q, 
                    self.low_state.motor_state[18].q, 
                    self.low_state.motor_state[25].q
                    )
                # this has to be separate from the other joints due to to the quicker correction
                for i, joint in enumerate([12, 13, 14]):
                    ratio = np.clip(self.time_ / 0.5, 0.0, 1.0)
                    self.low_cmd.mode_pr = 0 # pitch/roll joints (0)
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * waist_correction[i]
                    self.low_cmd.motor_cmd[joint].dq = 0. 
                    self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                    self.low_cmd.motor_cmd[joint].kd = Kd[joint]

                # rest of the joints
                for i, joint in enumerate(self.joints):
                    ratio = np.clip(interval_remaining / interval_len, 0.0, 1.0)
                    self.position[1] = ratio
                    self.low_cmd.mode_pr = 1 # AB joints (1)
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * current_target[i]
                    self.low_cmd.motor_cmd[joint].dq = 0. 
                    self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                    self.low_cmd.motor_cmd[joint].kd = Kd[joint]
            else:
                self.done = True
                self.position[1] = 1

                # keep the robot in its position until further instructions
                # control waist
                waist_correction = calc_waist(
                    self.low_state.motor_state[15].q, 
                    self.low_state.motor_state[22].q, 
                    self.low_state.motor_state[18].q, 
                    self.low_state.motor_state[25].q
                    )
                # this has to be separate from the other joints due to to the quicker correction
                for i, joint in enumerate([12, 13, 14]):
                    ratio = np.clip(self.time_ / 0.5, 0.0, 1.0)
                    self.low_cmd.mode_pr = 0 # pitch/roll joints (0)
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * waist_correction[i]
                    self.low_cmd.motor_cmd[joint].dq = 0. 
                    self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                    self.low_cmd.motor_cmd[joint].kd = Kd[joint]
                
                # operate the rest of the joints
                for i, joint in enumerate(self.joints):
                    self.low_cmd.mode_pr = 1 # AB joints (1)
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = self.target[i]
                    self.low_cmd.motor_cmd[joint].dq = 0. 
                    self.low_cmd.motor_cmd[joint].kp = Kp[joint] 
                    self.low_cmd.motor_cmd[joint].kd = Kd[joint]
        
        
        
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

    def give_control_back(self):
        """Checks if the robot is in neutral, 
        then sets the robot up to remove the script control from the arm_sdk"""

        print("All movements are done. Starting exit sequence. The robot will regain full control now.")
        self.done = False
        
        if self.position == ["neutral", 1]:
            self.time_ = 0.0
            self.script_control = False
        else:
            print("Going into neutral first...")
            self.set_target_position("neutral")
            time.sleep(self.duration_)
            self.script_control = False

    def get_current_position(self):
        """Returns the current target position and how far into the position it is."""
        return self.position

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this script.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    Executer = Executer()
    Executer.Init()
    Executer.Start()


    Executer.set_target_position("salute")
    while not Executer.done:
        time.sleep(.25)

    time.sleep(2)
    Executer.give_control_back()
    
    while True:        
        time.sleep(1)
        if Executer.done and not Executer.script_control: 
            print("Robot has regained control. You should be able to use the remote control again.") 
            print("Exiting...")
            sys.exit(0)