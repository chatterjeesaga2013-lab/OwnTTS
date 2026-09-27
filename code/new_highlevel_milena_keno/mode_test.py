import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient


kPi = 3.141592654
kPi_2 = 1.57079632


# ---------------- JOINT IDS ----------------
class G1JointIndex:
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    kNotUsedJoint = 29


# ---------------- MAIN CONTROLLER ----------------
class Custom:

    def __init__(self):

        self.time_ = 0.0
        self.control_dt_ = 0.02
        self.duration_ = 3.0

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None

        self.first_update_low_state = False
        self.done = False

        self.crc = CRC()

        self.kp = 20.0
        self.kd = 0.3

        # arm joints only (IMPORTANT: no waist)
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
            G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,

            G1JointIndex.RightShoulderPitch,
            G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,
            G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,
            G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
        ]

        self.target_left_elbow_up = 1.4

        self.motion_switcher = MotionSwitcherClient()

    # ---------------- INIT ----------------
    def Init(self):

        self.motion_switcher.Init()

        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    # ---------------- STATE ----------------
    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.first_update_low_state = True

    # ---------------- MODE SWITCH ----------------
    def WaitForMode(self, target="ai"):

        print("Requesting mode:", target)
        self.motion_switcher.SelectMode(target)

        for _ in range(50):

            status, state = self.motion_switcher.CheckMode()
            print("current mode:", state["name"])

            if state["name"] == target:
                print("Mode active:", target)
                return True

            time.sleep(0.2)

        raise RuntimeError("Failed to enter AI mode")

    # ---------------- CONTROL LOOP ----------------
    def LowCmdWrite(self):

        self.time_ += self.control_dt_

        # ALWAYS enable arm sdk
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

        # default gains every cycle (important for stability)
        for j in self.arm_joints:
            self.low_cmd.motor_cmd[j].tau = 0
            self.low_cmd.motor_cmd[j].dq = 0
            self.low_cmd.motor_cmd[j].kp = self.kp
            self.low_cmd.motor_cmd[j].kd = self.kd

        # ---------------- simple wave ----------------
        t = self.time_

        left_elbow = 1.2 + 0.3 * np.sin(2 * np.pi * 0.5 * t)

        self.low_cmd.motor_cmd[G1JointIndex.LeftElbow].q = left_elbow

        # publish
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

        if self.time_ > 10:
            self.done = True

    # ---------------- START ----------------
    def Start(self):

        self.thread = RecurrentThread(
            interval=self.control_dt_,
            target=self.LowCmdWrite,
            name="arm_control"
        )

        while not self.first_update_low_state:
            time.sleep(0.1)

        self.thread.Start()


# ---------------- MAIN ----------------
if __name__ == '__main__':

    print("Initializing...")

    ChannelFactoryInitialize(0)

    robot = Custom()
    robot.Init()

    # ---------------- MODE SWITCH SEQUENCE ----------------
    robot.motion_switcher.ReleaseMode()
    time.sleep(1.0)

    robot.WaitForMode("ai")

    time.sleep(2.0)

    # ---------------- START CONTROL ----------------
    robot.Start()

    while True:
        time.sleep(1)

        if robot.done:
            print("Done")
            break
