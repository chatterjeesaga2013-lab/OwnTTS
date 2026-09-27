# Recording and executing new movements
Record movements or positions by manipulating the robot and execute them. See [Current State](#current-state)



## To do:
- recording: 
    - go to damp (?) mode for recordings; work around: restart robot and record in zero-torque mode
    - shorten recording time (end), if interested in full movements
- clean highlevel-changes in this file

- execution:
    - waist: -> probably not necessary any more
        - add shoulder roll to waist_calc ?
        - ~~add waist roll, yaw control, only issue is currently with "drumming"~~
        - adjust formular (?)
        - remove waist yaw and roll for cleanliness or use figure out correct calculations
    - occasionally hitting itself (end/losing control?) (loser!)
        - clip values such that it doesn't hit itself





## Current state


- **recording positions** (preferred! run ```record_joints_single_position.py```)
    - the script asks for either a single position or two positions for a dynamic movement. After hitting record (enter) it records all specified joints to a list, which is saved to a file in recordings/positions. The file can be loaded in ```execute_from_positions.py``` (see below)
- **execution from positions** (single, static positions or a switching set of two positions (e.g. drumming motion)) ```execute_from_positions.py```

    - this doesn't go to neutral in between, i.e. goes from one position into the next
- for recording and executing **complete motion sequences** by manipulating the robot see folder ```full_movements``` 
    - **recording movements** works (run ```record_joint_data.py```)
        - it records at a set frequency all the specified joints and saves to a json file in the recordings folder (the program asks for a filename, you do not need to add the ".json"!)
    - a new movement has been created from scratch (no recording, see ```the_dead_arm.py```) 
    - execution of new commands from recordings overall works (```execute_new_commands.py```)

## Archive
Here live all the no longer relevant files, e.g. low-level execution files, which might still be useful in the future. 

Current files and their state
| file name                 | state (e.g. functional) | known issues          | archive date |   |
|---------------------------|-------------------------|-----------------------|--------------|---|
| execute_from_positions.py | functional?             | waist control missing | 26.08.       |   |
| execute_new_commands.py | functional?             | waist control missing | 26.08.       |   |
|                           |                         |                       |              |   |



## Changes from low-level to highlevel file

- in class G1JointIndex add ```kNotUsedJoint = 29 # NOTE: Weight```, this is used as a weight of how much control the arm sdk gets, the weight iteself is set later
-  **__init__(self)**
    - add

    ```self.first_update_low_state = False```
    - remove for cleanliness:
    ```python
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_() 
    ```

- remove the following from **Init(self)**
    ```python
    self.msc = MotionSwitcherClient()
    self.msc.SetTimeout(5.0)
    self.msc.Init()

    status, result = self.msc.CheckMode()
    while result['name']:
        self.msc.ReleaseMode()
        status, result = self.msc.CheckMode()
        time.sleep(1)
    ```
- in **Init(self)** change ```ChannelPublisher("rt/lowstate", LowState_)``` to ```ChannelPublisher("rt/arm_sdk", LowCmd_)``` (not for ChannelSubscriber)

- remove the following from **LowStateHandler**
    ```python
    if self.update_mode_machine_ == False:
        self.mode_machine_ = self.low_state.mode_machine
        self.update_mode_machine_ = True
    ```
    and replace with:
    ```python
    if self.first_update_low_state == False:
                    self.first_update_low_state = True
    ```
- replace **Start(self):** with
    ```python
    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.first_update_low_state == False:
            time.sleep(1)
        
        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()
    ```
- in **LowCmdWrite(self)** 
    - remove all 
    ```python
    self.low_cmd.mode_pr = Mode.PR
    self.low_cmd.mode_machine = self.mode_machine_
    ```
    - after all ```if self.time_ < duration_``` add **either** ```self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:Disable arm_sdk```  (can snap this way!) **or** something similar to the below code to hand over control slowly (recomended)
    ```python
    # handing over control from robot to script within the first second
                # this only happens if the the control is not with the arm_sdk / script yet
                # i.e. first time this function is executed
                if self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q < 1:
                    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  np.clip(self.time_, 0.0, 1.0) # 1:Enable arm_sdk, 0:Disable arm_sdk
                else:             
                    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
    ```

