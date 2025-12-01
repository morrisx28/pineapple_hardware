import time
import sys
import numpy as np
import threading
import traceback
import yaml
import csv
import argparse

from csl_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from csl_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from csl_sdk2py.idl.default import csl_pineapple_msg_dds__LowCmd_
from csl_sdk2py.idl.default import csl_pineapple_msg_dds__LowState_
from csl_sdk2py.idl.csl_pineapple.msg.dds_ import LowCmd_
from csl_sdk2py.idl.csl_pineapple.msg.dds_ import LowState_
from csl_sdk2py.utils.crc import CRC
from csl_sdk2py.utils.thread import RecurrentThread

NUM_MOTORS = 4

class Filter:
    def __init__(self, alpha):
        self.filter_value = None
        self.alpha = alpha
    
    def filt(self, input):
        if self.filter_value is None:
            self.filter_value = input
        else:
            self.filter_value = self.alpha * input + (1 - self.alpha) * self.filter_value
        return self.filter_value

class Controller:
    def __init__(self):

        self.low_cmd = csl_pineapple_msg_dds__LowCmd_()  
        self.low_state = None  


        self.controller_rt = 0.0
        self.is_running = False

        self.ang_vel_data = []
        self.qtau_data = []
        self.qtau_cmd = []

        # thread handling
        self.lowCmdWriteThreadPtr = None

        # state
        self.target_dof_vel = np.zeros(NUM_MOTORS)
        self.qpos = np.zeros(NUM_MOTORS, dtype=np.float32)
        self.qvel = np.zeros(NUM_MOTORS, dtype=np.float32)
        self.qtau = np.zeros(NUM_MOTORS, dtype=np.float32)
        self.quat = np.zeros(4) # q_w q_x q_y q_z
        self.ang_vel = np.zeros(3)

        self.mode = ''
        self.dt = 0.001


        self.crc = CRC()

    # Control methods
    def Init(self):
        self.InitLowCmd()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        # Init default pos #
        self.Start()

        print("Initial Sucess !!!")

    def get_gravity_orientation(self, quaternion):
        qw = quaternion[0]
        qx = quaternion[1]
        qy = quaternion[2]
        qz = quaternion[3]

        gravity_orientation = np.zeros(3)

        gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
        gravity_orientation[1] = -2 * (qz * qy + qw * qx)
        gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

        return gravity_orientation
    

    def Start(self):
        self.is_running = True
        self.lowCmdWriteThreadPtr = threading.Thread(target=self.LowCmdWrite, daemon=True)
        self.lowCmdWriteThreadPtr.start()

    def ShutDown(self):
        self.is_running = False


    # Private methods
    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(NUM_MOTORS):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= 0
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        self.get_current_state()
        # print(f'qpos {self.low_state.motor_state[0].q}')
        # quat = self.low_state.imu_state.quaternion
        # ang_vel = self.low_state.imu_state.gyroscope
        # print(f'quat w: {quat[0]} x: {quat[1]} y: {quat[2]} z: {quat[3]}')
        # print(f'ang_vel x: {ang_vel[0]} y: {ang_vel[1]} z: {ang_vel[2]}')
    
    
    def test(self):
        for i in range(NUM_MOTORS):
            self.low_cmd.motor_cmd[i].q = 0
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = 1.0
            self.low_cmd.motor_cmd[i].kd = 1.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            
    

    def st_test(self):
        self.mode = 'test'


    def get_current_state(self):
        for i in range(NUM_MOTORS):
            self.qpos[i] = self.low_state.motor_state[i].q
            self.qvel[i] = self.low_state.motor_state[i].dq
            self.qtau[i] = self.low_state.motor_state[i].tau_est
        
        for i in range(3):
            self.ang_vel[i] = self.low_state.imu_state.gyroscope[i]

        for i in range(4):
            self.quat[i] = self.low_state.imu_state.quaternion[i]
    


    def LowCmdWrite(self):
        
        while self.is_running:
            step_start = time.perf_counter()
            if self.mode == 'test':
                self.test()
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher.Write(self.low_cmd)

            time_until_next_step = self.dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        self.ResetParam()
    
        
    def ResetParam(self):
        self.controller_rt = 0
        self.is_running = False


if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(1, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo") # default DDS port for pineapple

    controller = Controller()
    controller.Init()

    command_dict = {
        "test": controller.st_test,
    }

    while True:        
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                controller.ShutDown()
                break

        except Exception as e:
            traceback.print_exc()
            break
    sys.exit(-1)   