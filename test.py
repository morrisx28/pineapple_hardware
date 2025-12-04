import time
import sys
import numpy as np
import threading
import traceback
import yaml
import csv
import argparse
import matplotlib.pyplot as plt # Import for plotting

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

        # Data logging lists for plotting (for motor 0 as an example)
        self.time_data = []
        self.qpos_data = []
        self.qvel_data = []
        self.qtau_data = []
        self.dq_cmd_data = []
        self.tau_cmd_data = []


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
        self.start_time = time.perf_counter() # To calculate elapsed time


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
        self.start_time = time.perf_counter() # Reset start time after threads are initialized

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
        if self.lowCmdWriteThreadPtr:
            self.lowCmdWriteThreadPtr.join()


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
        self.record_data() # Record data when a new state message is received
        # print(f'qpos {self.low_state.motor_state[0].q}')
        # quat = self.low_state.imu_state.quaternion
        # ang_vel = self.low_state.imu_state.gyroscope
        # print(f'quat w: {quat[0]} x: {quat[1]} y: {quat[2]} z: {quat[3]}')
        # print(f'ang_vel x: {ang_vel[0]} y: {ang_vel[1]} z: {ang_vel[2]}')
    
    
    def test(self):
        for i in range(NUM_MOTORS):
            self.low_cmd.motor_cmd[i].q = 0
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = 1.0 # Command velocity of 1.0
            self.low_cmd.motor_cmd[i].kd = 1.0
            self.low_cmd.motor_cmd[i].tau = 0.0
    
    def test1(self):
        for i in range(NUM_MOTORS):
            self.low_cmd.motor_cmd[i].q = 0
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = -1.0 # Command velocity of -1.0
            self.low_cmd.motor_cmd[i].kd = 1.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            
    

    def st_test(self):
        self.mode = 'test'
        print("Mode set to 'test': Target motor velocity 1.0")
    
    def st_test1(self):
        self.mode = 'test1'
        print("Mode set to 'test1': Target motor velocity -1.0")


    def get_current_state(self):
        for i in range(NUM_MOTORS):
            self.qpos[i] = self.low_state.motor_state[i].q
            self.qvel[i] = self.low_state.motor_state[i].dq
            self.qtau[i] = self.low_state.motor_state[i].tau_est
        
        for i in range(3):
            self.ang_vel[i] = self.low_state.imu_state[0].gyroscope[i]
        
        # print("angular vel: ", self.ang_vel)

        for i in range(4):
            self.quat[i] = self.low_state.imu_state[0].quaternion[i]
    

    def record_data(self):
        """Records current state and command data for plotting."""
        # Use motor 0 for plotting as an example
        motor_idx = 0 
        
        current_time = time.perf_counter() - self.start_time
        
        self.time_data.append(current_time)
        self.qpos_data.append(self.qpos[motor_idx])
        self.qvel_data.append(self.qvel[motor_idx])
        self.qtau_data.append(self.qtau[motor_idx])
        
        # Record command for the active motor
        self.dq_cmd_data.append(self.low_cmd.motor_cmd[motor_idx].dq)
        self.tau_cmd_data.append(self.low_cmd.motor_cmd[motor_idx].tau)

    def plot_data(self):
        """Plots the recorded state and command data."""
        if not self.time_data:
            print("No data recorded to plot.")
            return

        print("Generating plots...")

        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle('Motor 0 State and Command Over Time')

        # 1. Position Plot
        axs[0].plot(self.time_data, self.qpos_data, label='Actual Position (q)')
        axs[0].set_ylabel('Position (rad)')
        axs[0].grid(True)
        axs[0].legend()

        # 2. Velocity Plot (Actual vs Command)
        axs[1].plot(self.time_data, self.qvel_data, label='Actual Velocity (dq)', color='blue')
        axs[1].plot(self.time_data, self.dq_cmd_data, label='Command Velocity (dq_cmd)', linestyle='--', color='orange')
        axs[1].set_ylabel('Velocity (rad/s)')
        axs[1].grid(True)
        axs[1].legend()

        # 3. Torque Plot (Estimated vs Command)
        axs[2].plot(self.time_data, self.qtau_data, label='Estimated Torque (tau_est)', color='green')
        axs[2].plot(self.time_data, self.tau_cmd_data, label='Command Torque (tau_cmd)', linestyle=':', color='red')
        axs[2].set_ylabel('Torque (Nm)')
        axs[2].set_xlabel('Time (s)')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96]) # Adjust layout to fit suptitle
        plt.show()


    def LowCmdWrite(self):
        
        while self.is_running:
            step_start = time.perf_counter()
            if self.mode == 'test':
                self.test()
            elif self.mode == 'test1':
                self.test1()
            
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
        "test1": controller.st_test1,
    }

    while True:        
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "plot":
                controller.plot_data() # New command to plot
            elif cmd == "exit":
                controller.ShutDown()
                break

        except Exception as e:
            traceback.print_exc()
            break
    sys.exit(-1)