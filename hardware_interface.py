import numpy as np
import traceback
import time
import threading 
import usb.core
import usb.util

from damiao import *
from xsens_imu_test import *

from csl_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from csl_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from csl_sdk2py.idl.default import csl_pineapple_msg_dds__LowCmd_
from csl_sdk2py.idl.default import csl_pineapple_msg_dds__LowState_
from csl_sdk2py.idl.csl_pineapple.msg.dds_ import LowCmd_
from csl_sdk2py.idl.csl_pineapple.msg.dds_ import LowState_
from csl_sdk2py.utils.crc import CRC
from csl_sdk2py.utils.thread import RecurrentThread

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"

DEVICE_ID = "8618B478D60114596A72552FA077E579"

MOTOR_ID_LIST = [(DM_Motor_Type.DM8006, 0x01, 0x11), (DM_Motor_Type.DM8006, 0x03, 0x13), 
                 (DM_Motor_Type.DM8009, 0x05, 0x15), (DM_Motor_Type.DM8009, 0x06, 0x16)]
NUM_MOTOR = 4

class DDSHandler:
    def __init__(self, domain_id=1, interface="lo"):
        ChannelFactoryInitialize(domain_id, interface)


        self.motor_list = list()
        self.is_running = False
        self.dt = 0.001

        self.init_motor_manager()
        self.body_imu = XsensIMU()

        self.low_state_publisher = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_publisher.Init()
        self.low_state = csl_pineapple_msg_dds__LowState_()
        self.low_state_pub_thread = threading.Thread(target=self.low_state_handler, daemon=True)
        self.low_state_pub_thread.start()

        self.low_cmd_subscriber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_subscriber.Init(self.low_cmd_handler, 10)
    
    def init_motor_manager(self):
        for (motor_type, can_id, mst_id) in MOTOR_ID_LIST:
            self.motor_list.append(DmActData(
                    motorType=motor_type,  
                    mode=Control_Mode.MIT_MODE,   
                    can_id=can_id,
                    mst_id=mst_id))
        
        self.motor_manager = Motor_Control(1000000, 5000000,DEVICE_ID,self.motor_list) 
    
    
    def low_state_handler(self):
        while True:
            step_start = time.perf_counter()
            for i in range(NUM_MOTOR):
                self.low_state.motor_state[i].q = self.motor_manager.getMotor(self.motor_list[i].can_id).Get_Position()
                self.low_state.motor_state[i].dq = self.motor_manager.getMotor(self.motor_list[i].can_id).Get_Velocity()
                self.low_state.motor_state[i].tau_est = self.motor_manager.getMotor(self.motor_list[i].can_id).Get_tau()

                # print(f"canid is: {self.motor_list[i]} pos: {self.low_state.motor_state[i].q} vel: {self.low_state.motor_state[i].dq} effort: {self.low_state.motor_state[i].tau_est} ", file=sys.stderr)

            # pub body imu quatanion data
            quaternion = self.body_imu.get_quaternion()
            angular_vel = self.body_imu.get_angular_vel()
            for i in range(4):
                self.low_state.imu_state.quaternion[i] = quaternion[i]
            
            for i in range(3):
                self.low_state.imu_state.gyroscope[i] = angular_vel[i]

            self.low_state_publisher.Write(self.low_state)
            time_until_next_step = self.dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    
    def low_cmd_handler(self, msg:LowCmd_):
        self.low_cmd = msg
        if self.low_cmd is not None:
            for i in range(NUM_MOTOR):
                self.motor_manager.control_mit(self.motor_manager.getMotor(self.motor_list[i].can_id), 
                                               self.low_cmd.motor_cmd[i].kp, self.low_cmd.motor_cmd[i].kd,
                                               self.low_cmd.motor_cmd[i].q, self.low_cmd.motor_cmd[i].dq, self.low_cmd.motor_cmd[i].tau)

    
    def close(self):
        self.is_running = False
        self.motor_manager.close()


if __name__ == '__main__':
    
    hardware_manager = DDSHandler()
    command_dict = {
        # "test": hardware_manager.test,
    }

    while True:        
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                hardware_manager.close()
                break

        except Exception as e:
            traceback.print_exc()
            break
    sys.exit(-1)