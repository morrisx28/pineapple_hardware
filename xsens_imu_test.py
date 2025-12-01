from xsens_imu.SerialHandler import SerialHandler
from xsens_imu.XbusPacket import XbusPacket
from xsens_imu.DataPacketParser import DataPacketParser, XsDataPacket
from xsens_imu.SetOutput import set_output_conf
import time
from xsens_imu.data_logging import save_data_to_csv
from datetime import datetime
import threading
import traceback
import sys

class XsensIMU:
    def __init__(self, port="/dev/ttyUSB0", b_rate=921600):
        self.serial = SerialHandler(port_name=port, baud_rate=b_rate)
        go_to_config = bytes.fromhex('FA FF 30 00')
        go_to_measurement = bytes.fromhex('FA FF 10 00')

        self.serial.send_with_checksum(go_to_config)

        self.rate_of_turn_degree = [0, 0, 0]
        self.quaternion = [0, 0, 0, 0]
        ###if you want to configure your sensor's output, check the set_output_conf function.
        ##set_output_conf(serial)
        time.sleep(0.1)  # Sleep for 0.1 sec
        
        self.packet = XbusPacket(on_data_available=lambda p: self.on_live_data_available(p))
        
        
        self.serial.send_with_checksum(go_to_measurement)
        print("Listening for packets...")

        self.imu_data_thread = threading.Thread(target=self.process_imu_data, daemon=True)
        self.imu_data_thread.start()
    
    def process_imu_data(self):
        while True:
            try:
                byte = self.serial.read_byte()
                self.packet.feed_byte(byte)
            except RuntimeError as e:
                print(f"Error reading byte: {e}")
                continue  # Skip to the next byte
    
    def get_angular_vel(self):
        return self.rate_of_turn_degree

    def get_quaternion(self):
        return self.quaternion

    def on_live_data_available(self, packet):
        xbus_data = XsDataPacket() 
        DataPacketParser.parse_data_packet(packet, xbus_data)

        # if xbus_data.packetCounterAvailable:
        #     print(f"\npacketCounter: {xbus_data.packetCounter}, ", end='')

        # if xbus_data.sampleTimeFineAvailable:
        #     print(f"sampleTimeFine: {xbus_data.sampleTimeFine}, ", end='')

        # if xbus_data.utcTimeAvailable:
        #     print(f"utctime epochSeconds: {xbus_data.utcTime:.9f}")

        # if xbus_data.eulerAvailable:
        #     print(f"\nRoll, Pitch, Yaw: [{xbus_data.euler[0]:.2f}, {xbus_data.euler[1]:.2f}, {xbus_data.euler[2]:.2f}], ", end='')

        if xbus_data.quaternionAvailable:
            # w x y z
            for i in range(4):
                self.quaternion[i] = xbus_data.quat[i]
            # print(f"q0, q1, q2, q3: [{xbus_data.quat[0]:.4f}, {xbus_data.quat[1]:.4f}, {xbus_data.quat[2]:.4f}, {xbus_data.quat[3]:.4f}], ", end='')

        if xbus_data.rotAvailable:
            self.rate_of_turn_degree = [
                xbus_data.rad2deg * xbus_data.rot[0],
                xbus_data.rad2deg * xbus_data.rot[1],
                xbus_data.rad2deg * xbus_data.rot[2]
            ]
            # print(f"\nRateOfTurn: [{self.rate_of_turn_degree[0]:.2f}, {self.rate_of_turn_degree[1]:.2f}, {self.rate_of_turn_degree[2]:.2f}], ", end='')

        # if xbus_data.accAvailable:
        #     print(f"Acceleration: [{xbus_data.acc[0]:.2f}, {xbus_data.acc[1]:.2f}, {xbus_data.acc[2]:.2f}], ", end='')

        # if xbus_data.magAvailable:
        #     print(f"Magnetic Field: [{xbus_data.mag[0]:.2f}, {xbus_data.mag[1]:.2f}, {xbus_data.mag[2]:.2f}]")

        # if xbus_data.latlonAvailable and xbus_data.altitudeAvailable:
        #     print(f"\nLat, Lon, Alt: [{xbus_data.latlon[0]:.9f}, {xbus_data.latlon[1]:.9f}, {xbus_data.altitude:.9f}]")

        # if xbus_data.velocityAvailable:
        #     print(f"Vel E, N, U: [{xbus_data.vel[0]:.9f}, {xbus_data.vel[1]:.9f}, {xbus_data.vel[2]:.9f}]")
        
        # if xbus_data.temperatureAvailable:
        #     print(f"Temperature: {xbus_data.temperature:.2f}")
        
        # if xbus_data.baropressureAvailable:
        #     print(f"Barometric Pressure: {xbus_data.baropressure} Pa")
        
        # if xbus_data.deltaVAvailable:
        #     print(f"Delta V: [{xbus_data.deltaV[0]:.2f}, {xbus_data.deltaV[1]:.2f}, {xbus_data.deltaV[2]:.2f}]")
        
        # if xbus_data.deltaQAvailable:
        #     print(f"Delta Q: [{xbus_data.deltaQ[0]:.4f}, {xbus_data.deltaQ[1]:.4f}, {xbus_data.deltaQ[2]:.4f}, {xbus_data.deltaQ[3]:.4f}]")
        
        # save_data_to_csv(xbus_data, filename, log_position_velocity=False)


if __name__ == '__main__':
    
    imu = XsensIMU()

    command_dict = {

    }

    while True:        
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break

        except Exception as e:
            traceback.print_exc()
            break
    sys.exit(-1)