import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import lib.pineapple_multi_imu as imu
import time

imu1 = imu.ImuRs485Worker("/dev/ttyUSB0")
imu2 = imu.ImuRs485Worker("/dev/ttyUSB1")
imu1.start()
imu2.start()
time.sleep(1.0)
t0 = time.time()
count = 0
last_gyro = None
for i in range(5000):
    imu1_data = imu1.snapshot()
    print("quat1:", imu1_data.quaternion)
    print("rpy1:", imu1_data.rpy)
    print("gyro1:", imu1_data.gyro)
    print("accel1:", imu1_data.accel)

    imu2_data = imu2.snapshot()
    print("quat2:", imu2_data.quaternion)
    print("rpy2:", imu2_data.rpy)
    print("gyro2:", imu2_data.gyro)
    print("accel2:", imu2_data.accel)

    time.sleep(1/500)
imu1.stop()
imu2.stop()