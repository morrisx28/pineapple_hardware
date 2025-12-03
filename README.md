# pineapple_hardware_interface
Python interface for pineapple version 2
Xsens imu sensor repo refer to https://github.com/jiminghe/Xsens_MTi_Serial_Reader.git
Damiao motor CANFD repo refer to https://gitee.com/kit-miao/damiao.git

# Installation
## Dependencies
- Python >= 3.8
- cyclonedds == 0.10.2
- numpy
- opencv-python
## Install csl_sdk2_python

Execute the following commands in the terminal:
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/morrisx28/csl_sdk2_python.git
cd csl_sdk2_python
pip3 install -e .
```
## Set up usb2canfd
```bash
pip3 install pyusb
sudo nano /etc/udev/rules.d/99-usb.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666" ## Insert following line
sudo udevadm control --reload-rules
sudo udevadm trigger
```


