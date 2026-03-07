from DM_CAN import *
import serial
import time

Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)

serial_device = serial.Serial('COM3', 921600, timeout=0.5)

Motor_Control = MotorControl(serial_device)
Motor_Control.addMotor(Motor1)

Motor_Control.set_zero_position(Motor1)
time.sleep(0.2)

print("Set Zero Position Successfully!")

# 语句结束关闭串口
serial_device.close()

