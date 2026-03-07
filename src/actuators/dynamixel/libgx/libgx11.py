import time
from .dynamixel_sdk import PortHandler, PacketHandler
from .motor import Motor
import sys
import numpy as np
from .config import BAUDRATE, PROTOCOL_VERSION

class SSRSurgery:

    def __init__(self, port) -> None:
        self.is_connected = False
        self.port = port
        self.name = 'SSR-Surgery'


    def connect(self):
        """
        连接Hand，并且使能每个电机为默认的力控位置模式
        """

        portHandler = PortHandler(self.port)
        packetHandler = PacketHandler(PROTOCOL_VERSION)

        if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
            print(f'Open {self.port} Success...')
            self.is_connected = True
        else:
            print(f'Failed...')
            self.is_connected = False
            sys.exit(0)

        self.portHandler = portHandler
        self.packetHandler = packetHandler

        self.motors = [Motor(i + 1, portHandler, packetHandler) for i in range(2)]

        for m in self.motors:
            m.init_config()

        print(f'{self.name} init done...')

    def off(self):
        """
        失能所有电机
        """
        for m in self.motors:
            m.torq_off()

    def on(self):
        """
        使能所有电机
        """
        for m in self.motors:
            m.torq_on()

    def getj(self):
        """
        获取ISR关节角度，单位度
        """
        js = [m.get_pos() for m in self.motors]
        return js

    def setj(self, js):
        """
        设置ISR关节角度，单位度
        """
        for m, j in zip(self.motors, js):
            m.set_pos(j)

    def set_all_zero(self):
        """
        将所有电机的角度设置为零
        """
        for m in self.motors:
            m.set_zero()

    def Safe_control(self, angle):
        """
        直接设置电机角度，不进行安全限制
        """
        self.setj(angle)

        # # 定义安全范围
        # valid_ranges = [
        #     (0, 180),  # 第1个电机的角度范围
        #     (0, 180),  # 第2个电机的角度范围
        # ]

        # # 初始化上一个有效角度的存储（与电机数量相同）
        # last_valid_angles = [None] * len(self.motors)

        # # 获取当前电机的角度
        # current_angles = self.getj()

        # # 更新角度并检查是否在有效范围内
        # for i, current_angle in enumerate(current_angles):
        #     # 检查角度是否在预定义的有效范围内
        #     if valid_ranges[i][0] <= angle[i] <= valid_ranges[i][1]:
        #         # 如果在有效范围内，更新上一次有效角度，并设置当前角度
        #         last_valid_angles[i] = angle[i]
        #         current_angles[i] = angle[i]
        #     else:
        #         # 如果不在有效范围内，使用上一次有效角度，如果没有则使用当前角度
        #         current_angles[i] = last_valid_angles[i] if last_valid_angles[i] is not None else current_angle

        # # 设置更新后的角度
        # self.setj(current_angles)

    def get_angle(self):
      
        js = [m.get_pos() for m in self.motors]

        return js

