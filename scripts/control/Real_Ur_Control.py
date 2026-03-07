import rtde_control
import rtde_receive
import numpy as np
import cv2
import queue
import threading
import time  # 导入时间模块，避免不必要的高 CPU 占用


def rotation_vector_to_rpy(rotation_vector):
    R, _ = cv2.Rodrigues(rotation_vector)

    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    roll = np.arctan2(R[2, 1], R[2, 2])

    return roll, pitch, yaw

# 新增：欧拉角转旋转向量
def rpy_to_rotation_vector(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    R = R_z @ R_y @ R_x
    rotation_vector, _ = cv2.Rodrigues(R)
    return rotation_vector.flatten()


def UR_control():
    robot_ip = "192.168.3.4"
    time_duration = 0.1
    lookahead_time = 0.2
    gain = 100

    # Initialize RTDE interfaces for receiving and controlling the robot
    rtde_R = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_C = rtde_control.RTDEControlInterface(robot_ip)

    # Get initial pose and orientation
    initial_pose = rtde_R.getActualTCPPose()
    initial_q = rtde_R.getActualQ()
    print("111")
    print(f"Initial pose: {initial_pose}")


    # 让末端Z轴垂直向下（roll=0, pitch=π, yaw=0）
    roll = 0
    pitch = np.pi
    yaw = 0
    rotation_vector = rpy_to_rotation_vector(roll, pitch, yaw)
    # new_pose = [0.4308283576912434, -0.11271855238218005, 0.40, 
    #             -1.1113946900002296, 2.9307050851101204, -0.07006688172175954]
    # rtde_C.servoL(new_pose, 0.1, 0.08, time_duration, lookahead_time, gain)

    
if __name__ == "__main__":
    while True:
        UR_control()
        time.sleep(1)  # Pause before retrying if an exception occurs
