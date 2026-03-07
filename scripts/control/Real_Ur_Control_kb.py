import rtde_control
import rtde_receive
import numpy as np
import cv2
import queue
import threading
import time  # 导入时间模块，避免不必要的高 CPU 占用
import keyboard


def rotation_vector_to_rpy(rotation_vector):
    # Ensure rotation_vector is a numpy array with the correct shape
    rotation_vector = np.array(rotation_vector).reshape(3, 1)
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


def move_step(rtde_C, current_pose, direction, step_size=0.01, time_duration=0.1, lookahead_time=0.2, gain=100):
    """
    Move the robot arm one step in the specified linear direction
    
    Args:
        rtde_C: RTDE control interface
        current_pose: Current pose [x, y, z, rx, ry, rz]
        direction: 'up', 'down', 'left', 'right', 'forward', 'backward'
        step_size: Step size for movement (default: 0.01)
    
    Returns:
        new_pose: Updated pose after movement
    """
    new_pose = current_pose.copy()
    
    if direction == 'up':
        new_pose[2] += step_size  # Z axis
        axis = 'Z'
    elif direction == 'down':
        new_pose[2] -= step_size  # Z axis
        axis = 'Z'
    elif direction == 'left':
        new_pose[0] -= step_size  # X axis
        axis = 'X'
    elif direction == 'right':
        new_pose[0] += step_size  # X axis
        axis = 'X'
    elif direction == 'forward':
        new_pose[1] += step_size  # Y axis
        axis = 'Y'
    elif direction == 'backward':
        new_pose[1] -= step_size  # Y axis
        axis = 'Y'
    else:
        print(f"Invalid direction: {direction}")
        return current_pose
    
    rtde_C.servoL(new_pose, 0.1, 0.08, time_duration, lookahead_time, gain)
    
    # Print the appropriate axis value
    if axis == 'X':
        print(f"Moved {direction}: X = {new_pose[0]:.3f}")
    elif axis == 'Y':
        print(f"Moved {direction}: Y = {new_pose[1]:.3f}")
    elif axis == 'Z':
        print(f"Moved {direction}: Z = {new_pose[2]:.3f}")
    
    return new_pose


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
    init_pose = [0.43, -0.11, 0.40, 
                -1.1113946900002296, 2.9307050851101204, -0.07006688172175954]
    rtde_C.servoL(init_pose, 0.1, 0.08, time_duration, lookahead_time, gain)
    
    return rtde_C, init_pose, time_duration, lookahead_time, gain

    
if __name__ == "__main__":
    rtde_C, current_pose, time_duration, lookahead_time, gain = UR_control()
    print("Keyboard control started!")
    print("Controls:")
    print("Q/E: Up/Down (Z axis)")
    print("A/D: Left/Right (X axis)")  
    print("W/S: Forward/Backward (Y axis)")
    print("ESC: Quit")
    print(f"Starting position: X={current_pose[0]:.3f}, Y={current_pose[1]:.3f}, Z={current_pose[2]:.3f}")
    
    # Buffer variables to prevent consecutive steps
    last_command_time = 0
    command_buffer_time = 0.2  # Minimum time between commands (seconds)
    
    try:
        while True:
            current_time = time.time()
            
            # Z axis controls (up/down)
            if keyboard.is_pressed('q') and (current_time - last_command_time) > command_buffer_time:
                current_pose = move_step(rtde_C, current_pose, 'up', 0.01, time_duration, lookahead_time, gain)
                last_command_time = current_time
            elif keyboard.is_pressed('e') and (current_time - last_command_time) > command_buffer_time:
                current_pose = move_step(rtde_C, current_pose, 'down', 0.01, time_duration, lookahead_time, gain)
                last_command_time = current_time
            # X axis controls (left/right)
            elif keyboard.is_pressed('a') and (current_time - last_command_time) > command_buffer_time:
                current_pose = move_step(rtde_C, current_pose, 'left', 0.01, time_duration, lookahead_time, gain)
                last_command_time = current_time
            elif keyboard.is_pressed('d') and (current_time - last_command_time) > command_buffer_time:
                current_pose = move_step(rtde_C, current_pose, 'right', 0.01, time_duration, lookahead_time, gain)
                last_command_time = current_time
            # Y axis controls (forward/backward)
            elif keyboard.is_pressed('w') and (current_time - last_command_time) > command_buffer_time:
                current_pose = move_step(rtde_C, current_pose, 'forward', 0.01, time_duration, lookahead_time, gain)
                last_command_time = current_time
            elif keyboard.is_pressed('s') and (current_time - last_command_time) > command_buffer_time:
                current_pose = move_step(rtde_C, current_pose, 'backward', 0.01, time_duration, lookahead_time, gain)
                last_command_time = current_time
            elif keyboard.is_pressed('esc'):
                print("Exiting...")
                break
            
            time.sleep(0.01)  # Small delay to prevent high CPU usage
            
    except KeyboardInterrupt:
        print("\nProgram interrupted. Exiting...")

