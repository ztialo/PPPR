"""
Author  : Zi Tao Li
Date    : March 25, 2025
Description: This file is the main running file for simulation
"""

import pybullet as p
import pybullet_data
import cv2
import time
import numpy as np
import Globals
import helperFunctions as hf
from JointControl import DifferentialDriveRobot

def main():
    # Connect to physics server with GUI
    p.connect(p.GUI)

    # initialize 
    p.setGravity(0, 0, -9.8)
    Globals.debug_init()

    # Load plane and robot
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for default URDFs
    p.loadURDF("plane.urdf")
    robot = DifferentialDriveRobot("urdf/diff_drive.urdf", [0,0,0.1])
    b1 = p.loadURDF("urdf/ping_pong.urdf", [0.8,0,0.2])
    Globals.balls.append(b1)

    # print joints
    robot.print_joints()
    robot.left_wheel_joint_index = 0
    robot.right_wheel_joint_index = 1

    # robot.forward(initial_speed, initial_force)

    run_flag = 0
    # Run simulation
    for i in range(1000000):
        p.stepSimulation()
        
        if i > 50 and run_flag == 0:
            print("Running...")
            robot.forward_for(1)
            run_flag = 1
        
        # # get robot position
        # pos, orn = p.getBasePositionAndOrientation(robot.id)
        
        # rot_matrix = p.getMatrixFromQuaternion(orn) # orientation is in quaternion angle
        # camera_forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
        
        # # Place camera on the scanner position
        # # Set eye slightly forward and above the robot
        # camera_eye = [pos[0] + 0.1 * camera_forward[0],
        #             pos[1] + 0.1 * camera_forward[1],
        #             pos[2] + 0.1]  # slightly above base

        # # Look ahead in same direction
        # camera_target = [camera_eye[0] + camera_forward[0],
        #                 camera_eye[1] + camera_forward[1],
        #                 camera_eye[2] + camera_forward[2]]


        # view_matrix = p.computeViewMatrix(cameraEyePosition=camera_eye,
        #                                 cameraTargetPosition=camera_target,
        #                                 cameraUpVector=[0, 0, 1])
        
        # proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        
        # # Get camera image
        # _, _, rgb_img, _, _ = p.getCameraImage(
        #     width=width,
        #     height=height,
        #     viewMatrix=view_matrix,
        #     projectionMatrix=proj_matrix
        # )
        
        # # Convert to numpy array
        # frame = np.reshape(rgb_img, (height, width, 4)).astype(np.uint8)  # 4 = RGBA
        # frame = frame[:, :, :3]  # Drop alpha channel
        
        # # read left wheel and right wheel speed input from GUI
        # if i > 100 and i % 100 == 0: # delay read time, make sure everything is initialize correctly first
        #     hf.slider_control(robot)
        
        # constantly checks for ball 1 contact
        if (Globals.score != hf.checkContact(robot, Globals.balls)):
            # score changed so update score 
            p.removeUserDebugItem(Globals.score_text_id)  
            Globals.score_text_id = p.addUserDebugText(f"Collected: {Globals.score}", [0, 0, 1.5], textSize=2, lifeTime=0)
        
        time.sleep(1/240)  # Slow it down to real-time

    # Keep window open
    input("Press Enter to exit...")

    # Disconnect when done
    p.disconnect()


if __name__ == "__main__":
    main()

