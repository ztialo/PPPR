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
    b1 = p.loadURDF("urdf/ping_pong.urdf", [0.8,0.5,0.05])
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
        
        # if i > 50 and run_flag == 0:
        #     print("Running...")
        #     robot.forward_for(1)
            
        #     robot.wait(500) # wait for 500 ms (0.5s)
            
        #     robot.right_for(180)
            
        #     # robot.forward_for(1)
        #     # robot.left_for(180)
        #     run_flag = 1
        
        if i % 5 == 0:
            frame = hf.getCamera(robot)  # Get camera image and update the view, this will be used for image processing
            mask, offset, annotated_img = hf.findBall(frame)
            
            # ball detection debug
            cv2.imshow("Ball Detection", annotated_img)
            cv2.waitKey(1)
            
            if offset != None:
                threshold = 20
                if abs(offset) < threshold:
                    robot.forward_for(0.25)
                elif offset > threshold:
                    robot.right_for(5)
                else:
                    robot.left_for(5)

            
            
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

