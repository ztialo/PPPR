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
import addBalls as ab
import depthMeas
import pathPlanning

def main():
    # Connect to physics server with GUI
    p.connect(p.GUI)
    # initialize
    p.setGravity(0, 0, -9.8)
    # Globals.debug_init()

    # Load plane and robot
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for default URDFs
    p.loadURDF("plane.urdf")

    # red is x axis, green is y axis
    robot = DifferentialDriveRobot("URDF/diff_drive.urdf", [0,0,0.1])  
    # ab.addBalls_rand(520)
    ab.fourCoordTest()
    # ab.tenBallTest()

    # print joints
    robot.print_joints()
    robot.left_wheel_joint_index = 0
    robot.right_wheel_joint_index = 1
    
    testFlag = 1
    # Run simulation
    for i in range(1000000):
        p.stepSimulation()
        
        if i == 30:
            depthMeas.Map_scan(robot)
            world_map = []
            # depthMeas.print_balls(Globals.detected_balls)
            world_map = depthMeas.extract_coords(Globals.detected_balls)
            
            # add currrent position of the robot to the world_map
            robot_pos = p.getBasePositionAndOrientation(robot.id)[0]
            world_map.append((round(robot_pos[0], 3), round(robot_pos[1], 3)))
            print("world map: ", world_map)
    
            path = pathPlanning.NearestNeighbor(world_map)
            print("path:", path)
            robot.followPath(path)       

        # constantly checks for ball 1 contact
        hf.ContactWrapper(robot)
        
        time.sleep(1/240)  # Slow it down to real-time

    # Keep window open
    input("Press Enter to exit...")

    # Disconnect when done
    p.disconnect()


if __name__ == "__main__":

    main()
    # depthMeas.Test()

