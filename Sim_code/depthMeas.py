"""
Author  : Zi Tao Li
Date    : March 30, 2025
Description: This file is design to sense the depth of the ball by calculating 
            the distance by using the camera and radius of the ball.
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

def Map_scan(robot):
    angleCounter = 0
    # Run simulation
    for i in range(1000000):
        p.stepSimulation()
        
        if (i % (int(240*(1/240)*20)) == 0): # every 20 seconds
            
            if angleCounter >= 360:
                # don't overlap the scan
                return 
            
            frame = hf.getCamera(robot)  # Get camera image and update the view, this will be used for image processing
            # mask, offset, annotated_img = hf.findBall(frame)
            pos, orn = p.getBasePositionAndOrientation(robot.id)
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]  # radians
            robot_heading = np.rad2deg(yaw) # convert to degrees
            robot_degree = hf.degreeIn360(robot_heading)
            
            # get the robot world coordinates
            robot_x = pos[0]
            robot_y = pos[1]
            balls_found = hf.measureBallDistance(frame, robot_degree, robot_x, robot_y)
            # print(f"Robot heading: {robot_heading:.2f} degrees, angle: {init_angle:.2f} degrees")
            
            if(balls_found != None):
                for ball in balls_found:
                    Globals.detected_balls.append(ball)
            
            # ball detection debug
            # cv2.imshow("Ball Detection", annotated_img)
            # cv2.waitKey(1)
            
            robot.right_for(60)
            angleCounter += 60
            
        
        
def print_balls(detected_balls):
    for i, ball in enumerate(detected_balls):
        x_local, y_local = ball['local_coord']
        x_world, y_world = ball['world_coord']
        print(f"Ball {i+1}: distance = {ball['distance']:.3f}, radius = {ball['radius']:.3f}, "
            f"local = ({x_local:.3f}, {y_local:.3f}), world = ({x_world:.3f}, {y_world:.3f})")
    
def extract_coords(detected_balls):
    ball_map = []
    for ball in detected_balls:
        x_world, y_world = ball['world_coord']
        ball_map.append((round(float(-x_world), 3), round(float(y_world), 3)))
    return ball_map


def Test():
    # Connect to physics server with GUI
    p.connect(p.GUI)
    
    # initialize 
    p.setGravity(0, 0, -9.8)
    Globals.debug_init()
    
    # Load plane and robot
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for default URDFs
    p.loadURDF("plane.urdf")
    robot = DifferentialDriveRobot("urdf/diff_drive.urdf", [0,0,0.1])
    # ab.addBalls_rand(10)
    
    # coordinates in (y, x) facing forward
    # ab.ballMeasTest()
    ab.fourCoordTest()

    # print joints
    robot.print_joints()
    robot.left_wheel_joint_index = 0
    robot.right_wheel_joint_index = 1
    
    init_angle = 0
    
    # Run simulation
    for i in range(1000000):
        p.stepSimulation()
        
        if i % (int(240*(1/240)*20)) == 0: # every 20 seconds
            frame = hf.getCamera(robot)  # Get camera image and update the view, this will be used for image processing
            # mask, offset, annotated_img = hf.findBall(frame)
            pos, orn = p.getBasePositionAndOrientation(robot.id)
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]  # radians
            robot_heading = np.rad2deg(yaw) # convert to degrees
            robot_degree = hf.degreeIn360(robot_heading)
            
            # get the robot world coordinates
            robot_x = pos[0]
            robot_y = pos[1]
            balls_found = hf.measureBallDistance(frame, robot_degree, robot_x, robot_y)
            # print(f"Robot heading: {robot_heading:.2f} degrees, angle: {init_angle:.2f} degrees")
            
            if(init_angle <= 360):
                if(balls_found != None):
                    for ball in balls_found:
                        Globals.detected_balls.append(ball)
                
                    for i, ball in enumerate(Globals.detected_balls):
                        x_local, y_local = ball['local_coord']
                        x_world, y_world = ball['world_coord']
                        print(f"Ball {i+1}: distance = {ball['distance']:.3f}, radius = {ball['radius']:.3f}, "
                            f"local = ({x_local:.3f}, {y_local:.3f}), world = ({x_world:.3f}, {y_world:.3f})")

            
            # # ball detection debug
            # cv2.imshow("Ball Detection", annotated_img)
            # cv2.waitKey(1)
            
            robot.right_for(60)
            init_angle += 60
        time.sleep(1/240)  # Slow it down to real-time

    # Keep window open
    input("Press Enter to exit...")

    # Disconnect when done
    p.disconnect()