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
    hf.addBalls_rand(10)

    # print joints
    robot.print_joints()
    robot.left_wheel_joint_index = 0
    robot.right_wheel_joint_index = 1
    
    # initial pos for robot path tracing
    prev_pos = p.getBasePositionAndOrientation(robot.id)[0]  # starting position
    
    # Run simulation
    for i in range(1000000):
        p.stepSimulation()
        
        if i % 6 == 0:
            frame = hf.getCamera(robot)  # Get camera image and update the view, this will be used for image processing
            mask, offset, annotated_img = hf.findBall(frame)
            
            # ball detection debug
            cv2.imshow("Ball Detection", annotated_img)
            cv2.waitKey(1)
            
            if offset != None:
                threshold = 30
                if abs(offset) < threshold:
                    robot.forward_for(0.35)
                elif offset > threshold:
                    robot.right_for(5)
                else:
                    robot.left_for(5)
            else:
                robot.right_for(45) # turn to search for balls
                
        curr_pos = hf.drawPath(robot, prev_pos)
        prev_pos = curr_pos      
            
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
    
def depthMeas():
    # Connect to physics server with GUI
    p.connect(p.GUI)
    
    # initialize 
    p.setGravity(0, 0, -9.8)
    Globals.debug_init()
    
    # Load plane and robot
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for default URDFs
    p.loadURDF("plane.urdf")
    robot = DifferentialDriveRobot("urdf/diff_drive.urdf", [0,0,0.1])
    # hf.addBalls_rand(10)
    
    # coordinates in (y, x) facing forward
    hf.addBalls_at(0.458258, -0.2) # radius 0.5
    hf.addBalls_at(0.95394, 0.3) # radius 1
    hf.addBalls_at(1.48661, -0.2) # radius 1.5
    hf.addBalls_at(1.98997, 0.2) # radius 2
    hf.addBalls_at(2.498, -0.1) # radius 2.5
    hf.addBalls_at(2.99833, 0.1) # radius 3

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
            _, orn = p.getBasePositionAndOrientation(robot.id)
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]  # radians
            robot_heading = np.rad2deg(yaw) # convert to degrees
            balls_found = hf.measureBallDistance(frame, robot_heading)
            # print(f"Robot heading: {robot_heading:.2f} degrees, angle: {init_angle:.2f} degrees")
            
            if(init_angle <1):
                if(balls_found != None):
                    for ball in balls_found:
                        Globals.detected_balls.append(ball)
            
            if(init_angle == 0):
                for i, ball in enumerate(Globals.detected_balls):
                    print(f"Ball {i+1}: distance = {ball['distance']:.3f}, radius = {ball['radius']:.3f}, camera_offset_x = ={ball['camera_offset_x']:.3f}, lateral_offset_x = {ball['lateral_offset_x']:.3f}")

            
            # # ball detection debug
            # cv2.imshow("Ball Detection", annotated_img)
            # cv2.waitKey(1)
            
            # robot.right_for(45)
            init_angle += 45
        time.sleep(1/240)  # Slow it down to real-time

    # Keep window open
    input("Press Enter to exit...")

    # Disconnect when done
    p.disconnect()


if __name__ == "__main__":
    # main()
    depthMeas()

