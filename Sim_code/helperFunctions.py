
"""
Author  : Zi Tao Li
Date    : March 26, 2025
Description: This file contains all of the helper functions 
"""

import Globals
import pybullet as p
import numpy as np
import cv2
import random
import math


def checkContact(robot, balls):
    for i in range(len(balls)):
        ball_contacts = p.getContactPoints(bodyA=robot.id, bodyB=balls[i])
        
        if ball_contacts:
            print("Ball touched!")
            p.removeBody(balls[i])
            Globals.score += 1
        
    return Globals.score
    
def slider_control(robot):
    lw_speed = p.readUserDebugParameter(Globals.left_wheel_slider)
    rw_speed = p.readUserDebugParameter(Globals.right_wheel_slider)

    # update wheel speed 
    robot.wheel_control(robot.left_wheel_joint_index, p.VELOCITY_CONTROL, lw_speed, Globals.initial_force)
    robot.wheel_control(robot.right_wheel_joint_index, p.VELOCITY_CONTROL, rw_speed, Globals.initial_force)
    
def getCamera(robot):
    # get robot position
        try:
            pos, orn = p.getBasePositionAndOrientation(robot.id)
        except p.error as e:
            print("Error in getting position:", e)
            return None
        
        rot_matrix = p.getMatrixFromQuaternion(orn) # orientation is in quaternion angle
        camera_forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
        
        # Place camera on the scanner position
        # Set eye slightly forward and above the robot
        camera_eye = [pos[0] + 0.1 * camera_forward[0],
                    pos[1] + 0.1 * camera_forward[1],
                    pos[2] + 0.1]  # slightly above base

        # Look ahead in same direction
        camera_target = [camera_eye[0] + camera_forward[0],
                        camera_eye[1] + camera_forward[1],
                        camera_eye[2] + camera_forward[2]-0.25]


        view_matrix = p.computeViewMatrix(cameraEyePosition=camera_eye,
                                        cameraTargetPosition=camera_target,
                                        cameraUpVector=[0, 0, 1])
        
        proj_matrix = p.computeProjectionMatrixFOV(Globals.fov, Globals.aspect, Globals.near, Globals.far)
        
        # Get camera image
        _, _, rgb_img, _, _ = p.getCameraImage(
            width=Globals.width,
            height=Globals.height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix
        )
        
        # Convert to numpy array
        frame = np.reshape(rgb_img, (Globals.height, Globals.width, 4)).astype(np.uint8)  # 4 = RGBA
        frame = frame[:, :, :3]  # Drop alpha channel
        return frame
        
def findBall(frame):
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    lower_ball = np.array([12, 200, 140])
    upper_ball = np.array([18, 255, 255])

    mask = cv2.inRange(hsv, lower_ball, upper_ball)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pixel_offset = None
    output_img = frame_bgr.copy()

    if contours:
        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)

        if radius > 0.5:
            ball_center = (int(x), int(y))
            cv2.circle(output_img, ball_center, int(radius), (0, 255, 0), 2)

            img_center_x = frame.shape[1] // 2
            pixel_offset = ball_center[0] - img_center_x

            angle_offset = (pixel_offset / (frame.shape[1] / 2)) * (Globals.fov / 2)

    return mask, angle_offset, output_img

# return the nearest ball coordinate and distance in the given frame 
def getBallCoordinat(frame, heading, robot_x, robot_y):
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    lower_ball = np.array([12, 200, 140])
    upper_ball = np.array([18, 255, 255])

    mask = cv2.inRange(hsv, lower_ball, upper_ball)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)

        if radius > 0.5:
            ball_center = (x, y)
            camera_offset_px = ball_center[0] - (frame.shape[0] // 2)  # offset from center of the image
            offset_y = ball_center[1] - (frame.shape[1] // 2)  # offset from center of the image
            
            distance = calc_dist(radius)  # Euclidean distance from center
            lateral_offset_x = estimate_lateral_offset(distance, camera_offset_px)
            y_coord = np.sqrt(distance**2 - lateral_offset_x**2)  # y coordinate in the robot's frame
            
            heading_rad = np.deg2rad(heading)
            x_local = lateral_offset_x
            y_local = y_coord
            printHeading(heading)
            print("found ball at local coord: ", (x_local, y_local))
            # print("robot is now at world coord: ", (robot_x, robot_y))
            
            x_world = robot_x + (x_local * np.cos(heading_rad)) - (y_local * np.sin(heading_rad))
            y_world = robot_y + (x_local * np.sin(heading_rad)) + (y_local * np.cos(heading_rad))

            return (x_world, y_world)


def measureBallDistance(frame, heading, robot_x, robot_y):
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    lower_ball = np.array([12, 200, 140])
    upper_ball = np.array([18, 255, 255])

    mask = cv2.inRange(hsv, lower_ball, upper_ball)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    
    printHeading(heading)
    if contours:
        # if a ball is detected, collect its position relative to the robot and its size
        detected_balls = []
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius > 0.5: 
                ball_center = (x, y)
                camera_offset_px = ball_center[0] - (frame.shape[1] // 2)  # offset from center of the image
                offset_y = ball_center[1] - (frame.shape[0] // 2)  # offset from center of the image
                
                angle = np.arctan2(offset_y, camera_offset_px)
                angle_deg = np.rad2deg(angle)  # convert to degrees
                # Adjust angle based on robot heading
                angle_deg = (angle_deg + heading) % 360
                
                distance = calc_dist(radius)  # Euclidean distance from center
                lateral_offset_x = estimate_lateral_offset(distance, camera_offset_px)
                y_coord = np.sqrt(distance**2 - lateral_offset_x**2)  # y coordinate in the robot's frame
                
                heading_rad = np.deg2rad(heading)
                x_local = lateral_offset_x
                y_local = y_coord
                
                x_world = robot_x + (x_local * np.cos(heading_rad)) - (y_local * np.sin(heading_rad))
                y_world = robot_y + (x_local * np.sin(heading_rad)) + (y_local * np.cos(heading_rad))

                detected_balls.append({
                    'distance': distance,
                    'radius': radius,
                    'camera_offset_px': camera_offset_px,
                    'local_coord': (x_local, y_local),  # y coordinate in the robot's frame
                    'world_coord': (round(float(x_world), 3), round(float(y_world), 3))  # world coordinates
                })
        return detected_balls
    else:
        return None
    

def calc_dist(radius):
    # the contant k is the inverse proportionality constant for the distance calculation
    a = 7.2622
    b = 1.6356
    distance = a / (radius + b)
    
    return distance

def drawPath(robot, prev_pos):
    # Get current position
    curr_pos = p.getBasePositionAndOrientation(robot.id)[0]
    
    z_offset = 0.05  # or 0.1 meters above ground

    # Adjust positions
    prev_pos_with_offset = (prev_pos[0], prev_pos[1], prev_pos[2] + z_offset)
    curr_pos_with_offset = (curr_pos[0], curr_pos[1], curr_pos[2] + z_offset)

    # Draw a line from previous to current
    p.addUserDebugLine(prev_pos_with_offset, curr_pos_with_offset,
                    lineColorRGB=[1, 0.5, 0],  # orange
                    lineWidth=1.0,            # thicker line
                    lifeTime=0)               # stays forever

    return curr_pos  # return current position for update

def estimate_lateral_offset(distance, camera_offset_px, image_width=320, fov_deg=Globals.fov):
    fov_rad = np.radians(fov_deg)
    angle_per_pixel = fov_rad / image_width
    angle = camera_offset_px * angle_per_pixel
    lateral_offset = np.tan(angle) * distance
    return lateral_offset

def degreeIn360(angle):
    return (angle + 360) % 360

def toCompassHeading(angle):
    return (360 - angle + 90) % 360

def printHeading(angle):
    heading = toCompassHeading(angle)
    print(f"Heading: {heading:.2f} degrees")

def ContactWrapper(robot):
    if (Globals.score != checkContact(robot, Globals.balls)):
            # score changed so update score 
            p.removeUserDebugItem(Globals.score_text_id)  
            Globals.score_text_id = p.addUserDebugText(f"Collected: {Globals.score}", [0, 0, 1.5], textSize=2, lifeTime=0)
        
