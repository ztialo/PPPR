"""
Author  : Zi Tao Li
Date    : March 25, 2025
Description: This file contains functions that controls the joint
                movements. 
"""
import pybullet as p
import numpy as np
import time
import Globals
import helperFunctions as hf
from PIDcontrol import PIDController
import cv2

class DifferentialDriveRobot:
    def __init__(self, urdf_path, start_pos):
        self.id = p.loadURDF("urdf/diff_drive.urdf", [0,0,0.01])
        self.left_wheel_joint_index = None
        self.right_wheel_joint_index = None
    

    def print_joints(self):
        num_joints = p.getNumJoints(self.id)
        for i in range(num_joints):
            print(i, p.getJointInfo(self.id, i)[1])
            
            
    def wheel_control(self, joint_index, mode, v, f):
        p.setJointMotorControl2(
            bodyUniqueId = self.id,
            jointIndex = joint_index,
            controlMode = mode,
            targetVelocity = v,  # Positive value = forward
            force = f
        )
        
    def forward(self, v, f):
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, v, f) # lw move forward at 50 m/s
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, v, f) # rw move forward
        
    def turn_left(self, v, f):
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, -v, f) # lw move forward at 50 m/s
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, v, f) # rw move forward
        
    def turn_right(self, v, f):
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, v, f) # lw 
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, -v, f *1.5) # rw move forward
    
    def backward(self, v, f):
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, -v, f) # lw move forward at 50 m/s
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, -v, f) # rw move forward
    
    def stop(self, brake_force):
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, 0, brake_force)
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, 0, brake_force)

    def forward_for(self, target_dist):
        # target_dist += 0.1  # slight overshoot to ensure ball collection

        start_pos, orn = p.getBasePositionAndOrientation(self.id)
        start_vec = np.array(start_pos)

        # Get initial heading vector and yaw angle
        init_rot_matrix = p.getMatrixFromQuaternion(orn)
        forward_vec = [init_rot_matrix[0], init_rot_matrix[3], init_rot_matrix[6]]
        target_yaw = p.getEulerFromQuaternion(orn)[2]

        # Distance PID
        dist_pid = PIDController(kp=100.0, ki=0.0, kd=5.0)
        
        # Yaw PID to correct heading during forward movement
        heading_pid = PIDController(kp=50.0, ki=0.0, kd=2.0)

        dt = 1 / 240
        prev_pos = start_pos

        for i in range(10000):
            p.stepSimulation()

            try:
                current_pos, orn = p.getBasePositionAndOrientation(self.id)
            except p.error as e:
                print("Error in getting position:", e)
                continue

            current_pos, orn = p.getBasePositionAndOrientation(self.id)
            current_vec = np.array(current_pos)

            # Distance traveled in the original heading direction
            delta = current_vec - start_vec
            moved_dist = np.dot(delta, forward_vec)
            dist_error = target_dist - moved_dist

            # Heading correction
            current_yaw = p.getEulerFromQuaternion(orn)[2]
            yaw_error = (target_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi

            # Compute forward speed and turning correction
            fwd_speed = np.clip(dist_pid.compute(dist_error, dt), -150, 150)
            turn_correction = heading_pid.compute(yaw_error, dt)
            turn_correction = np.clip(turn_correction, -10, 10)

            # Differential wheel speeds for heading correction
            left_speed = fwd_speed - turn_correction
            right_speed = fwd_speed + turn_correction

            self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, left_speed, 10)
            self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, right_speed, 10)

            # Optional contact detection
            hf.ContactWrapper(self)

            # Stop condition
            if abs(dist_error) < 0.03:
                print("✅ Traveled distance:", moved_dist)
                self.stop(50)
                return

            # Optional: path trace
            curr_pos = hf.drawPath(self, prev_pos)
            prev_pos = curr_pos

            time.sleep(dt)


    def left_for(self, angle_deg):
        angle_rad = np.deg2rad(angle_deg)

        prev_pos, orn = p.getBasePositionAndOrientation(self.id)
        start_yaw = p.getEulerFromQuaternion(orn)[2]

        kp = 5
        kd = 0.3
        ki = 0.0
        pid = PIDController(kp, ki, kd)
        dt = 1 / 240

        for i in range(10000):
            p.stepSimulation()
            try:
                _, orn = p.getBasePositionAndOrientation(self.id)
            except p.error as e:
                print("Error in getting position:", e)
                continue

            current_yaw = p.getEulerFromQuaternion(orn)[2]
            delta_yaw = (current_yaw - start_yaw + np.pi) % (2 * np.pi) - np.pi
            error = angle_rad - delta_yaw

            control = pid.compute(error, dt)
            speed = np.clip(control, -kp, kp)
            self.turn_left(abs(speed), 3)

            if abs(error) < 0.02:
                self.stop(50)
                return
            
            # Optional: path trace
            curr_pos = hf.drawPath(self, prev_pos)
            prev_pos = curr_pos
            time.sleep(dt)

    def right_for(self, angle_deg):
        angle_rad = np.deg2rad(angle_deg)

        prev_pos, orn = p.getBasePositionAndOrientation(self.id)
        start_yaw = p.getEulerFromQuaternion(orn)[2]

        kp = 5
        kd = 0.3
        ki=0.0
        pid = PIDController(kp, ki, kd)
        dt = 1 / 240

        for i in range(10000):
            p.stepSimulation()
            try:
                _, orn = p.getBasePositionAndOrientation(self.id)
            except p.error as e:
                print("Error in getting position:", e)
                continue

            current_yaw = p.getEulerFromQuaternion(orn)[2]
            delta_yaw = (current_yaw - start_yaw + np.pi) % (2 * np.pi) - np.pi
            error = angle_rad - delta_yaw

            control = pid.compute(error, dt)
            speed = np.clip(control, -kp, kp)
            self.turn_right(abs(speed), 3)

            if abs(error) < 0.02:
                self.stop(50)
                return

            
            # Optional: path trace
            curr_pos = hf.drawPath(self, prev_pos)
            prev_pos = curr_pos

            time.sleep(dt)

    # a function that orient the robbot to face the target
    def faceTarget(self, target_orientation):
        # Get current position and orientation
        try:
            current_pos, orn = p.getBasePositionAndOrientation(self.id)
        except p.error as e:
            print("Error in getting position:", e)
            return

        # Get current heading angle
        curr_yaw = p.getEulerFromQuaternion(orn)[2]
        curr_heading_deg = np.rad2deg(curr_yaw)
        curr_heading_deg = hf.degreeIn360(curr_heading_deg)

        angle_to_turn = target_orientation - curr_heading_deg
        angle_to_turn = (angle_to_turn + 540) % 360 - 180  # Normalize to [-180, 180)

        print(f"Turn angle: {angle_to_turn:.2f} degrees")

        if angle_to_turn > 0:
            self.left_for(angle_to_turn)
        elif angle_to_turn < 0:
            self.right_for(angle_to_turn)
    
    # a function that move the robot to a target coordinate
    def toCoord(self, target_coord):
        print("Moving to target coordinate:", target_coord)
        # Get current position and orientation
        try:
            current_pos, orn = p.getBasePositionAndOrientation(self.id)
        except p.error as e:
            print("Error in getting position:", e)
            return

        curr_pos = (-current_pos[1], current_pos[0])
        curr_yaw = p.getEulerFromQuaternion(orn)[2]
        curr_heading_deg = np.rad2deg(curr_yaw)
        curr_heading_deg = hf.degreeIn360(curr_heading_deg)

        #calculate orienation of angle to turn to face target coord
        delta_vec = np.array(target_coord) - np.array(curr_pos)
        dist = np.linalg.norm(delta_vec)
        # print(f"Moving from {curr_pos} to {target_coord}, distance = {dist:.3f} m")
        angle_to_turn = np.arctan2(delta_vec[1], delta_vec[0])
        target_angle_deg = np.rad2deg(angle_to_turn)
        target_angle_deg = hf.degreeIn360(target_angle_deg)  # Normalize to [0, 360)
        target_angle_deg = hf.convertAxis(target_angle_deg)
        # print("current: ", curr_heading_deg, " target: ", target_angle_deg)

        self.faceTarget(target_angle_deg)
        self.forward_for(dist)
        

    # a wait function that just have the robot sits in idle
    def wait(self, ms):
        target_range = int(ms / ((1/240)*1000))
        for i in range(target_range):
            
            time.sleep(1/240)  # Slow it down to real-time 
    
    def followPath(self, path):
        # for i in range(len(path) - 1):
        for i in range(0,3):
            print("moving to coord: ", i+1)
            p.stepSimulation()

            # Get current position and orientation
            try:
                current_pos, orn = p.getBasePositionAndOrientation(self.id)
            except p.error as e:
                print("Error in getting position:", e)
                continue
            curr_pos = (-current_pos[1], current_pos[0])  # pybullet has coord (y,x)
            curr_yaw = p.getEulerFromQuaternion(orn)[2]
            curr_heading_deg = np.rad2deg(curr_yaw)
            curr_heading_deg = hf.degreeIn360(curr_heading_deg)  # Normalize to [0, 360)

            # Next target position
            next_pos = path[i + 1]

            # Compute vector and distance
            delta_vec = np.array(next_pos) - np.array(curr_pos)
            dist = np.linalg.norm(delta_vec)
            print(f"Moving from {curr_pos} to {next_pos}, distance = {dist:.3f} m")
            
            # Compute angle to turn (world frame to robot frame)
            target_angle_rad = np.arctan2(delta_vec[1], delta_vec[0])
            target_angle_deg = np.rad2deg(target_angle_rad)
            target_angle_deg = hf.degreeIn360(target_angle_deg)  # Normalize to [0, 360)
            target_angle_deg = hf.convertAxis(target_angle_deg)

            print("current: ", curr_heading_deg, " target: ", target_angle_deg) 
            # Relative angle the robot must turn (signed shortest path)
            angle_to_turn = (target_angle_deg - curr_heading_deg + 540) % 360 - 180

            print(f"Turn angle: {angle_to_turn:.2f} degrees")
            
            if angle_to_turn > 0:
                self.left_for(angle_to_turn)
            elif angle_to_turn < 0:
                self.right_for(angle_to_turn)
                
            frame = hf.getCamera(self)  # Get camera image and update the view, this will be used for image processing
            while(frame is None):
                frame = hf.getCamera(self)

            mask, offset, annotated_img = hf.findBall(frame)
            cv2.imshow("Ball Detection", annotated_img)
            cv2.waitKey(1)
            

            if offset > 0:
                print("     offset right for: ", offset)
                self.right_for(offset)
            else:
                print("     offset left for: ", offset)
                self.left_for(offset)
            
            # Move forward
            self.forward_for(dist)

            # stop robot to reset wheel momemtum
            self.stop(50)

            time.sleep(1 / 240)  # Slow it down to real-time
