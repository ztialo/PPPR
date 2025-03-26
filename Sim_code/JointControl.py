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
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, v, f) # lw move forward at 50 m/s
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, -v, f) # rw move forward
    
    def backward(self, v, f):
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, -v, f) # lw move forward at 50 m/s
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, -v, f) # rw move forward
    
    def stop(self, brake_force):
        self.wheel_control(self.left_wheel_joint_index, p.VELOCITY_CONTROL, 0, brake_force)
        self.wheel_control(self.right_wheel_joint_index, p.VELOCITY_CONTROL, 0, brake_force)

    
    
    def forward_for(self, dist):
        # get current position, calculate the difference and stop when reach target distance
        start_pos, orn = p.getBasePositionAndOrientation(self.id)
        start_vec = np.array(start_pos)
        
        rot_matrix = p.getMatrixFromQuaternion(orn)
        # forward direction is the first column of the rotation matrix
        forward_vec = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
        
        force = 10
        self.forward(50, force)
        for i in range(10000):
            p.stepSimulation()
            
            # slow down the position check
            if i % 10 == 0:
                current_pos, _ = p.getBasePositionAndOrientation(self.id)
                current_vec = np.array(current_pos)

                # Project movement onto forward vector
                delta = current_vec - start_vec
                moved_distance = np.dot(delta, forward_vec)

                if moved_distance >= dist:
                    print(f"✅ Reached target distance: {moved_distance:.3f} m")

                    self.stop(force*10) # 5 times the force to stop the robot?
                    return
            
            if(Globals.score != hf.checkContact(self, Globals.balls)):
                # score changed so update score 
                p.removeUserDebugItem(Globals.score_text_id)  
                Globals.score_text_id = p.addUserDebugText(f"Collected: {Globals.score }", [0, 0, 1.5], textSize=2, lifeTime=0)
            
            time.sleep(1/240)  # Slow it down to real-time

        
        


        