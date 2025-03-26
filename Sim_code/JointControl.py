"""
Author  : Zi Tao Li
Date    : March 25, 2025
Description: This files contains functions that controls the joint
                movements. 
"""
import pybullet as p

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
    