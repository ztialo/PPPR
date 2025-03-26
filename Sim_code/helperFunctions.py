
"""
Author  : Zi Tao Li
Date    : March 26, 2025
Description: This file contains all of the helper functions 
"""

import Globals
import pybullet as p

def checkContact(robot, balls):
    b1contacts = p.getContactPoints(bodyA=robot.id, bodyB=balls[0])
    if b1contacts:
        print("Ball touched!")
        p.removeBody(balls[0])
        Globals.score += 1
        
    return Globals.score
    
def slider_control(robot):
    lw_speed = p.readUserDebugParameter(Globals.left_wheel_slider)
    rw_speed = p.readUserDebugParameter(Globals.right_wheel_slider)

    # update wheel speed 
    robot.wheel_control(robot.left_wheel_joint_index, p.VELOCITY_CONTROL, lw_speed, Globals.initial_force)
    robot.wheel_control(robot.right_wheel_joint_index, p.VELOCITY_CONTROL, rw_speed, Globals.initial_force)