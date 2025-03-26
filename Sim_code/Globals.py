
"""
Author  : Zi Tao Li
Date    : March 26, 2025
Description: This file contains all of the global variables to share among files
"""
import pybullet as p

# Global Variable
initial_speed = 50 # m/s
initial_force = 5  # N
score = 0

# object lists
balls = []

# camera variables
width, height = 320, 240
fov = 60
aspect = width / height
near = 0.1
far = 10

# global debug variables
score_text_id = 0
left_wheel_slider = 0
right_wheel_slider = 0

def debug_init():
    score_text_id = p.addUserDebugText(f"Collected: {score}", [0, 0, 1.5], textSize=2, lifeTime=0)
    left_wheel_slider = p.addUserDebugParameter("left wheel speed", -50, 50, initial_speed)
    right_wheel_slider = p.addUserDebugParameter("right wheel speed", -50, 50, initial_speed)  
