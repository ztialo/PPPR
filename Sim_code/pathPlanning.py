"""
Author  : Zi Tao Li
Date    : March 31, 2025
Description: This file contains all the methods that I will be trying to 
            find the most optimal path plannigng agoroithm for ball collecting
"""

import numpy as np

def NearestNeighbor(world_map):
    # Initialize variables
    current_pos = world_map[-1] # the last positoin stored the robot position
    world_map.pop()
    path = [current_pos] # a ordered path for the robot to follow
    ball_nums = len(world_map)
    visited = set()
    
    while len(path) != ball_nums+1:
        # Find the nearest neighbor
        nearest_ball = None
        min_distance = float('inf')
        
        for ball in world_map:
            if ball not in visited:
                distance = np.linalg.norm(np.array(current_pos) - np.array(ball))
                if distance < min_distance:
                    min_distance = distance
                    nearest_ball = ball
        
        # Move to the nearest ball
        path.append(nearest_ball)
        visited.add(nearest_ball)
        current_pos = nearest_ball
    
    return path