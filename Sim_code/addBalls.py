import Globals
import random
import pybullet as p


def addBalls_at(x, y):
    ball_id = p.loadURDF("urdf/ping_pong.urdf", [x, y, 0.05])
    Globals.balls.append(ball_id)
    return ball_id

# Function to add balls in random locations in simulation
def addBalls_rand(n):
    for i in range(1, n+1):
        x = random.uniform(-2, 2)  # Random x position
        y = random.uniform(-2, 2)  # Random y position
        ball_id = p.loadURDF("urdf/ping_pong.urdf", [x, y, 0.05])
        Globals.balls.append(ball_id)

def ballMeasTest():
    addBalls_at(0.458258, -0.2) # radius 0.5
    addBalls_at(0.95394, 0.3) # radius 1
    addBalls_at(1.48661, -0.2) # radius 1.5
    addBalls_at(1.98997, 0.2) # radius 2
    addBalls_at(2.498, -0.1) # radius 2.5
    addBalls_at(2.99833, 0.1) # radius 3
    
def fourCoordTest():
    addBalls_at(0.5, 0) # radius 0.5
    addBalls_at(0, -1) # radius 1
    addBalls_at(-1.5, 0) # radius 1.5
    addBalls_at(0, 2) # radius 2