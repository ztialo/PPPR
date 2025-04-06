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
        x = random.uniform(-10, 10)  # Random x position
        y = random.uniform(-10, 10)  # Random y position
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
    
def tenBallTest():
    addBalls_at(-1.414, -1.699) #(-1.699, -1.414)
    addBalls_at(-1.09, -0.139)  #(-0.139, -1.09)
    addBalls_at(0.107, 0.918)   #(0.918, 0.107)
    addBalls_at(0.978, -1.362)  #(-1.362, 0.978)
    addBalls_at(1.918, 0.672)   #(0.672, 1.918)
    addBalls_at(0.119, 1.728)   #(1.728, 0.119)
    addBalls_at(0.488, -0.824)  #(-0.824, 0.488)
    addBalls_at(0.422, -0.109)  #(-0.109, 0.422)
    addBalls_at(0.623, -1.892)  #(-1.892, 0.623)
    addBalls_at(0.869, 1.345)   #(1.345, 0.869)
    