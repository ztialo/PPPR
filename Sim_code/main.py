import pybullet as p
import pybullet_data
import time
from JointControl import DifferentialDriveRobot

# Global Variable
initial_speed = 50 # m/s
initial_force = 5  # N

def main():
    # Connect to physics server with GUI
    p.connect(p.GUI)

    # initialize gravity
    p.setGravity(0, 0, -9.8)

    # Load plane and robot
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for default URDFs
    p.loadURDF("plane.urdf")
    robot = DifferentialDriveRobot("urdf/diff_drive.urdf", [0,0,0.1])
    b1 = p.loadURDF("urdf/ping_pong.urdf", [0.8,0,0.2])

    # print joints
    robot.print_joints()
    robot.left_wheel_joint_index = 0
    robot.right_wheel_joint_index = 1
    
    # left and right wheel control: MOVING FORWARD
    robot.forward(initial_speed, initial_force)
    
    # adding a wheel control slider
    left_wheel_slider = p.addUserDebugParameter("left wheel speed", -50, 50, initial_speed)
    right_wheel_slider = p.addUserDebugParameter("right wheel speed", -50, 50, initial_speed)  

    
    score = 0
    score_text_id = p.addUserDebugText(f"Collected: {score}", [0, 0, 1.5], textSize=2, lifeTime=0)

    

    
    # Run simulation
    for i in range(1000000):
        p.stepSimulation()
        
        # read left wheel and right wheel speed input from GUI
        if i > 100 and i % 100 == 0: # delay read time, make sure everything is initialize correctly first
            lw_speed = p.readUserDebugParameter(left_wheel_slider)
            rw_speed = p.readUserDebugParameter(right_wheel_slider)
        
            # update wheel speed 
            robot.wheel_control(robot.left_wheel_joint_index, p.VELOCITY_CONTROL, lw_speed, initial_force)
            robot.wheel_control(robot.right_wheel_joint_index, p.VELOCITY_CONTROL, rw_speed, initial_force)
        
        # constantly checks for ball 1 contact
        b1contacts = p.getContactPoints(bodyA=robot.id, bodyB=b1)
        if b1contacts:
            print("Ball touched!")
            p.removeBody(b1)
            score += 1
            
            # update score 
            p.removeUserDebugItem(score_text_id)  
            score_text_id = p.addUserDebugText(f"Collected: {score}", [0, 0, 1.5], textSize=2, lifeTime=0)
            # robot.backward(initial_speed, initial_force)
            
        
        time.sleep(1/240)  # Slow it down to real-time

    # Keep window open
    input("Press Enter to exit...")

    # Disconnect when done
    p.disconnect()


if __name__ == "__main__":
    main()

