import pybullet as p
import pybullet_data
import time

def print_joints(robot):
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        print(i, p.getJointInfo(robot, i)[1])
        
def wheel_control(robot, joint_index, mode, v, f):
    p.setJointMotorControl2(
        bodyUniqueId = robot,
        jointIndex = joint_index,
        controlMode = mode,
        targetVelocity = v,  # Positive value = forward
        force = f
    )

def main():
    # Connect to physics server with GUI
    p.connect(p.GUI)

    # initialize gravity
    p.setGravity(0, 0, -9.8)

    # Load plane and robot
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for default URDFs
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("urdf/diff_drive.urdf", [0,0,0.1])
    b1 = p.loadURDF("urdf/ping_pong.urdf", [0.8,0,0.2])

    # print joints
    print_joints(robot)
    left_wheel_joint_index = 0
    right_wheel_joint_index = 1

    # left and right wheel control: MOVING FORWARD
    wheel_control(robot, left_wheel_joint_index, p.VELOCITY_CONTROL, 50, 5.0) # lw move forward at 50 m/s
    wheel_control(robot, right_wheel_joint_index, p.VELOCITY_CONTROL, 50, 5.0) # rw move forward


    # Run simulation
    for i in range(1000000):
        p.stepSimulation()
        time.sleep(1/240)  # Slow it down to real-time

    # Keep window open
    input("Press Enter to exit...")

    # Disconnect when done
    p.disconnect()


if __name__ == "__main__":
    main()

