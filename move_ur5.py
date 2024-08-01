from myRobot import MyRobot
import numpy as np

def move_ur5(path):
    # Initialize the UR5 robot
    ur5 = MyRobot()
    
    #check initial position
    current_joint = ur5.get_joints()
    print("start joint", current_joint)

    if (max(abs(path[0]-current_joint))>0.3):
        print("start joint is not correct")
    else:
        for joint_path in path:
            joints_movement = np.max(np.abs(joint_path - current_joint))
            ur5.move_joints(joint_path,duration = 5* joints_movement, wait = 0.1)
            current_joint = joint_path
    