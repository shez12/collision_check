from myRobot import MyRobot
import numpy as np
import rospy


'''

Given a path, move the robot to the path

'''



class myur5(MyRobot):
    def __init__(self):
        rospy.init_node('ur5_collision', anonymous=True)
        super().__init__()

    def get_current_joint(self):
        return super().get_joints()
    
    def move_(self,path):
        # Initialize the UR5 robot
        
        #check initial position
        current_joint = self.get_current_joint()
        print("start joint", current_joint)

        if (max(abs(path[0]-current_joint))>0.3):
            print("start joint is not correct")
        else:
            step = 0
            for joint_path in path:
                diff  = [abs(a-b) for a,b in zip(joint_path,current_joint)]
                joints_movement = np.max(diff)
                self.move_joints(joint_path,duration =0.5* joints_movement, wait = True)
                current_joint = joint_path
                # step+=1
                # if step%100 ==0:
                #     input("press enter to continue")
                
# multi robot




if __name__ == "__main__":
    myur5 = MyRobot()
    myur5.__init__()
    myur5.get_joints()
