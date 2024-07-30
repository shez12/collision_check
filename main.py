import numpy as np
import rospy
from ur5_dual import UR5DualEnv


'''

three main parts in the code:
1. realtime refreash obstacles and robots

2. offline path_planning

3. collision avoidance during moving
'''


def get_obstacles():
    '''
    get obstacles from ros topic
    
                octomap_full (octomap_msgs/Octomap)

                Get an OctoMap. Available only if rtabmap_ros is built with octomap. 

                octomap_binary (octomap_msgs/Octomap)

                Get an OctoMap. Available only if rtabmap_ros is built with octomap. 

                octomap_occupied_space (sensor_msgs/PointCloud2)

                A point cloud of the occupied space (obstacles and ground) of the OctoMap. Available only if rtabmap_ros is built with octomap. 

                octomap_obstacles (sensor_msgs/PointCloud2)

                A point cloud of the obstacles of the OctoMap. Available only if rtabmap_ros is built with octomap. 

                octomap_ground (sensor_msgs/PointCloud2)

                A point cloud of the ground of the OctoMap. Available only if rtabmap_ros is built with octomap. 

                octomap_empty_space (sensor_msgs/PointCloud2)

                A point cloud of empty space of the OctoMap. Available only if rtabmap_ros is built with octomap. 

                octomap_grid (nav_msgs/OccupancyGrid)

                The projection of the OctoMap into a 2D occupancy grid map. Available only if rtabmap_ros is built with octomap.

    return:
        obstacles: octotree        
    '''


    pass

def get_robots(robot_id):
    '''
    get robots poses from ros topic
    
    return:
        robots: list of robots poses
    '''
    pass
    

class AvoidObstacles():
    def __init__(self, UR5DualEnv) -> None:
        '''
        args:
            env: UR5DualEnv class
        
        '''
        self.env = UR5DualEnv


    def update_env(self):
        '''
        update obstacles and robots in the environment
            
        '''
        obstacles = get_obstacles()
        self.env.update_obstacles(obstacles)



    def update_robot_pose(self):
        '''
        update robot poses in the environment
            
        '''
        #todo: get robot poses from ros topic
        robot1_pose = get_robots(1)
        robot2_pose = get_robots(2)

        self.env.robot1._set_joint_positions(robot1_pose)
        self.env.robot2._set_joint_positions(robot2_pose)

    def path_planning(self, start1,start2,goal1,goal2):
        pass


    