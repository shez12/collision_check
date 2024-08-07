'''
export PYTHONPATH=$PYTHONPATH:/home/rmqlife/work/collision_check/ompl/py-bindings

'''

import pybullet as p
import pybullet_data
import numpy as np
import pb_ompl
from move_ur5 import *


class UR5Demo():
    def __init__(self,posi,ori) -> None:
        '''
        Initialize the environment
        args:
            posi: list of 3 floats, position of the robot
            ori: list of 4 floats, orientation of the robot(quaternion xyzw)
        '''
        self.obstacles = []
        
        p.connect(p.GUI)
        # p.connect(p.DIRECT)
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # load robot
        urdf_path = "ur5e/ur5e.urdf"
        robot_id = p.loadURDF(urdf_path,basePosition=posi, baseOrientation= ori, useFixedBase = 1)
        self.robotid = robot_id
        robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot
        
        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        
        # set planner
        # possible values: "RRT", "RRTConnect", "RRTstar", "FMT", "PRM" "EST", "BITstar","BFMT"
        self.pb_ompl_interface.set_planner("BITstar")

        # add obstacles
        self.update_obstacles()


    def clear_obstacles(self):
        '''
        clear all the obstacles in the environment
        
        '''
        for obstacle in self.obstacles:
            p.removeBody(obstacle)


    def update_obstacles(self):
        '''
        update the obstacles in the environment

        '''

        self.pb_ompl_interface.set_obstacles(self.obstacles)
    

            
    def add_mesh(self,file_path,posi,ori):
        '''
        Add a mesh(.obj) to the environment
        args:
            file_path: str, path to the mesh file
            posi: list of 3 floats, position of the mesh
            ori: list of 4 floats, orientation of the mesh
        '''

        meshId = p.createCollisionShape(p.GEOM_MESH,fileName=file_path, meshScale=[1,1,1],flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        # create a multi body with the collision shape
        mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=posi, baseOrientation= ori )    
        self.obstacles.append(mesh_body)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return mesh_body
    

    def add_box(self, box_pos, half_box_size):
        '''
        Add a box to the environment
        args:
            box_pos: list of 3 floats, position of the box
            half_box_size: list of 3 floats, half size of the box
        
        '''

        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return box_id     
    

    def run(self, start, goal):
        '''
        Run the planner and execute the path
        args:
            start: list of floats, start configuration
            goal: list of floats, goal configuration
        return:
            res: bool, True if a path is found
            path: list of list of floats, path found by the planner
        
        '''


        self.robot.set_state(start)
        res, path = self.pb_ompl_interface.plan(goal)
        self.path = path
        self.pb_ompl_interface.execute(self.path)
            
        return res, path


if __name__== '__main__':
    ##### EXAMPLE 1#####


    ur5_move = myur5()
    env = UR5Demo([-0.2445,-0.6305,0.1093],[ 0.71562555 ,0.54186322 ,0.248717 , 0.36387385])
    start  = ur5_move.get_current_joint()

    goal_point = [0.1,-0.15,-0.2]
    goal = p.calculateInverseKinematics(env.robot.id, 6, goal_point)

    # end_point = [-0.3,-0.1,-0.2]
    # goal = p.calculateInverseKinematics(env.robot.id, 6, end_point)

    env.robot.set_state(start)
    env.add_mesh("plydoc/mesh12.obj",[0,0,0],[0,0,0,1])
    res,path = env.run(start,goal)
    input("Press Enter to continue...")
    ur5_move.move_(path)
    



