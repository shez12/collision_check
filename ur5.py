import os.path as osp
import pybullet as p
import open3d as o3d
import sys
import pybullet_data
import numpy as np
from pointcloud_util import pointcloud
import time
import threading


sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl


class UR5Demo():
    def __init__(self) -> None:
        self.obstacles = []
        
        p.connect(p.GUI)
        # p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane = p.loadURDF("plane.urdf")

        # load robot
        urdf_path = "ur5e/ur5e.urdf"
        # urdf_path =pybullet_data.getDataPath()+ "/kuka_iiwa/model.urdf"

        robot_id = p.loadURDF(urdf_path, (0,0,0), useFixedBase = 1)
        robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot


        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)


        # set planner
        # possible values: "RRT", "RRTConnect", "RRTstar", "FMT", "PRM" "EST", "BITstar","BFMT"
        self.pb_ompl_interface.set_planner("RRTstar")


        # add obstacles
        self.update_obstacles()


        self.path = []



    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)


    def update_obstacles(self):
        '''
        update the obstacles in the environment

        '''

        self.pb_ompl_interface.set_obstacles(self.obstacles)
    


    def add_mesh(self, meshFile, pos=[0,0,0]):
        '''
        args:
            mesh: o3d.geometry.TriangleMesh
        
        '''
        # import mesh as pybullet collision shape
        # create a collision shape from the mesh

        meshId = p.createCollisionShape(p.GEOM_MESH,fileName=meshFile, meshScale=[1,1,1])
        # create a multi body with the collision shape
        mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=pos)    
        self.obstacles.append(mesh_body)
        self.pb_ompl_interface.set_obstacles(self.obstacles)

        return mesh_body
    
    
    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return box_id     
    

    def continue_plan(self,path,goal):
        self.robot.set_state(path[-1])
        self.pb_ompl_interface.set_planner("FMT")
        res, path = self.pb_ompl_interface.plan(goal,allowed_time=4)
        self.path = path


    def run(self, start, goal):
        self.robot.set_state(start)
        res, path = self.pb_ompl_interface.plan(goal)
        self.path = path
        print(path[-1])
        thread1 = threading.Thread(target=self.pb_ompl_interface.execute, args=(path,))
        thread2 = threading.Thread(target=self.continue_plan, args=(path, goal))

        if res and path[-1] != goal:
            thread1.start()
            thread2.start()

            thread2.join()
            thread1.join()

        self.pb_ompl_interface.execute(self.path)
        return res, path
    
if __name__== '__main__':
    ####  put mesh 
    env = UR5Demo()
    start = [0,-2 ,0,-1  ,0   ,0]
    goal = [0 ,0 ,0 ,0 ,0,-1.2]

    # load mesh
    pcd = o3d.io.read_point_cloud("plydoc/bunny.ply")
    pointcloud_obj = pointcloud(pcd)
    mesh = pointcloud_obj.create_mesh() 
    pointcloud_obj.export(mesh, "plydoc/bunny.obj")
    pos = pointcloud_obj.get_pos()
    # env.add_mesh("plydoc/bunny.obj", pos)
    env.add_box([1, 0, 0.7], [0.5, 0.5, 0.05])
    env.run(start,goal)



