import os.path as osp
import pybullet as p
import open3d as o3d
import transforms3d as td
import pybullet_data
import pb_ompl2
import numpy as np
from pointcloud_util import pointcloud

# sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))




'''

IMU: world frame
slam target frame: 
robot1 frame: based on slam target frame
robot2 frame: based on slam target frame
camer1 frame: based on slam target frame
camer2 frame: based on slam target frame

additional obstalces: based on camera frame--> convert to world frame

'''


class UR5DualEnv():
    def __init__(self,pos1,pos2,arm_1_orientation,arm_2_orientation) -> None:
        '''
        args:
            pos1: list, [x,y,z]
            pos2: list, [x,y,z]
            arm_1_orientation: p.getQuaternionFromEuler([r,p,y])
            arm_2_orientation: p.getQuaternionFromEuler([r,p,y])
        
        '''

        self.unfixed_obstacles = [] # will move
        self.fixed_obstacles = []   # will not move

        p.connect(p.GUI)
        # p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # load robot
        urdf_path = "ur5e/ur5e.urdf"
  

        #robot1
        robot_1 = p.loadURDF(urdf_path, (0,0,0), useFixedBase = 1)
        robot1 = pb_ompl2.PbOMPLRobot(robot_1)
        self.robot1 = robot1

        #robot2
        robot_2 = p.loadURDF(urdf_path, (0,0,0), useFixedBase = 1)
        robot2 = pb_ompl2.PbOMPLRobot(robot_2)
        self.robot2 = robot2

        # reset robot position and orientation
        p.resetBasePositionAndOrientation(robot_1, pos1, arm_1_orientation)
        p.resetBasePositionAndOrientation(robot_2, pos2, arm_2_orientation)


        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl2.PbOMPL2(self.robot1,self.robot2, self.unfixed_obstacles,self.fixed_obstacles)


        ## set planner/ if not set, it will use the default planner
        # possible values: "RRT", "RRTConnect", "RRTstar", "FMT", "PRM" "EST", "BITstar","BFMT"
        # self.pb_ompl_interface.set_planner("InformedRRTstar")


        # add obstacles
        self.update_obstacles()


    def clear_obstacles(self):
        
        for obstacle in self.unfixed_obstacles:
            p.removeBody(obstacle)
            
        self.unfixed_obstacles = []
        self.update_obstacles()


    def update_obstacles(self):
        '''
        update the obstacles in the environment

        '''
        self.pb_ompl_interface.set_obstacles(self.unfixed_obstacles)


    # def add_mesh(self, pcd,to_bounding_box=False):
    #     '''
    #     import mesh as pybullet collision shape
    #     create a collision shape from the mesh


    #     args:
    #         mesh: o3d.geometry; example: o3d.io.read_point_cloud(path)
    #         to_bounding_box: bool, if True, convert the mesh to bounding bo

    #     '''

    #     pointcloud_obj = pointcloud(pcd)
    #     mesh = pointcloud_obj.create_mesh() 
    #     meshFile= pointcloud_obj.export(mesh, "plydoc/model_{time.strftime(\"%m%d-%H%M\")}.obj")
    #     pos = pointcloud_obj.get_pos()
    #     # orientation = pointcloud_obj.get_orientation()
    #     orientation = [0,0,0,1]

    #     if to_bounding_box==False:
    #         meshId = p.createCollisionShape(p.GEOM_MESH,fileName=meshFile, meshScale=[1,1,1])
    #         # create a multi body with the collision shape
    #         mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=pos, baseOrientation=orientation)    
    #         self.unfixed_obstacles.append(mesh_body)
    #         self.pb_ompl_interface.set_obstacles(self.unfixed_obstacles)
    #     else:
    #         mesh = pointcloud_obj.get_mesh()
    #         min_x, min_y, min_z, max_x, max_y, max_z = pointcloud_obj.to_bounding_box(mesh)
    #         collision_box_id = p.createCollisionShape(p.GEOM_BOX,
    #                                        halfExtents=[(max_x - min_x) / 2,
    #                                              (max_y - min_y) / 2,
    #                                              (max_z - min_z) / 2],
    #                                       meshScale=[1, 1, 1])
    #         coll_box = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_box_id, basePosition=pos, baseOrientation=orientation)
    #         self.unfixed_obstacles.append(coll_box)
    #         self.pb_ompl_interface.set_obstacles(self.unfixed_obstacles)

    def add_mesh(self,file_path,posi,ori):
        meshId = p.createCollisionShape(p.GEOM_MESH,fileName=file_path, meshScale=[1,1,1],flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        # create a multi body with the collision shape
        mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=posi, baseOrientation= ori )    
        self.obstacles.append(mesh_body)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return mesh_body


    
    def add_box(self, box_pos, half_box_size):
        '''
        add box as obstacle
        args:
            box_pos: list, [x,y,z]
            half_box_size: list, [x,y,z]
    
        '''
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.unfixed_obstacles.append(box_id)
        self.pb_ompl_interface.set_obstacles(self.unfixed_obstacles)
        return box_id     
    


    def run(self, start1, goal1,start2,goal2):
        '''
        execute the planner and execute the planned path
        '''
        self.robot1.set_state(start1)
        self.robot2.set_state(start2)
        # print("robot1 state:", env.robot1.get_cur_state())
        # print("robot2 state:",env.robot2.get_cur_state())

        res, path1,path2 = self.pb_ompl_interface.plan(goal1,goal2)
        # execute the planned path
        self.pb_ompl_interface.execute(path1,path2)


    
if __name__== '__main__':

    matrix = np.array([
    [-0.9966,    0.05556,   0.06165,   0.6978],
    [-0.05208,  -0.997,     0.05659,  -0.4639],
    [ 0.06462,   0.05319,   0.9965,    0.5333],
    [ 0,         0,         0,         1]
])
    R = matrix[:3,:3]
    t = matrix[:3,3]

    quat = td.quaternions.mat2quat(R)

    
    arm_1_position = [0.3,0,1]
    arm_1_orientation = p.getQuaternionFromEuler([0, 1.57, 0])  # Rotate to face the box
    # arm_1_position = t
    # arm_1_orientation = quat



    # Right side
    arm_2_position = [-0.3,0,1]
    arm_2_orientation = p.getQuaternionFromEuler([0, -1.57, 0])  # Rotate to face the box

    env = UR5DualEnv(arm_1_position,arm_2_position,arm_1_orientation,arm_2_orientation)  

    robot1 = env.robot1
    robot2 = env.robot2



    env.add_mesh("plydoc/mesh6.obj",[0,0,0],[0,0,0,1])


    start = [0 ,-1.57,0,0 ,0,1]
    start2 = [-3 ,-1.57 ,0,0 ,0,0]

    robot1._set_joint_positions(robot1.joint_idx,start)
    robot2._set_joint_positions(robot2.joint_idx,start2)


    box_position = [0,0,0]
    box_size = [0.3,0.1,1.5]
    env.add_box(box_position, box_size)


    env.add_box([0, -0.3, 0.9], [0.3, 0.3, 0.05])
    env.add_box([0, -0.3, 0.4], [0.3, 0.3, 0.05])


    pos1 = [0.15,-0.3,0.7]
    pos2 = [-0.15,-0.3,0.7]


    import time
    cur_time  = time.time()
    goal1 = p.calculateInverseKinematics(robot1.id, 6, pos1)
    goal2 = p.calculateInverseKinematics(robot2.id, 6, pos2)
    print("time:",time.time()-cur_time)


    print("planned goal1",goal1)
    print("planned goal2",goal2)
    

    env.run(start,goal1,start2,goal2)

    input("Press Enter to continue...")

    print("obstacles not cleared",env.unfixed_obstacles)
    env.clear_obstacles()
    print("obstacles cleared",env.unfixed_obstacles)

    input("Press Enter to continue...")




