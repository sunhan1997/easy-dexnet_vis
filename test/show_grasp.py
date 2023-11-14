import open3d as o3d
from grasp_gripper_open3d.collision_detector import ModelFreeCollisionDetector
from graspnetAPI import GraspGroup
from grasp_gripper_open3d.data_utils import CameraInfo, create_point_cloud_from_depth_image
import math
import numpy as np



def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=0.01)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=0.01)
    gg = gg[~collision_mask]
    return gg

def vis_grasps(gg, cloud,axis):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:50]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, axis, *grippers])


ROOT_PATH = '/home/sunh/6D_grasp/easy-dexnet'
## read mesh
mesh  = o3d.io.read_triangle_mesh(ROOT_PATH + '/data/Gear/Gear.obj')
cloud  = o3d.io.read_point_cloud( ROOT_PATH +'/data/Gear/Gear.ply')
## read grasp
grasp_pose = np.load(ROOT_PATH +'/data/grasp_reault_gear/grasp.npy')
grasp_q = np.load(ROOT_PATH +'/data/grasp_reault_gear/grasp_q.npy')


gg_list = []

for i in  range(len(grasp_pose) ):
    g_T = grasp_pose[i]
    q = grasp_q[i]

    ## set grasp quality
    if q<0.1:
        continue

    ########### set grasp
    T = np.array([[0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [1, 0, 0, 0],
                  [0, 0, 0, 1]])
    g_T = np.dot(g_T, T)
    g = np.array([q, 0.03, 0.05, 0.00,  ## score with hight detpt
                  g_T[0, 0], g_T[0, 1], g_T[0, 2], g_T[1, 0],
                  g_T[1, 1], g_T[1, 2], g_T[2, 0], g_T[2, 1],
                  g_T[2, 2], g_T[0, 3], g_T[1, 3], g_T[2, 3], -1.00000000e+00])

    gg_list.append(g)
    print(q)


gg_array = np.array(gg_list)
gg = GraspGroup(gg_array)
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
grippers = gg.to_open3d_geometry_list()
o3d.visualization.draw_geometries([mesh, *grippers])




##################################################################### grasp class from graspnet  ###########################################
# import open3d as o3d
# from grasp_gripper_open3d.collision_detector import ModelFreeCollisionDetector
# from graspnetAPI import GraspGroup
# from grasp_gripper_open3d.data_utils import CameraInfo, create_point_cloud_from_depth_image
# import math
# import numpy as np
#
# def angle2Rmat(theta):
#     R_x = np.array([[1, 0, 0],
#                     [0, math.cos(theta[0]), -math.sin(theta[0])],
#                     [0, math.sin(theta[0]), math.cos(theta[0])]
#                     ])
#
#     R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
#                     [0, 1, 0],
#                     [-math.sin(theta[1]), 0, math.cos(theta[1])]
#                     ])
#
#     R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
#                     [math.sin(theta[2]), math.cos(theta[2]), 0],
#                     [0, 0, 1]
#                     ])
#     R = np.dot(R_z, np.dot(R_y, R_x))
#     return R
#
# def collision_detection(gg, cloud):
#     mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=0.01)
#     collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=0.01)
#     gg = gg[~collision_mask]
#     return gg
#
# def vis_grasps(gg, cloud,axis):
#     gg.nms()
#     gg.sort_by_score()
#     gg = gg[:50]
#     grippers = gg.to_open3d_geometry_list()
#     o3d.visualization.draw_geometries([cloud, axis, *grippers])
#
#
# ## read mesh
# mesh  = o3d.io.read_triangle_mesh('/media/sunh/Samsung_T5/6D_grasp/easy-dexnet/data/cad/baota.obj')
# cloud  = o3d.io.read_point_cloud('/media/sunh/Samsung_T5/6D_grasp/easy-dexnet/data/cad/baota.ply')
# ## read grasp
# grasp_pose = np.load('/media/sunh/Samsung_T5/6D_grasp/easy-dexnet/data/grasp_reault/grasp.npy')
# grasp_q = np.load('/media/sunh/Samsung_T5/6D_grasp/easy-dexnet/data/grasp_reault/grasp_q.npy')
# gg_list = []
#
# for i in  range(len(grasp_pose)):
#
#     g_T = grasp_pose[i]
#     q = grasp_q[i]
#
#     ## set grasp quality
#     if q<0.1:
#         continue
#
#     ########### set grasp
#     T = np.array([[0, 1, 0, 0],
#                   [0, 0, 1, 0],
#                   [1, 0, 0, 0],
#                   [0, 0, 0, 1]])
#     g_T = np.dot(g_T, T)
#     g = np.array([q, 0.03, 0.05, 0.00,  ## score with hight detpt
#                   g_T[0, 0], g_T[0, 1], g_T[0, 2], g_T[1, 0],
#                   g_T[1, 1], g_T[1, 2], g_T[2, 0], g_T[2, 1],
#                   g_T[2, 2], g_T[0, 3], g_T[1, 3], g_T[2, 3], -1.00000000e+00])
#
#     gg_list.append(g)
#     print(q)
#
#
# gg_array = np.array(gg_list)
# gg = GraspGroup(gg_array)
# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
# grippers = gg.to_open3d_geometry_list()
# o3d.visualization.draw_geometries([mesh, *grippers])

##################################################################### grasp class from graspnet  ###########################################