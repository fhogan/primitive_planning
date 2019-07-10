import fcl
import os
import trimesh
import util
import copy
import numpy as np

request = fcl.CollisionRequest()
result = fcl.CollisionResult()

def initialize_collision_object(tris, verts):
    m = fcl.BVHModel()
    m.beginModel(len(verts), len(tris))
    m.addSubModel(verts, tris)
    m.endModel()
    t = fcl.Transform()
    return fcl.CollisionObject(m, t)

def is_collision(body1, body2):
    return fcl.collide(body1, body2, request, result)

class CollisionBody():
    def __init__(self, mesh_name = "config/descriptions/meshes/table/table_top_collision.stl"):
        self.trimesh = trimesh.load(mesh_name)
        self.collision_object = initialize_collision_object(self.trimesh.faces, self.trimesh.vertices)

    def setCollisionPose(self, collision_object, pose_world):
        T_gripper_pose_world = util.matrix_from_pose(pose_world)
        R = T_gripper_pose_world[0:3, 0:3]
        t = T_gripper_pose_world[0:3, 3]
        collision_object.setRotation(R)
        collision_object.setTranslation(t)
        self.collision_object = collision_object

class CheckCollisions():
    def __init__(self, gripper_name, table_name):
        self. gripper_left = CollisionBody(mesh_name=gripper_name)
        self.gripper_right = CollisionBody(mesh_name=gripper_name)
        self.table = CollisionBody(mesh_name=table_name)

    def check_collisions_with_table(self, gripper_pose, arm='l'):
        if arm=='l':
            self.gripper_left.setCollisionPose(self.gripper_left.collision_object,
                                           gripper_pose)
            _is_collision = is_collision(self.gripper_left.collision_object,
                                    self.table.collision_object)
        else:
            self.gripper_right.setCollisionPose(self.gripper_right.collision_object,
                                                    gripper_pose)
            _is_collision = is_collision(self.gripper_right.collision_object,
                                                self.table.collision_object)

        return _is_collision

    def avoid_collision(self, pose_gripper, arm="l", tol=0.003, height_above_table=0.006, axis=[-1, 0, 0]):
        pose_gripper_safe = copy.deepcopy(pose_gripper)
        pose_gripper_safe.pose.position.z -= height_above_table
        while self.check_collisions_with_table(pose_gripper_safe, arm=arm):
            pose_gripper_safe = util.offset_local_pose(pose_gripper_safe, np.array(axis) * tol)
        pose_gripper = copy.deepcopy(pose_gripper_safe)
        pose_gripper.pose.position.z += height_above_table
        return pose_gripper
