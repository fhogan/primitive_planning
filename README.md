# Planning Functions for Robot Manipulation with Twin Palms

This repo contains a Python library to plan robot manipulation primitives: pushing, pulling, levering, and grasping. This repo is adapting from [Mpalms Repo](https://github.com/mcubelab/mpalms), which contains the full planning + feedback pipeline for the tactile dexterity project.

## Dependencies

### Mandatory
This repo contains depencies to the following python librairies:

* numpy
* trimesh
* python-fcl

### Optional

* rospy

## Installation and Setup
To install the dependencies, run 

`sudo pip install -e .`

Note: These instructions were tested on Ubuntu 16.04. 

## Example Usage 

There are 4 primitive functions implemented: pushing, pulling, levering, and grasping. All are defined in the function 'planning.py'. Here is an example of how to call a primitive:

     manipulated_object = None
     object_pose1_world = util.list2pose_stamped([0.3, -0.2, 0.026029999560531224, 0.0, 0.0, 0.1736481776669303, 0.9848077530122082])
     object_pose2_world = util.list2pose_stamped([0.4, 0.1, 0.026029999560531224, 0.0, 0.0, 0.2588190451025207, 0.9659258262890683])
     palm_pose_l_object = util.list2pose_stamped([0.24468880597338838, 0.5590393007227565, 0.18711189974498546, 0.36391209838571453, 0.877140948669767, -0.14647338311772426, 0.27701496142492954])
     palm_pose_r_object = util.list2pose_stamped([-0.034310268964238944, 0.01420462473093076, 0.032571000439468735, 0.4055797427018062, 0.5792279968284825, 0.5792279968284825, 0.4055797427018062])
     pulling_plan = pulling_planning(object=manipulated_object,
                                    object_pose1_world=object_pose1_world,
                                    object_pose2_world=object_pose2_world,
                                    palm_pose_l_object=palm_pose_l_object,
                                    palm_pose_r_object=palm_pose_r_object,
                                     arm='r')


Note that manipulated_object = None as no information about the object properties is used to develop the plan. At this point, only the primitive 'levering' uses object information to check collisions between the object and the table. The primitive functions return a dictionary detailed the trajectory of the object and the palms:
Main pulling primitive function. Return a plan that contains the pose trajectories of the object and palms to achieve desired object reconfiguration.
'
    :param object: (collisions.CollisionBody) that contains the geometry of the object. (Not currently used)
    :param object_pose1_world: (util.PoseStamped) Initial object pose in world frame.
    :param object_pose2_world: (util.PoseStamped) Final object pose in world frame.
    :param palm_pose_l_object: (util.PoseStamped) Left palm pose in object frame.
    :param palm_pose_r_object: (util.PoseStamped) Right palm pose in object frame.
    :return: plan_list: (list of dict with keys)
        :param palm_poses_r_world: (list of util.PoseStamped) Trajectory of right palm poses in world frame
        :param palm_poses_l_world: (list of util.PoseStamped) Trajectory of left palm poses in world frame
        :param object_poses_world: (util.PoseStamped) Trajectory of object poses in world frame
        :param primitive: (util.PoseStamped) Name of primitive (i.e., 'grasping')
        :param name: (util.PoseStamped) Name of plan
        :param t: (util.PoseStamped) list of timestamps associated with each pose
        :param N: (util.PoseStamped) Number of keypoints in the plan (i.e., len(plan_dict['t'])'
   
