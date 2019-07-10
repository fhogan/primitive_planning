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

    'manipulated_object = None
    object_pose1_world = util.list2pose_stamped([0.3, -0.2, 0.02, 0, 0, 0.17365,0.98481])
    object_pose2_world = util.list2pose_stamped([0.4, .1, 0.02, 0, 0, 0.17365,0.98481])
    palm_pose_r_object = util.list2pose_stamped([0.0, -0.07997008425912933, 0.024995790000000007, 1.0000000000000002, -4.163336342344336e-17, 0.0, 0.0])
    palm_pose_l_object = util.list2pose_stamped([-0.0672855774812188, -0.2507075560568905, 0.20519579000000004, 0.9554435790175559, 0.10642282670079033, 0.2753110490405191, -0.002361259694729361])

    pushing_plan = pushing_planning(object=manipulated_object,
                                       object_pose1_world=object_pose1_world,
                                       object_pose2_world=object_pose2_world,
                                       palm_pose_l_object=palm_pose_l_object,
                                       palm_pose_r_object=palm_pose_r_object,
                                       arm='r')'
