from helper import util
import rospy
from visualization_msgs.msg import Marker

def simulate(grasp_plan):
    import time, os, sys
    sys.path.append(os.environ['CODE_BASE'] + '/catkin_ws/src/tactile_dexterity/src')

    for plan_dict in grasp_plan:
        for i, t in enumerate(plan_dict['t']):
            visualize_object_pose(plan_dict['object_poses_world'][i])
            update_yumi_cart(plan_dict['palm_poses_world'][i])
            time.sleep(.1)

def visualize_object_pose(q, object_name="realsense_box_experiments.stl"):
    for i in range(4):
        visualize_object(q,
                         filepath="package://config/descriptions/meshes/objects/" + object_name,
                         name="/object",
                         color=(1.0, 126.0 / 255.0, 34.0 / 255.0, 1.),
                         frame_id="/yumi_body")


def update_yumi_cart(poses):
    wrist_to_tip = util.list2pose_stamped([0.0, 0.071399, -0.14344421, 0.0, 0.0, 0.0, 1.0], '')
    world_to_world = util.unit_pose()
    wrist_left = util.convert_reference_frame(wrist_to_tip, world_to_world, poses[0], "yumi_body")
    wrist_right = util.convert_reference_frame(wrist_to_tip, world_to_world, poses[1], "yumi_body")
    visualize_object(wrist_left,
                     filepath="package://config/descriptions/meshes/mpalm/mpalms_all_coarse.stl",
                     name="/gripper_left",
                     color=(0., 0., 1., 1.),
                     frame_id="/yumi_body")

    visualize_object(wrist_right,
                     filepath="package://config/descriptions/meshes/mpalm/mpalms_all_coarse.stl",
                     name="/gripper_right",
                     color=(0., 0., 1., 1.),
                     frame_id="/yumi_body")


def visualize_object(pose, filepath="package://config/descriptions/meshes/objects/object.stl", name="/object",
                     color=(0., 0., 1., 1.), frame_id="/yumi_body", scale=(1., 1., 1.)):
    marker_pub = rospy.Publisher(name, Marker, queue_size=1)
    marker_type = Marker.MESH_RESOURCE
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = name
    marker.header.stamp = rospy.Time(0)
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.type = marker_type
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.lifetime.secs = 1
    marker.pose = pose.pose
    marker.mesh_resource = filepath
    marker.lifetime = rospy.Duration(10000)

    for i in range(10):
        marker_pub.publish(marker)