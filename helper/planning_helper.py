import transformations
import util
import numpy as np

def get_2d_pose(object_pose):
    T = util.matrix_from_pose(object_pose)
    x = object_pose.pose.position.x
    y = object_pose.pose.position.y
    euler_angles = transformations.euler_from_matrix(T, 'sxyz')
    return np.array([x, y, euler_angles[2]])

def get3dpose_object(pose2d, pose3d_nominal):
    x_nom, y_nom, theta_nom = get_2d_pose(pose3d_nominal)
    delta_theta = pose2d[2] - theta_nom
    T = transformations.euler_matrix(0,0,delta_theta, 'sxyz')
    pose_transform = util.pose_from_matrix(T)
    new_pose = util.transform_pose(pose_source=pose3d_nominal,
                                   pose_transform=pose_transform)
    new_pose.pose.position.x = pose2d[0]
    new_pose.pose.position.y = pose2d[1]
    return new_pose

def rotate_2d_pose_origin(pose, angle, origin, Flip=False):
    r = pose[0:2] - origin
    theta = pose[2]
    R = util.C3_2d(angle).transpose()
    r_new = origin + np.matmul(R, r)
    if Flip:
        theta_new = -theta
    else:
        theta_new = theta
    pose_new = np.array([r_new[0], r_new[1], theta_new])
    return pose_new

def dubins_trajectory(q0, qf, radius, velocity_real, step_size, contact_angle=0):
    velocity = velocity_real * 10
    #1. compute 2d trajectory
    configurations, N_star = compute_dubins_base(q0, qf, radius, velocity_real, step_size, contact_angle=contact_angle)
    #2. convert trajectory back to original coordinate system (account for different object sides)
    configurations_transformed = []
    for counter, configuration in enumerate(configurations):
        configurations_transformed.append(rotate_2d_pose_origin(configuration, contact_angle, q0[0:2]))
    #3. reformat object trajectory
    t_star = np.array(N_star) / velocity
    x_star = np.array(configurations_transformed)
    if len(configurations)>0:
        x_star[:, 2] = util.unwrap(x_star[:, 2])
    return configurations_transformed, N_star, x_star, t_star

def compute_dubins_base(q0, qf, radius, velocity_real, step_size, contact_angle=0, is_show=False):
    import dubins
    velocity = velocity_real * 10
    # 1. generate dubins trajectory from q0 to qf
    qf_tilde = rotate_2d_pose_origin(qf, -contact_angle, q0[0:2])
    path = dubins.shortest_path(q0, qf_tilde, radius)
    configurations, N_star = path.sample_many(step_size * velocity)
    if is_show:
        import matplotlib.pyplot as plt
        plt.plot(np.array(configurations)[:,0], np.array(configurations)[:,1]);plt.show()
    return configurations, N_star

def rotate_2d_pose_origin(pose, angle, origin, Flip=False):
    r = pose[0:2] - origin
    theta = pose[2]
    R = util.C3_2d(angle).transpose()
    r_new = origin + np.matmul(R, r)
    if Flip:
        theta_new = -theta
    else:
        theta_new = theta
    pose_new = np.array([r_new[0], r_new[1], theta_new])
    return pose_new


def palm_pose_from_object(object_pose, palm_pose_object):
    palm_pose_initial_world = util.convert_reference_frame(pose_source=palm_pose_object,
                                                      pose_frame_target=util.unit_pose(),
                                                      pose_frame_source=object_pose,
                                                      frame_id="world"
                                                      )
    return palm_pose_initial_world

def palm_poses_from_object(object_pose, palm_pose_l_object, palm_pose_r_object):
    palm_poses_initial_world_list = []
    for palm_pose_initial_object in [palm_pose_l_object, palm_pose_r_object]:
        palm_poses_initial_world_list.append(palm_pose_from_object(object_pose, palm_pose_initial_object))
    return palm_poses_initial_world_list

def align_arm_poses(pose, grasp_dist):
    '''Set position of right arm with respect to left arm'''
    T_left_world = util.matrix_from_pose(pose)
    pose_right_left = util.unit_pose()
    pose_right_left.header.frame_id = pose.header.frame_id
    pose_right_left.pose.position.y = - grasp_dist
    quat = util.rotate_quat_y(pose_right_left)
    pose_right_left.pose.orientation.x = quat[0]
    pose_right_left.pose.orientation.y = quat[1]
    pose_right_left.pose.orientation.z = quat[2]
    pose_right_left.pose.orientation.w = quat[3]
    T_right_left = util.matrix_from_pose(pose_right_left)
    T_right_world = np.matmul(T_left_world, T_right_left)
    pose_right = util.pose_from_matrix(T_right_world)
    return [pose, pose_right]

def initialize_plan(palm_poses_initial, object_pose_initial, primitive, plan_name):
    #1. Return robot plan
    plan_dict = {}
    plan_dict['type'] = 'SetPoseTraj'
    plan_dict['primitive'] = primitive
    plan_dict['name'] = plan_name
    plan_dict['palm_poses_world'] = [palm_poses_initial]
    plan_dict['palm_pose_l_world'] = [palm_poses_initial[0]]
    plan_dict['palm_pose_r_world'] = [palm_poses_initial[1]]
    plan_dict['object_poses_world'] = [object_pose_initial]
    return plan_dict

def move_cart(palm_poses_final, plan_previous, primitive, plan_name=None, N=100):
    #1. initialize variables
    initial_pose = plan_previous['palm_poses'][-1]
    object_pose = plan_previous['object_pose'][-1]
    #2. interpolate poses individually (left and right)
    pose_left_interpolate_left = util.interpolate_pose(initial_pose[0], palm_poses_final[0], N)
    pose_left_interpolate_right = util.interpolate_pose(initial_pose[1], palm_poses_final[1], N)
    #3. Return robot plan
    #5. return final plan
    plan_dict = {}
    plan_dict['palm_poses_world'] = [[pose_left_interpolate_left[x], pose_left_interpolate_right[x]] for x in
                               range(len(pose_left_interpolate_left))]
    plan_dict['palm_poses_l_world'] = pose_left_interpolate_left
    plan_dict['palm_poses_r_world'] = pose_left_interpolate_right
    plan_dict['primitive'] = primitive
    plan_dict['object_poses_world'] = [object_pose] * N
    plan_dict['name'] = plan_name
    plan_dict['t'] = list(np.linspace(0, 1, num=N, endpoint=False))
    plan_dict['N'] = N

    return plan_dict

def move_cart_synchro(palm_poses_final, plan_previous, primitive, plan_name=None, N=100, grasp_width=0.1, is_replan=False):
    final_pose = palm_poses_final[0]
    #1. initialize variables
    poses_initial = plan_previous['palm_poses_world'][-1]
    object_pose = plan_previous['object_poses_world'][-1]
    pose_initial_left_world = poses_initial[0]
    pose_final_left_world = final_pose
    # 2. define object pose relative to gripper frame
    pose_object_rel_gripper = util.convert_reference_frame(object_pose,
                                                        pose_initial_left_world,
                                                        util.unit_pose(),
                                                        frame_id="gripper_left")

    #2. interpolate gripper left pose trajectory
    palm_pose_left_world_list = util.interpolate_pose(pose_initial_left_world, pose_final_left_world, N)
    #3. Loop through gripper poses and compute object poses
    object_pose_list = []
    palm_pose_l_world_list = []
    palm_pose_r_world_list = []
    palm_poses_world_list = []
    for palm_pose_l_world in palm_pose_left_world_list:
        pose_list_world = align_arm_poses(palm_pose_l_world, grasp_width)
        pose_object_rel_world = util.convert_reference_frame(pose_object_rel_gripper,
                                                        util.unit_pose(),
                                                        palm_pose_l_world,
                                                        frame_id="yumi_body")
        #4. append to list
        object_pose_list.append(pose_object_rel_world)
        palm_pose_l_world_list.append(pose_list_world[0])
        palm_pose_r_world_list.append(pose_list_world[1])
        palm_poses_world_list.append([pose_list_world[0], pose_list_world[1]])
    #5. return final plan
    plan_dict = {}
    plan_dict['palm_poses_world'] = palm_poses_world_list
    plan_dict['palm_poses_r_world'] = palm_pose_r_world_list
    plan_dict['palm_poses_l_world'] = palm_pose_l_world_list
    plan_dict['primitive'] = primitive
    plan_dict['grasp_width'] = grasp_width
    plan_dict['object_poses_world'] = object_pose_list
    plan_dict['name'] = plan_name
    plan_dict['t'] = list(np.linspace(0, 1, num=N, endpoint=False))
    plan_dict['N'] = N
    return plan_dict

def grasp_width_from_palm_poses(palm_pose_l, palm_pose_l_r):
    palm_pose_r_palm_l = util.convert_reference_frame(palm_pose_l,
                                                     palm_pose_l_r,
                                                     util.unit_pose(),
                                                     "palm_r")
    return -palm_pose_r_palm_l.pose.position.y


def rotation_center_from_object_poses(corners_object, object_pose_initial, object_pose_final, tol=1e-3):
    #1. define corner poses in object frame
    corner_poses_object_list = []
    corner_poses_initial_world_list = []
    corner_poses_final_world_list = []
    ground_contact_initial_list = []
    ground_contact_final_list = []
    #2. find corners of object (initial and final) in world frame
    #3. check if contact point is in contact with ground. Find the intersection of ground points
    # between initial and final. This is a levering point!
    for corner_point in corners_object:
        corner_pose_object = util.list2pose_stamped([corner_point[0], corner_point[1], corner_point[2], 0, 0, 0, 1])
        corner_pose_initial_world = util.convert_reference_frame(pose_source=corner_pose_object,
                                                                 pose_frame_target=util.unit_pose(),
                                                                 pose_frame_source=object_pose_initial,
                                                                 frame_id="world")
        corner_pose_final_world = util.convert_reference_frame(pose_source=corner_pose_object,
                                                                 pose_frame_target=util.unit_pose(),
                                                                 pose_frame_source=object_pose_final,
                                                                 frame_id="world")
        is_ground_contact_initial = False
        is_ground_contact_final = False
        if abs(corner_pose_initial_world.pose.position.z) < tol:
            is_ground_contact_initial = True
        if abs(corner_pose_final_world.pose.position.z) < tol:
            is_ground_contact_final = True
        corner_poses_object_list.append(corner_pose_object)
        corner_poses_initial_world_list.append(corner_pose_initial_world)
        corner_poses_final_world_list.append(corner_pose_final_world)
        ground_contact_initial_list.append(is_ground_contact_initial)
        ground_contact_final_list.append(is_ground_contact_final)
    for i in range(len(ground_contact_initial_list)):
        if ground_contact_initial_list[i] and ground_contact_final_list[i]:
            index = i
            break

    return corner_poses_initial_world_list[index]
