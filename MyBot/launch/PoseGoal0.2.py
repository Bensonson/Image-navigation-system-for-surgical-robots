#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from ros_igtl_bridge.msg import igtltransform
import numpy as np
from tf.transformations import quaternion_multiply

# Initialize everything
scale_rate = float(input('Enter ROS:3D_Slicer length scaling rate. 10 is recommended for this case.'))
path_length = float(input('Enter the length of the path: (mm)'))
protract = path_length / scale_rate  # Protraction of needle
if protract > 5:
    print('The path is too long for the needle! Try increasing scaling rate.')
    sys.exit(1)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('MyBot', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = 'all'
plan_group = moveit_commander.MoveGroupCommander(group_name)
group_name = 'arm'
move_group = moveit_commander.MoveGroupCommander(group_name)
group_name = 'needle'
end_group = moveit_commander.MoveGroupCommander(group_name)
plan_group.set_max_velocity_scaling_factor(1.0)
plan_group.set_max_acceleration_scaling_factor(1.0)
move_group.set_max_velocity_scaling_factor(1.0)
move_group.set_max_acceleration_scaling_factor(1.0)
end_group.set_max_velocity_scaling_factor(1.0)
end_group.set_max_acceleration_scaling_factor(1.0)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
eef_link = plan_group.get_end_effector_link()

# Go back to initial posture
print('Going back to initial position...')
plan_group.set_named_target('start')
plan_group.go(wait=True)
start_posture = plan_group.get_current_pose().pose


# Listen to IGTL transformation
def get_transformation(transformation_node, scale_rate):
    x = transformation_node.transform.translation.x / scale_rate
    y = transformation_node.transform.translation.y / scale_rate
    z = transformation_node.transform.translation.z / scale_rate
    qx = transformation_node.transform.rotation.x
    qy = transformation_node.transform.rotation.y
    qz = transformation_node.transform.rotation.z
    qw = transformation_node.transform.rotation.w
    translation = np.array([x, y, z])
    rotation = np.array([qx, qy, qz, qw])
    return translation, rotation


print('initialization completed. waiting for IGTL input...')
transformation_node = rospy.wait_for_message("/IGTL_TRANSFORM_IN", igtltransform)
translation, rotation = get_transformation(transformation_node, scale_rate)
print('input received. start calculating the plan...')


# The translation of arm should compensate for injection
def get_needle_translation(length, rotation):
    qx, qy, qz, qw = rotation
    # Calculate the rotation matrix from quaternion
    R = np.array([
        [1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw],
        [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qx * qw],
        [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx ** 2 - 2 * qy ** 2]
    ])
    # Create the vector in x-axis direction with given length
    v = np.array([length, 0, 0])
    # Apply the rotation matrix R to the vector v
    result = np.dot(R, v)
    return result


needle_translation = get_needle_translation(protract, rotation)

# Move the arm
waypoints = []
transform = []

wpose = move_group.get_current_pose(eef_link).pose
wpose.position.x += translation[0] - needle_translation[0]  # Movement compensation
wpose.position.y += translation[1] - needle_translation[1]
wpose.position.z += translation[2] - needle_translation[2]

current_orientation = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
new_orientation = quaternion_multiply(rotation, current_orientation)  # Rotation should stack with starting orientation
wpose.orientation.x = new_orientation[0]
wpose.orientation.y = new_orientation[1]
wpose.orientation.z = new_orientation[2]
wpose.orientation.w = new_orientation[3]
move_group.set_pose_target(wpose)
print('the path is found. moving to target position...')
move_group.go(wpose, wait=True)
move_group.stop()

# Protract the needle
print('The arm is located in target position. Injection...')
joint_goal = end_group.get_current_joint_values()
joint_goal[0] = protract
end_group.go(joint_goal, wait=True)
end_group.stop()

# Print the pose for validation
effector_pose = plan_group.get_current_pose().pose
print("End effector position: ", effector_pose.position)
print("End effector orientation: ", effector_pose.orientation)