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
protract = path_length / scale_rate
if protract > 5:
    print('The path is too long for the needle! Try increasing scaling rate, or setting a lower max_length for path planning.')
    sys.exit(1)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('MyBot', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = 'all'
plan_group = moveit_commander.MoveGroupCommander(group_name)
plan_group.set_max_velocity_scaling_factor(1.0)
plan_group.set_max_acceleration_scaling_factor(1.0)
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


print('Initialization completed. waiting for IGTL input...')
transformation_node = rospy.wait_for_message("/IGTL_TRANSFORM_IN", igtltransform)
translation, rotation = get_transformation(transformation_node, scale_rate)
print('Input received. start executing the plan...')


# Move the arm
waypoints = []
transform = []
wpose = plan_group.get_current_pose(eef_link).pose
wpose.position.x += translation[0]
wpose.position.y += translation[1]
wpose.position.z += translation[2]
current_orientation = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
new_orientation = quaternion_multiply(rotation, current_orientation)  # Rotation should stack with starting orientation
wpose.orientation.x = new_orientation[0]
wpose.orientation.y = new_orientation[1]
wpose.orientation.z = new_orientation[2]
wpose.orientation.w = new_orientation[3]
plan_group.set_pose_target(wpose)
print('The path is found. Moving to target position...')
plan_group.go(wpose, wait=True)
plan_group.stop()


# Print the pose for validation
effector_pose = plan_group.get_current_pose().pose
print("End effector position: ", effector_pose.position)
print("End effector orientation: ", effector_pose.orientation)




