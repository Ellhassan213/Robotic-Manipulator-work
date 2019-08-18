#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from cw3_helper.srv import ChangeCollisionObject
import tf2_ros


def cw3_example_script():
    """
    This script will go through the main aspects of moveit and the components you will need to complete the coursework.
    You can find more information on
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cw3_example_script')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Robot contains the entire state of the robot (iiwa and shadow hand)
    robot = moveit_commander.RobotCommander()
    # We can get a list of all the groups in the robot
    print('============ Robot Groups:')
    print('{}\n\n'.format(robot.get_group_names()))

    # Planning groups are used to control seperate aspects of the robot.
    # iiwa_group controls the iiwa arm from the base link to the end effector
    # hand group controls all of the joints of the hand.

    iiwa_group = moveit_commander.MoveGroupCommander('hand_iiwa')
    hand_group = moveit_commander.MoveGroupCommander('sr_hand')

    print('\n\nhand_iiwa Group Information')
    print('============ Reference frame: {}'.format(iiwa_group.get_planning_frame()))
    print('============ End effector: {}\n\n'.format(iiwa_group.get_end_effector_link()))

    print('sr_hand Group Information')
    print('============ Reference frame: {}'.format(hand_group.get_planning_frame()))
    print('============ End effector: {}'.format(hand_group.get_end_effector_link()))

    rospy.sleep(1)

    # The robot can be moved according to the planning group
    pose = iiwa_group.get_current_pose(iiwa_group.get_end_effector_link())
    pose.pose.position.y -= 0.1
  
    print('iiwa planning to pose: {}'.format(pose.pose))
    iiwa_group.set_pose_target(pose)
    result = iiwa_group.plan()
    print('Plan result: {}'.format(result))
    iiwa_group.execute(result, wait=True)

    rospy.sleep(1)

    hand_group.set_named_target('open_grasp')
    plan = hand_group.plan()

    # Alternatively a list of joint values can be given
    # joint_values = hand_group.get_current_joint_values()
    # plan = hand_group.plan(joint_values)
    print('Plan result: {}\n\n'.format(plan))
    hand_group.execute(plan, wait=True)

    # The hand has a very strict controller and will occasionally say it as failed even though it has moved to the right
    # position (just not in the right amount of time), ensure you check the achieved position.


    # Objects in the scene can be found using the object tracker node which publishes on the topic
    # '/recognized_objects_array' and to tf. The list of object names is found in the param '/object_list'

    objects = rospy.get_param('object_list')

    object_pose = tf_buffer.lookup_transform('world', objects[0], rospy.Time.now())
    print('Found object at: {}'.format(object_pose.transform))


    # TF doesn't always return as transform even when the transform exists, try catching the execption, waiting a second
    # and looking up the transform again.

    # To grasp objects they must first be added to the allowed collision matrix so that the path planner knows to ignore
    #  the collision. You can do this using a service '/add_object_acm', they can also be removed from the acm after
    # grasping to prevent any unintended collisions.

    add_to_acm = rospy.ServiceProxy('/add_object_acm', ChangeCollisionObject)
    remove_from_acm = rospy.ServiceProxy('/remove_object_acm', ChangeCollisionObject)

    success = add_to_acm(objects[0])

    # Now you can plan a motion to grasp the object

    rospy.sleep(10)

    success = remove_from_acm(objects[0])

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        #rospy.sleep(10)
        cw3_example_script()
    except rospy.ROSInterruptException:
        pass