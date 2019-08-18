#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from cw3_helper.srv import ChangeCollisionObject
import tf2_ros
from math import pi
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ContactsState

class cw3_question1_script:

    def __init__(self):

    #--------------------------------------------------------------------------------------------------------------------#

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('cw3_question1')

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.add_to_acm = rospy.ServiceProxy('/add_object_acm', ChangeCollisionObject)
        self.remove_from_acm = rospy.ServiceProxy('/remove_object_acm', ChangeCollisionObject)


        self.sleep_time = 0.5
        self.c_ff = 0; self.c_mf = 0; self.c_rf = 0; self.c_th = 0

    #--------------------------------------------------------------------------------------------------------------------#

        # Robot contains the entire state of the robot (iiwa and shadow hand)
        robot = moveit_commander.RobotCommander()

        # We can get a list of all the groups in the robot
        print('============ Robot Groups:')
        print('{}\n\n'.format(robot.get_group_names()))

        # Planning groups are used to control seperate aspects of the robot.
        # iiwa_group controls the iiwa arm from the base link to the end effector
        # hand group controls all of the joints of the hand.

        self.iiwa_group = moveit_commander.MoveGroupCommander('hand_iiwa')
        self.hand_group = moveit_commander.MoveGroupCommander('sr_hand')

        rospy.sleep(self.sleep_time)

    #--------------------------------------------------------------------------------------------------------------------#

        for object_number in range(3):
            self.get_object(object_number)

        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
    
    #--------------------------------------------------------------------------------------------------------------------#



    def grasp(self, method):

        # Get List of Joint Values
        joint_values = self.hand_group.get_current_joint_values()

        if(method == 'open'):
            print('Open Grasp')
            wrapper = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.2, 1.22, 0.2, 0.2]
        elif(method == 'close'):
            print('Close Grasp')
            wrapper = [-0.05, 0.9, 0.0, 0.4, 0.05, 0.9, 0.0, 0.4, -0.05, 0.9, 0.0, 0.4, 0.5, 1.22, 0.4, 0.1]
        else:
            print('Invalid Grasping Method!')

        # First Finger
        joint_values[0] = wrapper[0] # Knockle - lateral movement - J4 -- 0 to pi/2
        joint_values[1] = wrapper[1] # Proximal - J3 -- 0 to pi/2
        joint_values[2] = wrapper[2] # Middle Joint - J2 -- 0 to pi/2
        joint_values[3] = wrapper[3] # Distal - J1 -- -pi/9 to pi/2

        # Middle Finger
        joint_values[4] = wrapper[4] # Knockle - lateral movement
        joint_values[5] = wrapper[5] # Proximal
        joint_values[6] = wrapper[6] # Middle Joint
        joint_values[7] = wrapper[7] # Distal - J1

        # Ring Finger
        joint_values[8] = wrapper[8] # Knockle - lateral movement
        joint_values[9] = wrapper[9] # Proximal
        joint_values[10] = wrapper[10] # Middle Joint
        joint_values[11] = wrapper[11] # Distal - J1

        # Thumb
        joint_values[12] = wrapper[12] # Base - J5 -- -pi/3 to pi/3
        joint_values[13] = wrapper[13] # Proximal -J4 -- 0 to 70/180pi
        joint_values[14] = wrapper[14] # Middle Joint - J2 -- -2pi/9 to 2pi/9
        joint_values[15] = wrapper[15] # Distal - J1 -- 0 to pi/2

        plan = self.hand_group.plan(joint_values)
        self.hand_group.execute(plan, wait=True)
        rospy.sleep(self.sleep_time)

    def set_orientation(self):

        self.pose.pose.orientation.x = self.object_quat[0]
        self.pose.pose.orientation.y = self.object_quat[1]
        self.pose.pose.orientation.z = self.object_quat[2]
        self.pose.pose.orientation.w = self.object_quat[3]
        print('iiwa planning Orientation: {}'.format(self.pose.pose))
        print
        self.iiwa_group.set_pose_target(self.pose)
        result = self.iiwa_group.plan()
        self.iiwa_group.execute(result, wait=True)
        rospy.sleep(self.sleep_time)

    def set_pose(self, axis, i):

        y_offset = 0.015; x_offset = 0; z_offset_0 = 0.3063972977; z_offset_1 = 0.348930734; z_offset_2 = 0.3113972977
        
        if(axis == 'y'):
            self.pose.pose.position.y = self.object_pose.transform.translation.y + y_offset
            print('For y-axis')
        elif(axis == 'x'):
            self.pose.pose.position.x = self.object_pose.transform.translation.x + x_offset
            print('For x-axis')
        elif(axis == 'z'):
            if(i == 0):
                self.pose.pose.position.z = self.object_pose.transform.translation.z + z_offset_0
            elif(i == 1):
                self.pose.pose.position.z = self.object_pose.transform.translation.z + z_offset_1
            elif(i == 2):
                self.pose.pose.position.z = self.object_pose.transform.translation.z + z_offset_2
            else:
                print('Invalid Object Selected!')
            print('For z-axis')
        else:
            print('Invalid Axis Selected')

        print('iiwa planning to pose: {}'.format(self.pose.pose))
        print
        self.iiwa_group.set_pose_target(self.pose)
        result = self.iiwa_group.plan()
        self.iiwa_group.execute(result, wait=True)
        rospy.sleep(self.sleep_time)

    def set_z_home(self):

        self.pose.pose.position.z = self.z_home
        print('iiwa planning to pose for Z Home Position: {}'.format(self.pose.pose))
        print
        self.iiwa_group.set_pose_target(self.pose)
        result = self.iiwa_group.plan()
        self.iiwa_group.execute(result, wait=True)
        rospy.sleep(self.sleep_time)

    def get_object(self, i):
        
        self.objects = rospy.get_param('object_list')

        while not rospy.is_shutdown():
            try:
                self.object_pose = self.tf_buffer.lookup_transform('world', self.objects[i], rospy.Time.now())
                print('Found object at: {}'.format(self.object_pose.transform))
                success = self.add_to_acm(self.objects[i])
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        
        object_angles = [self.object_pose.transform.rotation.x, self.object_pose.transform.rotation.y, 
                            self.object_pose.transform.rotation.z, self.object_pose.transform.rotation.w]
        
        object_euler = euler_from_quaternion(object_angles)

        self.object_quat = quaternion_from_euler(object_euler[0] + pi, object_euler[1], object_euler[2] + pi/2)

        # Now you can plan a motion to grasp the object
        
        # The robot can be moved according to the planning group

        # Allow replanning to increase the odds of a solution
        self.iiwa_group.allow_replanning(True)
        self.hand_group.allow_replanning(True)

        self.grasp('open')

        self.pose = self.iiwa_group.get_current_pose(self.iiwa_group.get_end_effector_link())
        self.z_home = self.pose.pose.position.z

        self.set_orientation()

        self.set_pose('y', i)
        self.set_pose('x', i)
        self.set_pose('z', i)

        self.grasp('close')
        rospy.sleep(self.sleep_time)

        rospy.Subscriber('/contacts/lh_ff/distal', ContactsState, self.callback_ff)
        rospy.Subscriber('/contacts/lh_mf/distal', ContactsState, self.callback_mf)
        rospy.Subscriber('/contacts/lh_rf/distal', ContactsState, self.callback_rf)
        rospy.Subscriber('/contacts/lh_th/distal', ContactsState, self.callback_th)
        rospy.sleep(1)

        if(i == 0):
            if(self.c_mf >= 1 and self.c_th >= 1):
                print('Good Contact!')
                self.grasp('open')
                self.set_z_home()
            else:
                print('Bad Contact!')
        else:
            if(self.c_ff >= 1 and self.c_mf >= 1 and self.c_rf >= 1 and self.c_th >= 1):
                print('Good Contact!')
                self.grasp('open')
                self.set_z_home()
            else:
                print('Bad Contact!')

        success = self.remove_from_acm(self.objects[i])
    def callback_ff(self, msg):
        if(len(msg.states) > 0):
            self.c_ff += 1
    
    def callback_mf(self, msg):
        if(len(msg.states) > 0):
            self.c_mf += 1

    def callback_rf(self, msg):
        if(len(msg.states) > 0):
            self.c_rf += 1

    def callback_th(self, msg):
        if(len(msg.states) > 0):
            self.c_th += 1

if __name__ == '__main__':
    try:
        cw3_question1_script()
    except rospy.ROSInterruptException:
        pass
