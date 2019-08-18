#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from cw3_helper.srv import ChangeCollisionObject
import tf
import tf2_ros
from sympy import *
from math import pi
import numpy as np

class cw3_question3_script():
    def __init__(self):

        #---------------------------------------------------------------------------------------------------#

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('cw3_question3_script')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot = moveit_commander.RobotCommander()

        self.iiwa_group = moveit_commander.MoveGroupCommander('object_iiwa')
        self.centre_of_mass_offset = np.array([[0,-0.03,0.12], [0.0003,0.059,0.042], [0,0.03,0.13],
                                                [0,0.067,0.034], [0.0001,0.021,0.076], 
                                                    [0,0.0006,0.0004], [0,0,0.02]])

        self.box_mass = Symbol('m')
        self.mass = [4, 4, 3, 2.7, 1.7, 1.8, 0.3]
        self.box_centre_of_mass = np.array([0, 0, Symbol('z')])
        self.theta = np.zeros(7)
        self.velocity = np.zeros(7)
        self.torque = np.zeros(7)        
        self.joint_num = 7
        self.sleep_time = 0.2

        self.joint_subscriber = rospy.Subscriber('/joint_states', JointState, self.callback)


        #---------------------------------------------------------------------------------------------------#

        self.move()
        rospy.sleep(5.0)
        self.get_mass()

        #---------------------------------------------------------------------------------------------------#


        # # ------------ theta,        d,        alpha,    a
        # self.dh_param = [[0,    0.1575,            0,   0],
        #                  [pi,   0.2025,     0.5 * pi,   0],
        #                  [pi,        0,     0.5 * pi,   0],
        #                  [0,      0.42,     0.5 * pi,   0],
        #                  [pi,        0,     0.5 * pi,   0],
        #                  [0,      0.40,     0.5 * pi,   0],
        #                  [pi,        0,     0.5 * pi,   0]]

        #---------------------------------------------------------------------------------------------------#

    def callback(self, msg):
        for i in range(self.joint_num):
            self.theta[i] = msg.position[i]
            self.velocity[i] = msg.velocity[i]
            self.torque[i] = msg.effort[i]

    def get_Jakobi(self, i):

        start = 1
        T = []
        while start <= self.joint_num:
            link = link = 'object_iiwa_link_' + str(start)
            while not rospy.is_shutdown():
                try:
                    pose = self.tf_buffer.lookup_transform('object_iiwa_link_0', link, rospy.Time())
                    break
                except:
                    rospy.sleep(self.sleep_time)

            trans = pose.transform.translation
            quat = pose.transform.rotation

            x = quat.x; y = quat.y; z = quat.z; w = quat.w
            R1 = [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w]
            R2 = [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z-2 * x * w]
            R3 = [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2]

            rotation = np.matrix([R1, R2, R3])

            translation = np.array([trans.x, trans.y, trans.z])
            
            T.append([translation, rotation])

            start = start + 1

        if  i < self.joint_num + 1:
            link = 'object_iiwa_link_' + str(i)
        else:
            link = 'object_link_0'

        while not rospy.is_shutdown():
            try:
                pose = self.tf_buffer.lookup_transform('object_iiwa_link_0', link, rospy.Time())
                break
            except:
                rospy.sleep(self.sleep_time)

        transl = pose.transform.translation

        if i < self.joint_num + 1:
            delta_p = T[i-1][1].dot(self.centre_of_mass_offset[i-1])
            delta_p = np.asarray(delta_p)[0]
            on = np.array([transl.x, transl.y, transl.z]) + delta_p
        else:
            delta_p = T[6][1].dot(self.box_centre_of_mass)
            delta_p = np.asarray(delta_p)[0]
            on = np.array([transl.x, transl.y, transl.z]) + delta_p

        o0 = T[0][0]
        o1 = T[1][0]
        o2 = T[2][0]
        o3 = T[3][0]
        o4 = T[4][0]
        o5 = T[5][0]
        o6 = T[6][0]

        z0 = np.array([T[0][1][0,2], T[0][1][1,2], T[0][1][2,2]])
        z1 = np.array([T[1][1][0,2], T[1][1][1,2], T[1][1][2,2]])
        z2 = np.array([T[2][1][0,2], T[2][1][1,2], T[2][1][2,2]])
        z3 = np.array([T[3][1][0,2], T[3][1][1,2], T[3][1][2,2]])
        z4 = np.array([T[4][1][0,2], T[4][1][1,2], T[4][1][2,2]])
        z5 = np.array([T[5][1][0,2], T[5][1][1,2], T[5][1][2,2]])
        z6 = np.array([T[6][1][0,2], T[6][1][1,2], T[6][1][2,2]])

        if i == 1:
            J1 = np.cross(z0, (on - o0))
            J2 = np.array([0, 0, 0])
            J3 = np.array([0, 0, 0])
            J4 = np.array([0, 0, 0])
            J5 = np.array([0, 0, 0])
            J6 = np.array([0, 0, 0])
            J7 = np.array([0, 0, 0])
        elif i == 2:
            J1 = np.cross(z0, (on - o0))
            J2 = np.cross(z1, (on - o1))
            J3 = np.array([0, 0, 0])
            J4 = np.array([0, 0, 0])
            J5 = np.array([0, 0, 0])
            J6 = np.array([0, 0, 0])
            J7 = np.array([0, 0, 0])
        elif i == 3:
            J1 = np.cross(z0, (on - o0))
            J2 = np.cross(z1, (on - o1))
            J3 = np.cross(z2, (on - o2))
            J4 = np.array([0, 0, 0])
            J5 = np.array([0, 0, 0])
            J6 = np.array([0, 0, 0])
            J7 = np.array([0, 0, 0])
        elif i == 4:
            J1 = np.cross(z0, (on - o0))
            J2 = np.cross(z1, (on - o1))
            J3 = np.cross(z2, (on - o2))
            J4 = np.cross(z3, (on - o3))
            J5 = np.array([0, 0, 0])
            J6 = np.array([0, 0, 0])
            J7 = np.array([0, 0, 0])
        elif i == 5:
            J1 = np.cross(z0, (on - o0))
            J2 = np.cross(z1, (on - o1))
            J3 = np.cross(z2, (on - o2))
            J4 = np.cross(z3, (on - o3))
            J5 = np.cross(z4, (on - o4))
            J6 = np.array([0, 0, 0])
            J7 = np.array([0, 0, 0])
        elif i == 6:
            J1 = np.cross(z0, (on - o0))
            J2 = np.cross(z1, (on - o1))
            J3 = np.cross(z2, (on - o2))
            J4 = np.cross(z3, (on - o3))
            J5 = np.cross(z4, (on - o4))
            J6 = np.cross(z5, (on - o5))
            J7 = np.array([0, 0, 0])
        elif i == 7 or i == 8:
            J1 = np.cross(z0, (on - o0))
            J2 = np.cross(z1, (on - o1))
            J3 = np.cross(z2, (on - o2))
            J4 = np.cross(z3, (on - o3))
            J5 = np.cross(z4, (on - o4))
            J6 = np.cross(z5, (on - o5))
            J7 = np.cross(z6, (on - o6))

        row1 = [J1[0], J2[0], J3[0], J4[0], J5[0], J6[0], J7[0]]
        row2 = [J1[1], J2[1], J3[1], J4[1], J5[1], J6[1], J7[1]]
        row3 = [J1[2], J2[2], J3[2], J4[2], J5[2], J6[2], J7[2]]

        Jakobi = np.matrix([row1, row2, row3])

        return Jakobi

    def get_mass(self):

        gravity = 9.8

        J1 = self.get_Jakobi(1)
        J2 = self.get_Jakobi(2)
        J3 = self.get_Jakobi(3)
        J4 = self.get_Jakobi(4)
        J5 = self.get_Jakobi(5)
        J6 = self.get_Jakobi(6)
        J7 = self.get_Jakobi(7)
        J8 = self.get_Jakobi(8)
        
        t_6 = self.mass[5] * gravity * J6[2, 5] + self.mass[6] * gravity * J7[2, 5] + self.box_mass * gravity * J8[2, 5]
        t_4 = self.mass[3] * gravity * J4[2, 3] + self.mass[4] * gravity * J5[2, 3] + self.mass[5] * gravity * J6[2, 3] + self.mass[6] * gravity * J7[2, 3] + self.box_mass * gravity * J8[2, 3]

        c_of_mass = solve([(t_6 - self.torque[5]), (t_4 - self.torque[3])], [Symbol('m'), Symbol('z')])
        mass = c_of_mass[0][0]
        centre_of_mass = c_of_mass[0][1]

        print('Calculated Mass: %f' % (mass))
        print('Calculated Centre of Mass: (0, 0, %f)' % (centre_of_mass))

        rospy.logwarn('CALCULATED MASS!!!: %f' % (mass))
        rospy.logwarn('CALCULATED CENTRE OF MASS!!!: (0, 0, %f)' % (centre_of_mass))
        
    def move(self):
        j = [0.8, 0.95, 0.0, 0.2, 0.0, 0.5, 0.9]
        plan = self.iiwa_group.plan(j)
        self.iiwa_group.execute(plan, wait=True)
        while True:
            try:
                transform = self.tf_buffer.lookup_transform('object_iiwa_link_0','object_link_0', rospy.Time())
                break
            except:
                rospy.sleep(self.sleep_time)

if __name__ == '__main__':

    try:
        cw3_question3_script()
    except rospy.ROSInterruptException:
        pass