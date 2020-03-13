#!/usr/bin/env python
#maple
#20191127
#190258039@qq.com

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2,fabs
from tf.transformations import euler_from_quaternion
import numpy as np
from angle import calcRoute,diffAng

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""


class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y, goal_a) = self.getkey()
        if goal_a > 180 or goal_a < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_a = np.deg2rad(goal_a)
        goal_da = diffAng(goal_a,rotation)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        while (goal_distance > 0.05) or (fabs(goal_da)>0.05):
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            goal_distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            path_angle = atan2(goal_y - y_start, goal_x- x_start)
            goal_da = diffAng(goal_a,rotation)
            turn_da = 0
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            if goal_distance < 0.05:
                turn_da = goal_da
            else:
                turn_da = calcRoute(rotation,path_angle,goal_a)
                if(fabs(turn_da)<0.05):
                    dir = -1
                    print fabs(diffAng(path_angle,rotation)),turn_da
                    if fabs(diffAng(path_angle,rotation)) <= fabs(2*turn_da):
                        dir = 1
                    move_cmd.linear.x = dir*min(linear_speed * goal_distance, 0.1)
            move_cmd.angular.z = angular_speed * turn_da
            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x, y, z = raw_input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            print(msg)
            GotoPoint()

    except:
        rospy.loginfo("shutdown program.")