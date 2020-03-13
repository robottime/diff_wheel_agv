#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#maple
#20191127
#190258039@qq.com
import rospy
import geometry_msgs.msg
import tf
from math import pi, fabs, radians, copysign, sqrt, pow, pi, atan2
import sys
import time
from std_srvs.srv import *
from lib.angle import calcRoute,diffAng
import numpy as np

start_nav = False

dis_err = 0.03
ang_err = 0.05

max_linear_vel = 0.2 # 最大线速度
max_angular_vel = 0.5 # 最大角速度
min_linear_vel = 0.05 # 最小线速度
min_angular_vel = 0.1 # 最小角速度

dec_linear_timing = 0.2 # 距目标0.2m内开始减速
dec_angular_timing = 0.5 # 距目标0.5rad内开始减速

base_frame = 'base_footprint'
parent_frame = 'map'
#waypoint type 'move' or outher
#waypoint x(m)
#waypoint y(m)
#waypoint a<=180 and a>=-180
way_points = [
  dict(type ='move', x=0.0,   y=0.5,  a=90),
  dict(type ='move', x=3.0,   y=0.5,  a=0),
  dict(type ='move', x=3.0,   y=1.5,  a=90),
  dict(type ='move', x=4.0,   y=1.5,  a=0),
  dict(type ='move', x=4.0,   y=0.5,  a=-90),
  dict(type ='move', x=0.0,   y=0.5,  a=-180),
  dict(type ='move', x=0.0,   y=0.0,  a=-90),
  dict(type ='move', x=0.0,   y=0.0,  a=0),
  #dict(type ='up'),
  #dict(type ='move', x=3.0,   y=1.5,  a=90),
  #dict(type ='move', x=3.5,  y=0.5,  a=10),
  #dict(type ='move', x=0.2, y=-0.5, a=130),
  #dict(type ='move', x=-1,  y=1,    a=0),
  #dict(type ='down')
  ]
cur_idx = 0

def stop():
	cmd = geometry_msgs.msg.Twist()
	cmd_vel.publish(cmd)
	rospy.sleep(0.1)
	cmd_vel.publish(cmd)

def on_shutdown():
  stop()
 
def handle(req):
  global start_nav
  print'received a request',req.data
  start_nav = req.data
  return([True,'agv_received_command'])

def getPose():
  try:
    (trans, rot) = tf_listener.lookupTransform(parent_frame, base_frame, rospy.Time(0)) # TODO: frame id !!!
    x, y, z = trans
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
  except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    rospy.loginfo("Cannot find transform between base_frame parent_frame")
    return None
  return [x,y,yaw]





if __name__ == '__main__':
  rospy.init_node('autonav')
  rospy.on_shutdown(on_shutdown)
  print "autonav initialize"
  print 'way points:'
  for wp in way_points:
    print wp
  tf_listener = tf.TransformListener()
  cmd_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=5) # TODO: topic namespace
  serve = rospy.Service('auto_nav_start',SetBool,handle)
  if rospy.get_namespace()!='/':
    base_frame = rospy.get_namespace().strip('/')+'/'+base_frame
    parent_frame = rospy.get_namespace().strip('/')+'/'+parent_frame
  client = None
  #try:
  #  rospy.wait_for_service('arm_get_put_start',timeout=5)
  #  client = rospy.ServiceProxy('arm_get_put_start',SetBool)
  #  print "service <arm_get_put_start> connected"
  #except:
  #  print "service <arm_get_put_start> no connect"
  #  pass
  rate = rospy.Rate(60.0)
  while not rospy.is_shutdown():
    try:
      if(cur_idx==len(way_points)):
        start_nav = False
        cur_idx=0
      if start_nav==False:
        stop()
        continue
      #print cur_idx
      target = way_points[cur_idx]
      if target['type'] == 'up':
        stop()
        stop()
        time.sleep(1)
        if client != None:
          client(True)
          print resp.success,resp.message
        rospy.sleep(15)
        cur_idx = (cur_idx + 1)
        continue
      elif target['type'] == 'down':
        stop()
        stop()
        time.sleep(1)
        if client != None:
          resp = client(False)
          print resp.success,resp.message
        rospy.sleep(15)
        cur_idx = (cur_idx + 1)
        continue
      elif target['type'] == 'move':
        if target['a'] > 180 or target['a'] < -180:
          print "you set wrong z range.",target
          stop()
          exit()
        p = getPose()
        goal_a = np.deg2rad(target['a'])
        goal_x = target['x']
        goal_y = target['y']
        goal_da = diffAng(goal_a,p[2])
        goal_distance = sqrt(pow(goal_x - p[0], 2) + pow(goal_y-p[1], 2))
        while (goal_distance > dis_err) or (fabs(goal_da)>ang_err):
          #print 'while',goal_distance,abs(goal_da)
          p = getPose()
          x_start = p[0]
          y_start = p[1]
          rotation = p[2]
          goal_distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
          path_angle = atan2(goal_y - y_start, goal_x- x_start)
          goal_da = diffAng(goal_a,rotation)
          turn_da = 0
          move_cmd = geometry_msgs.msg.Twist()
          move_cmd.linear.x = 0
          move_cmd.angular.z = 0
          if goal_distance < dis_err:
            turn_da = goal_da
          else:
            turn_da = calcRoute(rotation,path_angle,goal_a)
            if(fabs(turn_da)<ang_err):
              dir = -1
              if fabs(diffAng(path_angle,rotation)) <= fabs(2*turn_da):
                dir = 1
              vel1 = min(max_linear_vel*goal_distance/dec_linear_timing, max_linear_vel)
              move_cmd.linear.x = dir*max(vel1, min_linear_vel)
          move_cmd.angular.z = max_angular_vel * turn_da/dec_angular_timing
          if move_cmd.angular.z > 0:
            vel2 = min(move_cmd.angular.z, max_angular_vel)
            move_cmd.angular.z = max(vel2,min_angular_vel)
          else:
            vel3 = max(move_cmd.angular.z, -1*max_angular_vel)
            move_cmd.angular.z = min(vel3,-1*min_angular_vel)
          cmd_vel.publish(move_cmd)
          rate.sleep()
        cur_idx = (cur_idx + 1)
      cmd_vel.publish(geometry_msgs.msg.Twist())
      rate.sleep()
    except:
      cmd_vel.publish(geometry_msgs.msg.Twist())
