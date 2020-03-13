#!/usr/bin/env python
from lib.yz_motor_interface import YZMotorInterface
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from threading import Timer, Lock
#from apriltag_ros.msg import AprilTagDetectionArray

import math
import time

wheel_diam = 0.1
DIAGONAL_L = 0.373
BODY_WIDTH = 0.285
kWheelGearRatio = 1.0
kWheelMPerRad = math.pi * wheel_diam / 2.0 / math.pi
kWheelStepPerM = 32768/math.pi/wheel_diam #step/m
kWheelMPerStep = 1.0/kWheelStepPerM #m/step
kMotorMaxRVel = 1500.0 #r/min
kMotorMaxAngularVel = kMotorMaxRVel/60*2*math.pi #rad/s
kTurningAdjustRate = 1.08
kEquivalentTread = (DIAGONAL_L*DIAGONAL_L/BODY_WIDTH)*kTurningAdjustRate #m
feed_back = None
positions_init = [0,0,0,0]
positions_curr = [0,0,0,0]
motor_dir = [-1,1,-1,1]
parent_frame = 'odom'
base_frame = 'base_footprint'
d_step = [0,0,0,0]
last_read_time = time.time()
read_dt = None
current_x = 0
current_y = 0
current_a = 0

motors = YZMotorInterface("yz_motor",'/dev/rs485',False)
lock = Lock()

def _callback(msg):
  #print msg
  set_vel(msg.linear.x, msg.linear.y, msg.angular.z)

def _terminate():
  #print time.time(),"_terminate"
  set_vel(0, 0, 0)
  time.sleep(1)
  motors.end()
  print "end"

# TODO change to brake()
def stop():
  set_vel(0, 0, 0)

#vx: m/s; vy: m/s; w: rad/s
def set_vel(vx, vy, w):
	vel2Avel = lambda v: v / kWheelMPerRad
	v0 = vel2Avel(vx+vy)
	v1 = vel2Avel(vx-vy)
	vw = vel2Avel(w * kEquivalentTread / 2.0)
	if(
		math.fabs(v1 - vw)>kMotorMaxAngularVel or
		math.fabs(v0 + vw)>kMotorMaxAngularVel or
		math.fabs(v0 - vw)>kMotorMaxAngularVel or
		math.fabs(v1 + vw)>kMotorMaxAngularVel
		):
		return

	avel2Speed = lambda v, d=1: d * v / 2.0 / math.pi * 60.0
	lock.acquire()
	motors.setSpeed(avel2Speed(v1 - vw, -1),1)
	motors.setSpeed(avel2Speed(v0 + vw, 1),2)
	motors.setSpeed(avel2Speed(v0 - vw, -1),3)
	motors.setSpeed(avel2Speed(v1 + vw, 1),4)
	#print vx,vy,w, avel2Speed(v1 - vw, -1), avel2Speed(v0 + vw, 1), avel2Speed(v0 - vw, -1), avel2Speed(v1 + vw, 1)
	lock.release()

def read_encoder():
  global d_step, positions_curr, last_read_time,read_dt
  lock.acquire()
  ct = time.time()
  read_dt = ct - last_read_time
  last_read_time = ct
  for i in range(1,5):
    p = None
    t = time.time()
    while p==None:
      p = motors.getPosition(i)
      if p==None:
        time.sleep(0.2)
      if (time.time()-t)>1.0:
        print "Motor {} get position fial,ros shutdown".format(i)
        lock.release()
        raise Exception
    p = (p - positions_init[i-1])*motor_dir[i-1]
    d_step[i-1] = p-positions_curr[i-1]
    positions_curr[i-1] = p
  lock.release()

def pub_odom(x, y, a, vx, vy, va):
  quat = tf.transformations.quaternion_from_euler(0, 0, a)
  tf_broadcaster.sendTransform((x, y, 0.), quat, rospy.Time.now(), base_frame, parent_frame)
  odom = Odometry()
  odom.header.stamp = rospy.Time.now()
  odom.header.frame_id = parent_frame
  odom.child_frame_id = base_frame
  odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*quat))
  odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, va))
  odom_pub.publish(odom)

def calc_odom():
  global 	current_a,current_x,current_y,reda_dt
  read_encoder() 
  xx = (d_step[0]+d_step[1]+d_step[2]+d_step[3])/4.0/kWheelGearRatio*kWheelMPerStep
  yy = (d_step[1]+d_step[2]-d_step[0]-d_step[3])/4.0/kWheelGearRatio*kWheelMPerStep
  da = (d_step[1]+d_step[3]-d_step[0]-d_step[2])/2.0/kWheelGearRatio*kWheelMPerStep/(kEquivalentTread)

  dx = (xx*math.cos(current_a) - yy*math.sin(current_a))
  current_x += dx
  dy = (xx*math.sin(current_a) + yy*math.cos(current_a))
  current_y += dy
  current_a += da
  vx = dx/read_dt
  vy = dy/read_dt
  va = da/read_dt
  pub_odom(current_x, current_y, current_a, vx, vy, va)

def getEnableState():
  state = True
  lock.acquire()
  for addr in range(1,5):
    #print 'motor',addr
    if motors.isEnable(addr)==False:
      if motors.isEnable(addr)==False:
        state = False
        print ("Motors {} DisEnable".format(addr))
  lock.release()
  return state

def calibrate_pos(msg):
  global parent_frame, current_a, current_x, current_y,read_dt
  if not msg.detections:
    return
  detection = msg.detections[0]
  tags = detection.id
  if not tags:
    return
  parent_frame = "tag_{}".format(tags[0])
  read_encoder() 
  xx = (d_step[0]+d_step[1]+d_step[2]+d_step[3])/4.0/kWheelGearRatio*kWheelMPerStep
  yy = (d_step[1]+d_step[2]-d_step[0]-d_step[3])/4.0/kWheelGearRatio*kWheelMPerStep
  da = (d_step[1]+d_step[3]-d_step[0]-d_step[2])/2.0/kWheelGearRatio*kWheelMPerStep/(kEquivalentTread)

  dx = (xx*math.cos(current_a) - yy*math.sin(current_a))
  dy = (xx*math.sin(current_a) + yy*math.cos(current_a))
  vx = dx/read_dt
  vy = dy/read_dt
  va = da/read_dt

  current_x = -detection.pose.pose.pose.position.y
  current_y = -detection.pose.pose.pose.position.x
  quat = detection.pose.pose.pose.orientation
  explicit_quat = [quat.x, quat.y, quat.z, quat.w]
  r,p,y = tf.transformations.euler_from_quaternion(explicit_quat)
  current_a = y

  pub_odom(current_x, current_y, current_a, vx, vy, va)



if __name__=='__main__':
  rospy.init_node("four_mecanum_wheel_node")
  if motors.begin()==False:
    print 'serial /dev/rs485 open fial'
    exit()
  tb = time.time()
  for addr in range(1,5):
    if motors.busModeEn(1,addr)==False:
      print 'motor '+str(addr)+' busmod enable fail'
      motors.end()
      exit()
    p = None
    while p==None:
      p = motors.getPosition(addr)
      if time.time()-tb>2:
        print('motor '+str(addr)+' initialize fail')
        motors.end()
        exit()
    positions_init[addr-1] = p
    motors.getAll(addr)
  print ('initialize succeed')
  if rospy.get_namespace()!='/':
    base_frame = rospy.get_namespace().strip('/')+'/'+base_frame
    parent_frame = rospy.get_namespace().strip('/')+'/'+parent_frame
  odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
  tf_broadcaster = tf.TransformBroadcaster()
  rospy.Subscriber("cmd_vel",Twist,_callback,queue_size = 1)
  #rospy.Subscriber("tag_detections", AprilTagDetectionArray, calibrate_pos, queue_size = 1)
  rate = rospy.Rate(10)
  print "begin"
  while not rospy.is_shutdown():
    try:
      if getEnableState()==False:
        print "Motor xxx is DisEnable"
        raise Exception
      calc_odom()
      rate.sleep()
    except:
      print time.time(), "terminate"
      _terminate()
      exit()
  rospy.spin()
  print "shutdown"
  _terminate()




