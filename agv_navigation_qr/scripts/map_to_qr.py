#!/usr/bin/env python
import rospy

import tf

if __name__=='__main__':
  rospy.init_node("map_to_qr_tf_broadcaster")
  
  br = tf.TransformBroadcaster()
  xsp = rospy.get_param('/map_xspacing')
  ysp = rospy.get_param('/map_yspacing')
  map_points = rospy.get_param('/tagmap/points')
  
  for point in map_points:
    frame = 'tag_' + str(point['tag_id'])
    x = point['row'] * xsp
    y = point['col'] * ysp
    rospy.loginfo("%s: (%f, %f)", frame, x, y)

  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    for point in map_points:
      tag_frame = 'tag_' + str(point['tag_id'])
      x = point['row'] * xsp
      y = point['col'] * ysp
      br.sendTransform((x, y, 0),
                       tf.transformations.quaternion_from_euler(0, 0, -1.5708),
                       rospy.Time.now(),
                       tag_frame,
                       "map")
    rate.sleep()
  rospy.spin()
