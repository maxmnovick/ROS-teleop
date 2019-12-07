#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

g_target_dt_vels = [None, None]
g_dt_vels = [None, None]
g_last_dt_vels = [None, None]
g_last_send_time = None

def send_dt_vels(dt_vels_pub):
  global g_last_dt_vels_send_time, g_last_dt_vels
  t_now = rospy.Time.now()
  g_last_dt_vels_send_time = t_now
  dt_vels_pub.publish(g_last_dt_vels)

def convert_twist(msg, dt_vels_pub):
  global g_last_dt_vels
  if len(msg.data) == 0:
    return # unknown key.
  vr = msg.angular.z * (0.065/2 + 0.1/2)
  vl = msg.angular.z * (0.065/2 - 0.1/2)
  g_dt_vels.d = 
  send_dt_vels(dt_vels_pub)

if __name__ == '__main__':
  rospy.init_node('twist_to_dt_vels')
  dt_vels_pub = rospy.Publisher('dt_vels', Int32MultiArray, queue_size=1) 
  rospy.Subscriber('cmd_vel', Twist, convert_twist, dt_vels_pub)

  rate = rospy.Rate(20)
  while not rospy.is_shutdown():
    send_dt_vels(dt_vels_pub)
    rate.sleep()
