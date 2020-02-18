#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
import message_filters
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion

import csv
import sys

def callback0(states, x, y, theta):
  # write
  for index, name in enumerate(states.name):
      if name == 'tb3_0':
          true_x = states.pose[index].position.x
          true_y = states.pose[index].position.y
          true_theta = euler_from_quaternion((states.pose[index].orientation.x,states.pose[index].orientation.y,states.pose[index].orientation.z,states.pose[index].orientation.w))[2]
  with open(FILENAME,'a') as f:
      writer = csv.writer(f)
      writer.writerow([str(rospy.Time.now()),'tb3_0',str(true_x),str(true_y),str(true_theta),str(x.data),str(y.data),str(theta.data)])
  return

def callback1(states, x, y, theta):
  # write
  for index, name in enumerate(states.name):
      if name == 'tb3_1':
          true_x = states.pose[index].position.x
          true_y = states.pose[index].position.y
          true_theta = euler_from_quaternion((states.pose[index].orientation.x,states.pose[index].orientation.y,states.pose[index].orientation.z,states.pose[index].orientation.w))[2]
  with open(FILENAME,'a') as f:
      writer = csv.writer(f)
      writer.writerow([str(rospy.Time.now()),'tb3_1',str(true_x),str(true_y),str(true_theta),str(x.data),str(y.data),str(theta.data)])
  return

def callback2(states, x, y, theta):
  # write
  for index, name in enumerate(states.name):
      if name == 'tb3_2':
          true_x = states.pose[index].position.x
          true_y = states.pose[index].position.y
          true_theta = euler_from_quaternion((states.pose[index].orientation.x,states.pose[index].orientation.y,states.pose[index].orientation.z,states.pose[index].orientation.w))[2]
  with open(FILENAME,'a') as f:
      writer = csv.writer(f)
      writer.writerow([str(rospy.Time.now()),'tb3_2',str(true_x),str(true_y),str(true_theta),str(x.data),str(y.data),str(theta.data)])
  return

def callback3(states, x, y, theta):
  # write
  for index, name in enumerate(states.name):
      if name == 'tb3_3':
          true_x = states.pose[index].position.x
          true_y = states.pose[index].position.y
          true_theta = euler_from_quaternion((states.pose[index].orientation.x,states.pose[index].orientation.y,states.pose[index].orientation.z,states.pose[index].orientation.w))[2]
  with open(FILENAME,'a') as f:
      writer = csv.writer(f)
      writer.writerow([str(rospy.Time.now()),'tb3_3',str(true_x),str(true_y),str(true_theta),str(x.data),str(y.data),str(theta.data)])
  return

def log_position():
    rospy.init_node('log_position', anonymous=True)
    rospy.loginfo("initializing position logger...")

    gazebo_sub = message_filters.Subscriber('/gazebo/model_states', ModelStates)

    tb3_0_x_sub = message_filters.Subscriber('/tb3_0/position_estimate_x', Float64)
    tb3_0_y_sub = message_filters.Subscriber('/tb3_0/position_estimate_y', Float64)
    tb3_0_theta_sub = message_filters.Subscriber('/tb3_0/position_estimate_theta', Float64)
    tb3_1_x_sub = message_filters.Subscriber('/tb3_1/position_estimate_x', Float64)
    tb3_1_y_sub = message_filters.Subscriber('/tb3_1/position_estimate_y', Float64)
    tb3_1_theta_sub = message_filters.Subscriber('/tb3_1/position_estimate_theta', Float64)
    tb3_2_x_sub = message_filters.Subscriber('/tb3_2/position_estimate_x', Float64)
    tb3_2_y_sub = message_filters.Subscriber('/tb3_2/position_estimate_y', Float64)
    tb3_2_theta_sub = message_filters.Subscriber('/tb3_2/position_estimate_theta', Float64)
    tb3_3_x_sub = message_filters.Subscriber('/tb3_3/position_estimate_x', Float64)
    tb3_3_y_sub = message_filters.Subscriber('/tb3_3/position_estimate_y', Float64)
    tb3_3_theta_sub = message_filters.Subscriber('/tb3_3/position_estimate_theta', Float64)

    tb3_0 = message_filters.ApproximateTimeSynchronizer([gazebo_sub, tb3_0_x_sub, tb3_0_y_sub, tb3_0_theta_sub], 10, 0.1, allow_headerless=True)
    tb3_0.registerCallback(callback0)

    tb3_1 = message_filters.ApproximateTimeSynchronizer([gazebo_sub, tb3_1_x_sub, tb3_1_y_sub, tb3_1_theta_sub], 10, 0.1, allow_headerless=True)
    tb3_1.registerCallback(callback1)

    tb3_2 = message_filters.ApproximateTimeSynchronizer([gazebo_sub, tb3_2_x_sub, tb3_2_y_sub, tb3_2_theta_sub], 10, 0.1, allow_headerless=True)
    tb3_2.registerCallback(callback2)

    tb3_3 = message_filters.ApproximateTimeSynchronizer([gazebo_sub, tb3_3_x_sub, tb3_3_y_sub, tb3_3_theta_sub], 10, 0.1, allow_headerless=True)
    tb3_3.registerCallback(callback3)

    rospy.spin()

if __name__ == '__main__':
    FILENAME = sys.argv[1]
    log_position()
