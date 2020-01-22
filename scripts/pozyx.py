#!/usr/bin/env python2

'''
Everything here is in 2D right now.
'''

import rospy
import time
from turtlebot3_msgs.msg import *
if (sys.argv[1] == "True" or sys.argv[1] == "true"):
    from gazebo_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import random
import math
import numpy as np
import itertools as it

from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, DeviceList,
                     POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

''' Triangulation functions from IRISbots '''

def get_mid_dist(d1, d2, sep):
    x = (d1**2 / 2) + (d2**2 / 2) - (sep**2 / 4)
    if(x >= 0):
        return math.sqrt(x)
    else:
        return None

def get_angle(d1, d2, sep):
    x = get_mid_dist(d1,d2, sep);

    if x is None:
        return None
    elif x == 0:
        return 0

    ratio = (d2**2 - d1**2) / (2 * sep * x)
    ratio = round(ratio, 4)
    if ratio > 1 or ratio < -1:
        return None
    else:
        return math.asin(ratio)

def triangulate(node_data, a_nodelist):
    rospy.sleep(random.random())
    # print(node_data, a_nodelist)
    anchor_coor = [None]*10
    anchor_id = [None]*10
    num_nodes = 0
    estimates = []

    for node_id in sorted(node_data.keys()):
        if str(node_id) in a_nodelist.keys():
            # print("added")
            anchor_id[num_nodes] = node_id
            anchor_coor[num_nodes] = a_nodelist[str(node_id)]
        num_nodes = num_nodes + 1

    permutelist = list(it.permutations(range(num_nodes), r=3))
    # print(permutelist)
    i = 0
    for item in permutelist:
        i=i+1

        if(i>5):
            break

        # print(item)
        a = anchor_coor[item[0]]
        b = anchor_coor[item[1]]
        c = anchor_coor[item[2]]

        d1 = node_data[anchor_id[item[0]]]
        d2 = node_data[anchor_id[item[1]]]
        d3 = node_data[anchor_id[item[2]]]
        # d1 = node_data[anchor_id[item[0]]]
        # d2 = node_data[anchor_id[item[1]]]
        # d3 = node_data[anchor_id[item[2]]]

        ab_vec = np.subtract(a,b)
        # print(ab_vec)
        ab_mag = np.linalg.norm(ab_vec)

        # print('d1: ' + str(d1))
        # print('d2: ' + str(d2))
        # print('d3: ' + str(d3))
        # print('ab_mag: ' + str(ab_mag))

        # Lilly
        # if((d1 + d2) < ab_mag):
            # continue


        ab_mid = np.add(np.divide(ab_vec, 2), b)

        node_mag = get_mid_dist(d1, d2, ab_mag)
        node_ang = 0

        if ab_vec[0] == 0:
            ab_ang = math.atan2(ab_vec[1],ab_vec[0])
        else:
            ab_ang = math.atan(ab_vec[1]/ab_vec[0])
        # print(ab_vec[1], ab_vec[0], ab_vec[1]/ab_vec[0], math.atan(ab_vec[1]/ab_vec[0]), math.atan2(ab_vec[1],ab_vec[0]))

        if ab_vec[0] < 0 and ab_vec[1] < 0:
            ab_ang += math.pi
        if ab_vec[0] < 0 and ab_vec[1] >= 0:
            ab_ang += math.pi

        node_ang = get_angle(d1, d2, ab_mag)

        if node_ang is None:
            i-=1
            continue

        node_ang1 = ab_ang + math.pi/2 - node_ang
        node_ang2 = ab_ang - math.pi/2 + node_ang

        node_vec1 = [node_mag * math.cos(node_ang1), node_mag * math.sin(node_ang1)]
        node_vec2 = [node_mag * math.cos(node_ang2), node_mag * math.sin(node_ang2)]

        node_vec1 = np.add(node_vec1, ab_mid)
        node_vec2 = np.add(node_vec2, ab_mid)

        # print(str(i)+"***********************")
        # print("AB_ang: "+str(math.degrees(ab_ang)))
        # print("node_ang: " +str(math.degrees(node_ang)))
        # # print(math.degrees(node_ang))
        # print(math.degrees(node_ang1))
        # print(math.degrees(node_ang2))
        # print("AB_mid: "+str(ab_mid))
        # print("C: "+str(c))
        # print(node_vec1)
        # print(node_vec2)


        node_dist1 = abs(np.linalg.norm(np.subtract(c, node_vec1)) - d3)
        node_dist2 = abs(np.linalg.norm(np.subtract(c, node_vec2)) - d3)

        if(node_dist1 < node_dist2):
            estimates.append(node_vec1)
        else:
            estimates.append(node_vec2)

        # print("***********************")

    retval = np.mean(estimates, axis=0).tolist()
    return (round(retval[0], 4), round(retval[1], 4))


def pose_dist(p1, p2):
    return ((p1.position.x-p2.position.x)**2+(p1.position.y-p2.position.y)**2)**0.5

def noise(r):
    if r < 2:
        return max(0, r + random.random()*0.5 - 0.1) # usually overestimates
    if r < 6:
        return r + random.random()*0.2 - 0.1 # almost accurate
    else:
        return r + random.random() - 0.25

class Pozyx():

    def __init__(self,sim,p):
        rospy.init_node('pozyx')
        self.sim = sim
        self.p = p
        self.noise = False

        try:
            rospy.get_param('anchor0')
        except KeyError:
            rospy.loginfo("who is my anchor0?")
            return
        try:
            rospy.get_param('anchor1')
        except KeyError:
            rospy.loginfo("who is my anchor1?")
            return
        try:
            rospy.get_param('anchor2')
        except KeyError:
            rospy.loginfo("who is my anchor2?")
            return

        if self.sim:
            self.truestate_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.truestate_callback)
        self.anchor0_sub = rospy.Subscriber('/'+rospy.get_param('anchor0')+'/pozyx_position', PoseStamped, self.anchor0_callback)
        self.anchor1_sub = rospy.Subscriber('/'+rospy.get_param('anchor1')+'/pozyx_position', PoseStamped, self.anchor1_callback)
        self.anchor2_sub = rospy.Subscriber('/'+rospy.get_param('anchor2')+'/pozyx_position', PoseStamped, self.anchor2_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.pub = rospy.Publisher(rospy.get_namespace()+'pozyx_position', PoseStamped, queue_size=10)

        self.truepose, self.anchor0, self.anchor1, self.anchor2 = None, None, None, None

        self.initialpose = PoseStamped()
        self.initialpose.header.stamp = rospy.Time.now()
        self.initialpose.header.frame_id = rospy.get_namespace()+"base_scan"
        self.initialpose.pose.position.x, self.initialpose.pose.position.y, self.initialpose.pose.position.z = float(sys.argv[2]), float(sys.argv[3]), 0
        q = quaternion_from_euler(0, 0, float(sys.argv[4]))
        self.initialpose.pose.orientation.x, self.initialpose.pose.orientation.y, self.initialpose.pose.orientation.z, self.initialpose.pose.orientation.w = q[0], q[1], q[2], q[3]
        rospy.loginfo("pozyx: setting initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)
        self.mypose = self.initialpose

        # for real pozyx
        self.anchor_device = {"tb3_2":0x6858, "tb3_3":0x685a, "tb3_0":0x685e, "tb3_1":0x6816}
        self.ranges = {0x6858:DeviceRange(), 0x685a:DeviceRange(), 0x685e:DeviceRange(), 0x6816:DeviceRange()}
        self.dist_averages = {0x6858:0, 0x685a:0, 0x685e:0, 0x6816:0}
        self.RSS_averages = {0x6858:0, 0x685a:0, 0x685e:0, 0x6816:0}
        self.n = 10

        while not rospy.is_shutdown():
            self.set_pozyx_position()
            rospy.sleep(0.1) # to do: choose rate

    def odom_callback(self, odometry):
        self.mypose.pose.orientation = odometry.pose.pose.orientation

    def anchor0_callback(self, pose_stamped):
        self.anchor0 = pose_stamped.pose

    def anchor1_callback(self, pose_stamped):
        self.anchor1 = pose_stamped.pose

    def anchor2_callback(self, pose_stamped):
        self.anchor2 = pose_stamped.pose

    def set_pozyx_position(self):
        # locations
        if self.anchor0 and self.anchor1 and self.anchor2:
            coords = {rospy.get_param('anchor0'):(self.anchor0.position.x,self.anchor0.position.y),
                    rospy.get_param('anchor1'):(self.anchor1.position.x,self.anchor1.position.y),
                    rospy.get_param('anchor2'):(self.anchor2.position.x,self.anchor2.position.y)}

            if self.sim:
                # ranges (simulated)
                if self.noise: # switch this back to truepose for gazebo
                    r0 = noise(pose_dist(self.mypose.pose,self.anchor0))
                    r1 = noise(pose_dist(self.mypose.pose,self.anchor1))
                    r2 = noise(pose_dist(self.mypose.pose,self.anchor2))
                else:
                    r0 = pose_dist(self.mypose.pose,self.anchor0)
                    r1 = pose_dist(self.mypose.pose,self.anchor1)
                    r2 = pose_dist(self.mypose.pose,self.anchor2)
                dists = {rospy.get_param('anchor0'):r0, rospy.get_param('anchor1'):r1, rospy.get_param('anchor2'):r2}

            else:
                # ranges (real pozyx)
                try:
                    for i in range(self.n):
                        for dev in list(self.anchor_device.values()):
                            self.p.doRanging(dev,self.ranges[dev])
                            self.dist_averages[dev]+=self.ranges[dev].distance/1000.0
                            self.RSS_averages[dev]+=self.ranges[dev].RSS

                    all_dists = {"tb3_2":self.dist_averages[0x6858]/self.n,
                            "tb3_3":self.dist_averages[0x685a]/self.n,
                            "tb3_0":self.dist_averages[0x685e]/self.n,
                            "tb3_1":self.dist_averages[0x6816]/self.n}

                    dists = {rospy.get_param('anchor0'):all_dists[rospy.get_param('anchor0')],
                            rospy.get_param('anchor1'):all_dists[rospy.get_param('anchor1')],
                            rospy.get_param('anchor2'):all_dists[rospy.get_param('anchor2')]}
                except:
                    return

            pozyxpose = PoseStamped()
            pozyxpose.header.stamp = rospy.Time.now()
            pozyxpose.header.frame_id = rospy.get_namespace()+"base_scan"
            pozyxpose.pose.position.x, pozyxpose.pose.position.y = triangulate(dists,coords)
            if self.truepose: # simulated
                pozyxpose.pose.position.z = self.truepose.pose.position.z
                pozyxpose.pose.orientation = self.mypose.pose.orientation
            else: # real (to do)
                pozyxpose.pose.position.z = 0
                pozyxpose.pose.orientation = self.mypose.pose.orientation
            rospy.loginfo("estimated position: %.3f %.3f", pozyxpose.pose.position.x, pozyxpose.pose.position.y)
            rospy.loginfo("pozyx error: %.3f %.3f", pozyxpose.pose.position.x - self.initialpose.pose.position.x,
                                                   pozyxpose.pose.position.y - self.initialpose.pose.position.y);
            self.pub.publish(pozyxpose)
            self.mypose = pozyxpose

            if self.truepose:
                rospy.loginfo("pozyx error: %.3f %.3f", pozyxpose.pose.position.x - self.truepose.pose.position.x,
                                                       pozyxpose.pose.position.y - self.truepose.pose.position.y);
        else:
            rospy.logdebug("waiting for pozyx, publishing initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)
            self.pub.publish(self.initialpose)

    def truestate_callback(self, states):
        # TODO
        my_index = -1
        for i in range(len(states.name)):
            if states.name[i] == rospy.get_namespace()[1:-1]:
                my_index = i

        if rospy.get_namespace() == "/":
            my_index = len(states.name)-1

        if my_index != -1:
            self.truepose = PoseStamped()
            self.truepose.header.stamp = rospy.Time.now()
            self.truepose.header.frame_id = rospy.get_namespace()+"base_scan"
            self.truepose.pose = states.pose[my_index]

if __name__ == '__main__':
    try:
        sim = (sys.argv[1] == "True" or sys.argv[1] == "true")
        if not sim:
            serial_port = get_first_pozyx_serial_port()
            if serial_port is None:
                print("No Pozyx connected. Check your USB cable or your driver!")
                quit()
            try:
                p = PozyxSerial(serial_port)
            except:
                serial_port = '/dev/ttyACM0'
                p = PozyxSerial(serial_port)
            p.printDeviceInfo()
        else:
            p = None

        Pozyx(sim,p)
    except rospy.ROSInterruptException:
        pass
