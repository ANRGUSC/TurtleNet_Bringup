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
from sensor_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import random
import math
import numpy as np
import itertools as it
from copy import deepcopy
import os

from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, DeviceList,
                     POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port,
                     get_pozyx_ports, NetworkID)

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

    if sep == 0:
        return None

    ratio = (d2**2 - d1**2) / (2 * sep * x)
    ratio = round(ratio, 4)
    if ratio > 1 or ratio < -1:
        return None
    else:
        return math.asin(ratio)

def triangulate(node_data, a_nodelist):
    anchor_coor = list(a_nodelist.values())
    anchor_id = list(a_nodelist.keys())
    num_nodes = len(a_nodelist)
    estimates = []

    permutelist = list(it.permutations(range(num_nodes), r=3))
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

        ab_vec = np.subtract(a,b)
        # print(ab_vec)
        ab_mag = np.linalg.norm(ab_vec)

        # print('d1: ' + str(d1))
        # print('d2: ' + str(d2))
        # print('d3: ' + str(d3))
        # print('ab_mag: ' + str(ab_mag))

        if((d1 + d2) < ab_mag):
            # triangle inequality failed
            continue

        ab_mid = np.add(np.divide(ab_vec, 2), b)

        node_mag = get_mid_dist(d1, d2, ab_mag)
        node_ang = 0

        if ab_vec[0] == 0:
            ab_ang = math.atan2(ab_vec[1],ab_vec[0])
        else:
            ab_ang = math.atan(ab_vec[1]/ab_vec[0])

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

        node_dist1 = abs(np.linalg.norm(np.subtract(c, node_vec1)) - d3)
        node_dist2 = abs(np.linalg.norm(np.subtract(c, node_vec2)) - d3)


        if(node_dist1 <= node_dist2):
            estimates.append(node_vec1)
        else:
            estimates.append(node_vec2)

        # print("***********************")
    if len(estimates) == 0:
        return None
    retval = np.mean(estimates, axis=0).tolist()
    return (round(retval[0], 4), round(retval[1], 4))

def diangulate(node_data, a_nodelist):
    anchor_coor = list(a_nodelist.values())
    anchor_id = list(a_nodelist.keys())
    num_nodes = len(a_nodelist)
    estimates = []

    permutelist = list(it.permutations(range(num_nodes), r=2))
    i = 0
    for item in permutelist:
        i=i+1

        if(i>5):
            break

        # print(item)
        a = anchor_coor[item[0]]
        b = anchor_coor[item[1]]

        d1 = node_data[anchor_id[item[0]]]
        d2 = node_data[anchor_id[item[1]]]

        ab_vec = np.subtract(a,b)
        ab_mag = np.linalg.norm(ab_vec)

#         print('d1: ' + str(d1))
#         print('d2: ' + str(d2))
#         print('ab_mag: ' + str(ab_mag))

        r0 = d1
        r1 = d2
        r01 = ab_mag
        x0, y0 = a[0], a[1]
        x1, y1 = b[0], b[1]

        s = (r0 + r1 + r01)/2
        A = abs(s*(s-r0)*(s-r1)*(s-r01))**0.5
        d = 2*A/r01

        # two possible locations, given r0 and r1
        p0_x = x0 + (r0**2+r01**2-r1**2)*(x1-x0)/(2*r01**2) - d*(y1-y0)/r01
        p0_y = y0 + d*(x1-x0)/r01 + (r0**2+r01**2-r1**2)*(y1-y0)/(2*r01**2)

        p1_x = x0 + (r0**2+r01**2-r1**2)*(x1-x0)/(2*r01**2) + d*(y1-y0)/r01
        p1_y = y0 - d*(x1-x0)/r01 + (r0**2+r01**2-r1**2)*(y1-y0)/(2*r01**2)

        node_vec1 = [p0_x, p0_y]
        node_vec2 = [p1_x, p1_y]
        # print(node_vec1)
        # print(node_vec2)

        if node_vec1[0] > 0:
            estimates.append(node_vec1)
        elif node_vec2[0] > 0:
            estimates.append(node_vec2)

        # print("***********************")
    if len(estimates) == 0:
        return None
    retval = np.mean(estimates, axis=0).tolist()
    return (round(retval[0], 4), round(retval[1], 4))


def pose_dist(p1, p2):
    return ((p1.position.x-p2.position.x)**2+(p1.position.y-p2.position.y)**2)**0.5

def noise(r):
    return max(0, r+random.gauss(0,0.10))

# def noise(r):
#     if r < 2:
#         return max(0, r + random.random()*0.5 - 0.1) # usually overestimates
#     if r < 6:
#         return r + random.random()*0.2 - 0.1 # almost accurate
#     else:
#         return r + random.random() - 0.25
#
# def reverse_noise(r):
#     if r < 2:
#         return r - 0.15
#     if r < 6:
#         return r
#     else:
#         return r - 0.25

class Pozyx():

    def __init__(self,sim,p):
        rospy.init_node('pozyx')
        self.sim = sim
        self.p = p
        if not self.sim and self.p is None:
            rospy.loginfo("No pozyx device!")
        self.denoise = False

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

        # subscribe to the OBSTACLE_DETECTOR for the sake of determining if there is an obstacle obstructing pozyx
        self.laser_sub = rospy.Subscriber('obstacle_detect', LaserScan, self.laser_callback)

        self.pub = rospy.Publisher(rospy.get_namespace()+'pozyx_position', PoseStamped, queue_size=10)

        # for simulation
        self.gazebo_states, self.scan_reading, self.truepose = None, None, None

        # for everything
        self.anchor0, self.anchor1, self.anchor2 = None, None, None

        # for real pozyx
        if not self.sim:
            self.networkId = NetworkID()
            self.p.getNetworkId(self.networkId)
            self.anchor_device = {"tb3_2":0x6858, "tb3_3":0x685a, "tb3_0":0x685e, "tb3_1":0x6816}
            self.ranges = {}
            for tb in self.anchor_device.keys():
                if self.anchor_device[tb] != self.networkId.value:
                    self.ranges[self.anchor_device[tb]] = DeviceRange()

        self.initialpose = PoseStamped()
        self.initialpose.header.stamp = rospy.Time.now()
        self.initialpose.header.frame_id = rospy.get_namespace()+"base_scan"
        self.initialpose.pose.position.x, self.initialpose.pose.position.y, self.initialpose.pose.position.z = float(sys.argv[2]), float(sys.argv[3]), 0
        q = quaternion_from_euler(0, 0, float(sys.argv[4]))
        self.initialpose.pose.orientation.x, self.initialpose.pose.orientation.y, self.initialpose.pose.orientation.z, self.initialpose.pose.orientation.w = q[0], q[1], q[2], q[3]
        rospy.loginfo("pozyx: given initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)

        self.odompose = PoseStamped()

        self.skip_jump_avoidance = (rospy.get_namespace() == "/tb3_3/")

        while not rospy.is_shutdown():
            if not (self.anchor0 and self.anchor1 and self.anchor2):
                self.set_initial_position()
            else:
                rospy.logdebug("RECEIVED ALL ANCHOR POSITIONS")
                self.set_pozyx_position()
            # rospy.sleep(0.02) # to do: choose rate
            rospy.sleep(0.1)

    def set_initial_position(self):
        if not self.sim:
            dists = self.get_pozyx_ranges()
            while dists is None:
                dists = self.get_pozyx_ranges()
            rospy.loginfo(dists)
            if rospy.get_namespace() == "/tb3_0/":
                self.initialpose.pose.position.x, self.initialpose.pose.position.y = 0, 0
                rospy.loginfo("pozyx: calculated initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)
                self.pub.publish(self.initialpose)
            elif rospy.get_namespace() == "/tb3_1/":
                self.initialpose.pose.position.x, self.initialpose.pose.position.y = 0, -dists['tb3_0']
                rospy.loginfo("pozyx: calculated initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)
                self.pub.publish(self.initialpose)
            elif rospy.get_namespace() == "/tb3_2/":
                if self.anchor1:
                    tb3_1y = self.anchor1.position.y
                    my_location = diangulate({"tb3_0":dists['tb3_0'], "tb3_1":dists['tb3_1']},{"tb3_0":(0,0), "tb3_1":(0,tb3_1y)})
                    if my_location:
                        self.initialpose.pose.position.x, self.initialpose.pose.position.y = my_location
                        rospy.loginfo("pozyx: calculated initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)
                        self.pub.publish(self.initialpose)
                else:
                    rospy.loginfo("waiting for anchor1")
        else:
            self.pub.publish(self.initialpose)
        self.previous_pose = deepcopy(self.initialpose)
        self.previous_odompose = deepcopy(self.odompose)

    def truestate_callback(self, states):
        self.gazebo_states = states

    def odom_callback(self, odometry):
        try:
            rospy.logdebug("odom callback")
            self.odompose.header = odometry.header
            self.odompose.pose = odometry.pose.pose
        except:
            pass

    def laser_callback(self, scan):
        rospy.logdebug("scan callback")
        self.scan_reading = scan

    def anchor0_callback(self, pose_stamped):
        rospy.logdebug("anchor0 callback")
        # print(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        self.anchor0 = pose_stamped.pose
        # self.anchor0time = pose_stamped.header.stamp

    def anchor1_callback(self, pose_stamped):
        rospy.logdebug("anchor1 callback")
        # print(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        self.anchor1 = pose_stamped.pose
        # self.anchor1time = pose_stamped.header.stamp

    def anchor2_callback(self, pose_stamped):
        rospy.logdebug("anchor2 callback")
        # print(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        self.anchor2 = pose_stamped.pose
        # self.anchor2time = pose_stamped.header.stamp

    def get_pozyx_ranges(self):
        try:
            if (self.networkId.value == 26718 and 0.0 <= time.time() % 20 < 4.5) or \
                (self.networkId.value == 26646 and 5.0 <= time.time() % 20 < 9.5) or \
                (self.networkId.value == 26712 and 10.0 <= time.time() % 20 < 14.5) or \
                (self.networkId.value == 26714 and 15.0 <= time.time() % 20 < 19.5):
                dist_sums = {}
                for tb in self.anchor_device.keys():
                    if self.anchor_device[tb] != self.networkId.value:
                        dist_sums[self.anchor_device[tb]] = 0
                dist_counts = {}
                for tb in self.anchor_device.keys():
                    if self.anchor_device[tb] != self.networkId.value:
                        dist_counts[self.anchor_device[tb]] = 0
                for i in range(10):
                    for dev in list(self.ranges.keys()):
                        if p.doRanging(dev,self.ranges[dev]):
                            if self.ranges[dev].distance != 0: # distance can't be zero but sometimes the pozyx says it's successful but published zero!!!!!!!!!! a problem
                                if self.denoise: # to do
                                    dist_sums[dev]+=(self.ranges[dev].distance/1000.0-0.2)
                                else:
                                    dist_sums[dev]+=self.ranges[dev].distance/1000.0
                                # RSS_averages[dev]=ranges[dev].RSS
                                dist_counts[dev]+=1
                            else:
                                print("Bad pozyx read:",self.networkId.value,"ranging",dev)
                dists = {}
                for tb in self.anchor_device.keys():
                    if self.anchor_device[tb] != self.networkId.value:
                        if dist_counts[self.anchor_device[tb]] != 0:
                            dists[tb] = round(dist_sums[self.anchor_device[tb]]/dist_counts[self.anchor_device[tb]],2)
                if len(dists) < 2:
                    rospy.loginfo("not all anchors found")
                return dists

        except KeyboardInterrupt:
            rospy.signal_shutdown("pozyx node killed by keyboard interrupt")
            quit()

    # this should be true poses
    def is_obstructed(self, scan, mypose, otherpose):
        deltay = - otherpose.position.y + mypose.position.y
        deltax = otherpose.position.x - mypose.position.x
        angle_ = math.atan2(deltax, deltay)
        yaw = euler_from_quaternion((mypose.orientation.x,mypose.orientation.y,mypose.orientation.z,mypose.orientation.w))[2]
        check = angle_%(2*math.pi) - (math.pi/2 + yaw)
        index = int(round((check-scan.angle_min)/scan.angle_increment))
        epsilon = 0.1
        is_obstructed = scan.ranges[index%360] < (deltay**2+deltax**2)**0.5 - epsilon
        return is_obstructed

    def get_gazebo_pozyx_ranges(self):
        ## check if odompose updates fast enough to be usable
        dists = {}
        r0 = 0
        r1 = 0
        r2 = 0
        # ranges (simulated)
        for i in range(10):
            r0 += noise(pose_dist(self.truepose.pose,self.anchor0_truepose))
            r1 += noise(pose_dist(self.truepose.pose,self.anchor1_truepose))
            r2 += noise(pose_dist(self.truepose.pose,self.anchor2_truepose))
        if self.scan_reading:
            if not self.is_obstructed(self.scan_reading, self.truepose.pose, self.anchor0_truepose):
                dists[rospy.get_param('anchor0')]=r0/10
            if not self.is_obstructed(self.scan_reading, self.truepose.pose, self.anchor1_truepose):
                dists[rospy.get_param('anchor1')]=r1/10
            if not self.is_obstructed(self.scan_reading, self.truepose.pose, self.anchor2_truepose):
                dists[rospy.get_param('anchor2')]=r2/10
        else:
            dists[rospy.get_param('anchor0')]=r0/10
            dists[rospy.get_param('anchor1')]=r1/10
            dists[rospy.get_param('anchor2')]=r2/10
        return dists

    def recover(self, message=""):
        new = self.previous_pose.pose.position.x + (self.odompose.pose.position.x-self.previous_odompose.pose.position.x), self.previous_pose.pose.position.y + (self.odompose.pose.position.y-self.previous_odompose.pose.position.y)
        recoverypose = PoseStamped()
        recoverypose.header.stamp = rospy.Time.now()
        recoverypose.header.frame_id = rospy.get_namespace()+"base_scan" # to do
        recoverypose.pose.position.x, recoverypose.pose.position.y = new
        recoverypose.pose.orientation = self.odompose.pose.orientation ## YAW
        rospy.loginfo(message+": %.3f %.3f", recoverypose.pose.position.x, recoverypose.pose.position.y)
        self.pub.publish(recoverypose)
        return

    def set_pozyx_position(self):
        #################################
        # if self.sim and self.gazebo_states:
        #     if self.load_states(self.gazebo_states):
        #         self.pub.publish(self.truepose)
        #     else:
        #         rospy.loginfo(rospy.get_namespace()+": don't have gazebo ranges yet")
        #     return
        # elif self.sim:
        #     rospy.loginfo(rospy.get_namespace()+": don't have gazebo states yet")
        #     return
        # else:
        #     return
        ##################################
        if self.sim and self.gazebo_states:
            if self.load_states(self.gazebo_states):
                dists = self.get_gazebo_pozyx_ranges()
            else:
                rospy.loginfo(rospy.get_namespace()+": don't have gazebo ranges yet")
                return
        elif self.sim:
            rospy.loginfo(rospy.get_namespace()+": don't have gazebo states yet")
            return
        else:
            rospy.logdebug("ranging...")
            dists = self.get_pozyx_ranges()
            while dists is None:
                dists = self.get_pozyx_ranges()
            rospy.loginfo(dists)
        ####################################
        if len(dists) < 2:
            self.recover("less than 2 anchors")
            return
        else:
            coords = {}
            if rospy.get_param('anchor0') in dists.keys():
                coords[rospy.get_param('anchor0')] = (self.anchor0.position.x,self.anchor0.position.y)
            if rospy.get_param('anchor1') in dists.keys():
                coords[rospy.get_param('anchor1')] = (self.anchor1.position.x,self.anchor1.position.y)
            if rospy.get_param('anchor2') in dists.keys():
                coords[rospy.get_param('anchor2')] = (self.anchor2.position.x,self.anchor2.position.y)
            if len(dists) == 2:
                coords["me"] = (self.previous_pose.pose.position.x,self.previous_pose.pose.position.y)
                dists["me"] = pose_dist(self.previous_odompose.pose,self.odompose.pose)
                self.recover("only 2 anchors")
                return
        my_location = triangulate(dists,coords)
        if my_location is None:
            self.recover("triangulate failed")
            return

        pozyx_distance_traveled = np.linalg.norm(np.subtract(my_location,(self.previous_pose.pose.position.x,self.previous_pose.pose.position.y)))
        odom_distance_traveled = np.linalg.norm(np.subtract((self.previous_odompose.pose.position.x,self.previous_odompose.pose.position.y),(self.odompose.pose.position.x,self.odompose.pose.position.y)))
        if pozyx_distance_traveled > (0.2 + odom_distance_traveled) and not self.skip_jump_avoidance:
            self.recover("jumped ignored")
            return
        ################################################
        pozyxpose = PoseStamped()
        pozyxpose.header.stamp = rospy.Time.now()
        pozyxpose.header.frame_id = rospy.get_namespace()+"base_scan" # to do
        pozyxpose.pose.position.x, pozyxpose.pose.position.y = my_location
        # vec = np.subtract([pozyxpose.pose.position.x, pozyxpose.pose.position.y],[self.previous_pose.pose.position.x,self.previous_pose.pose.position.y])
        # if np.linalg.norm(vec) > 0.15 and False:
        #     print("vec:",vec)
        #     print("yaw:",self.odompose.pose.orientation.x,self.odompose.pose.orientation.y,self.odompose.pose.orientation.z,self.odompose.pose.orientation.w)
        #     yaw = math.atan2(vec[1],vec[0])
        #     q = quaternion_from_euler(0, 0, yaw)
        #     pozyxpose.pose.orientation.x, pozyxpose.pose.orientation.y, pozyxpose.pose.orientation.z, pozyxpose.pose.orientation.w = q[0], q[1], q[2], q[3]
        # else:
        pozyxpose.pose.orientation = self.odompose.pose.orientation
        rospy.loginfo(rospy.get_namespace()+"pozyx estimated position: %.3f %.3f", pozyxpose.pose.position.x, pozyxpose.pose.position.y)
        self.pub.publish(pozyxpose)
        self.previous_pose = deepcopy(pozyxpose)
        self.previous_odompose = deepcopy(self.odompose)

        if self.skip_jump_avoidance:
            self.initialpose = deepcopy(pozyxpose)
            self.skip_jump_avoidance = False
        if self.truepose:
            rospy.logdebug("pozyx error: %.3f %.3f", pozyxpose.pose.position.x - self.truepose.pose.position.x,
                                                   pozyxpose.pose.position.y - self.truepose.pose.position.y);


    def load_states(self, states):
        found = 0
        for index, name in enumerate(states.name):
            if name == rospy.get_namespace()[1:-1]:
                self.truepose = PoseStamped()
                self.truepose.header.stamp = rospy.Time.now()
                self.truepose.header.frame_id = rospy.get_namespace()+"base_scan"
                self.truepose.pose = states.pose[index]
                found +=1
            elif name == rospy.get_param('anchor0'):
                self.anchor0_truepose = Pose()
                self.anchor0_truepose = states.pose[index]
                found += 1
            elif name == rospy.get_param('anchor1'):
                self.anchor1_truepose = Pose()
                self.anchor1_truepose = states.pose[index]
                found +=1
            elif name == rospy.get_param('anchor2'):
                self.anchor2_truepose = Pose()
                self.anchor2_truepose = states.pose[index]
                found +=1
        if found == 4:
            return True
        else:
            return False

if __name__ == '__main__':
    try:
        sim = (sys.argv[1] == "True" or sys.argv[1] == "true")
        if not sim:
            p = None
            ports = get_pozyx_ports()
            for i in range(len(ports)):
                try:
                    if ports[i] != os.environ['CORE_PORT']:
                        p = PozyxSerial(ports[i])
                        break
                except:
                    pass
            if p is None:
                quit()
            p.printDeviceInfo()
        else:
            p = None

        Pozyx(sim,p)
    except rospy.ROSInterruptException:
        pass
