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

        if node_vec1[1] > 0:
            estimates.append(node_vec1)
        elif node_vec2[1] > 0:
            estimates.append(node_vec2)

        # print("***********************")
    if len(estimates) == 0:
        return None
    retval = np.mean(estimates, axis=0).tolist()
    return (round(retval[0], 4), round(retval[1], 4))


def pose_dist(p1, p2):
    return ((p1.position.x-p2.position.x)**2+(p1.position.y-p2.position.y)**2)**0.5

def noise(r):
    return max(0, r+random.gauss(0,0.1))

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
        self.noise = True
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
        self.gazebo_states, self.scan_reading = None, None

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

        while not rospy.is_shutdown():
            if not (self.anchor0 and self.anchor1 and self.anchor2):
                self.set_initial_position()
            else:
                self.set_pozyx_position()
            rospy.sleep(0.1) # to do: choose rate

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
                self.initialpose.pose.position.x, self.initialpose.pose.position.y = dists['tb3_0'], 0
                rospy.loginfo("pozyx: calculated initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)
                self.pub.publish(self.initialpose)
            elif rospy.get_namespace() == "/tb3_2/":
                while self.anchor1 is None:
                    pass
                tb3_1x = self.anchor1.position.x
                self.initialpose.pose.position.x, self.initialpose.pose.position.y = diangulate({"tb3_0":dists['tb3_0'], "tb3_1":dists['tb3_1']},{"tb3_0":(0,0), "tb3_1":(tb3_1x,0)})
                rospy.loginfo("pozyx: calculated initial pose (%f, %f)", self.initialpose.pose.position.x, self.initialpose.pose.position.y)
                self.pub.publish(self.initialpose)
        else:
            self.pub.publish(self.initialpose)

    def truestate_callback(self, states):
        self.gazebo_states = states

    def odom_callback(self, odometry):
        rospy.logdebug("odom callback")
        self.odompose.header = odometry.header
        self.odompose.pose = odometry.pose.pose

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
            # dist_sums = {0x6858:0, 0x685a:0, 0x685e:0, 0x6816:0}
            if (self.networkId.value == 26712 and 0 <= time.time() % 4 < 0.5) or (self.networkId.value == 26714 and 1.0 <= time.time() % 4 < 1.5) or \
                (self.networkId.value == 26718 and 2.0 <= time.time() % 4 < 2.5) or (self.networkId.value == 26646 and 3.0 <= time.time() % 4 < 3.5):
                dist_sums = {}
                for tb in self.anchor_device.keys():
                    if self.anchor_device[tb] != self.networkId.value:
                        dist_sums[self.anchor_device[tb]] = 0
                # dist_counts = {0x6858:0, 0x685a:0, 0x685e:0, 0x6816:0}
                dist_counts = {}
                for tb in self.anchor_device.keys():
                    if self.anchor_device[tb] != self.networkId.value:
                        dist_counts[self.anchor_device[tb]] = 0
                for i in range(10):
                    for dev in list(self.ranges.keys()):
                        if p.doRanging(dev,self.ranges[dev]):
                            if self.denoise: # to do
                                dist_sums[dev]+=(self.ranges[dev].distance/1000.0-0.2)
                            else:
                                dist_sums[dev]+=self.ranges[dev].distance/1000.0
                            # RSS_averages[dev]=ranges[dev].RSS
                            dist_counts[dev]+=1
                try:
                    # dists = {"tb3_2":round(dist_sums[0x6858]/dist_counts[0x6858],2),
                    #         "tb3_3":round(dist_sums[0x685a]/dist_counts[0x685a],2),
                    #         "tb3_0":round(dist_sums[0x685e]/dist_counts[0x685e],2),
                    #         "tb3_1":round(dist_sums[0x6816]/dist_counts[0x6816],2)}
                    dists = {}
                    for tb in self.anchor_device.keys():
                        if self.anchor_device[tb] != self.networkId.value:
                            dists[tb] = round(dist_sums[self.anchor_device[tb]]/dist_counts[self.anchor_device[tb]],2)
                except ZeroDivisionError:
                    rospy.loginfo("can't range with someone")
                    rospy.loginfo(dist_counts)
                    return None
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
        # if is_obstructed:
        #     if rospy.get_namespace() == "/tb3_0/":
        #         print("scan angle min:", str(scan.angle_min))
        #         print("scan angle increment:", str(scan.angle_increment))
        #         print("deltay:", str(deltay))
        #         print("deltax:", str(deltax))
        #         print("angle from me to you:", str(angle_))
        #         print("my heading:", str(yaw))
        #         print("search angle:", str(check), str(check*180/math.pi))
        #         print("search index:", str(index))
        #         print("scan reading at search angle:", str(scan.ranges[index%360]))
        #         print("distance to you should be:", str((deltay**2+deltax**2)**0.5))
        #         print("is obstructed?:", scan.ranges[index] < (deltay**2+deltax**2)**0.5 - epsilon)
        return is_obstructed

    def get_gazebo_pozyx_ranges(self):
        # ranges (simulated)
        if self.noise: # switch this back to truepose for gazebo
            r0 = noise(pose_dist(self.truepose.pose,self.anchor0_truepose))
            r1 = noise(pose_dist(self.truepose.pose,self.anchor1_truepose))
            r2 = noise(pose_dist(self.truepose.pose,self.anchor2_truepose))
            if self.denoise:
                dists = {rospy.get_param('anchor0'):reverse_noise(r0), rospy.get_param('anchor1'):reverse_noise(r1), rospy.get_param('anchor2'):reverse_noise(r2)}
            else:
                dists = {rospy.get_param('anchor0'):r0, rospy.get_param('anchor1'):r1, rospy.get_param('anchor2'):r2}
        else:
            r0 = pose_dist(self.truepose.pose,self.anchor0_truepose)
            r1 = pose_dist(self.truepose.pose,self.anchor1_truepose)
            r2 = pose_dist(self.truepose.pose,self.anchor2_truepose)
            dists = {rospy.get_param('anchor0'):r0, rospy.get_param('anchor1'):r1, rospy.get_param('anchor2'):r2}
        if self.scan_reading:
            if not (self.is_obstructed(self.scan_reading, self.truepose.pose, self.anchor0_truepose) or \
                    self.is_obstructed(self.scan_reading, self.truepose.pose, self.anchor1_truepose) or \
                    self.is_obstructed(self.scan_reading, self.truepose.pose, self.anchor2_truepose)):
                return dists
        return None

    def set_pozyx_position(self):
        coords = {rospy.get_param('anchor0'):(self.anchor0.position.x,self.anchor0.position.y),
                rospy.get_param('anchor1'):(self.anchor1.position.x,self.anchor1.position.y),
                rospy.get_param('anchor2'):(self.anchor2.position.x,self.anchor2.position.y)}

        if self.sim and self.gazebo_states:
            self.load_states(self.gazebo_states)
            try:
                dists = self.get_gazebo_pozyx_ranges()
            except:
                return
            if dists is None:
                rospy.loginfo(rospy.get_namespace()+" (obstacles): estimated position: %.3f %.3f", self.odompose.pose.position.x, self.odompose.pose.position.y)
                self.pub.publish(self.odompose)
                return
        elif self.sim:
            rospy.loginfo(rospy.get_namespace()+": don't have gazebo states yet")
            return

        else:
            # ranges (real pozyx)
            rospy.logdebug("ranging...")
            dists = self.get_pozyx_ranges()
            while dists is None:
                dists = self.get_pozyx_ranges()
            rospy.loginfo(dists)

        pozyxpose = PoseStamped()
        pozyxpose.header.stamp = rospy.Time.now()
        pozyxpose.header.frame_id = rospy.get_namespace()+"base_scan"
        if triangulate(dists,coords) is not None:
            pozyxpose.pose.position.x, pozyxpose.pose.position.y = triangulate(dists,coords)
        if self.truepose: # simulated
            pozyxpose.pose.position.z = self.truepose.pose.position.z
            pozyxpose.pose.orientation = self.truepose.pose.orientation
        else: # real
            pozyxpose.pose.position.z = 0
            pozyxpose.pose.orientation = self.odom_posestamped.pose.orientation # figure out yaw from odom
        rospy.loginfo(rospy.get_namespace()+": estimated position: %.3f %.3f", pozyxpose.pose.position.x, pozyxpose.pose.position.y)
        self.pub.publish(pozyxpose)

        if self.truepose:
            rospy.logdebug("pozyx error: %.3f %.3f", pozyxpose.pose.position.x - self.truepose.pose.position.x,
                                                   pozyxpose.pose.position.y - self.truepose.pose.position.y);

    def load_states(self, states):
        for index, name in enumerate(states.name):
            if name == rospy.get_namespace()[1:-1]:
                self.truepose = PoseStamped()
                self.truepose.header.stamp = rospy.Time.now()
                self.truepose.header.frame_id = rospy.get_namespace()+"base_scan"
                self.truepose.pose = states.pose[index]
            elif name == rospy.get_param('anchor0'):
                self.anchor0_truepose = Pose()
                self.anchor0_truepose = states.pose[index]
            elif name == rospy.get_param('anchor1'):
                self.anchor1_truepose = Pose()
                self.anchor1_truepose = states.pose[index]
            elif name == rospy.get_param('anchor2'):
                self.anchor2_truepose = Pose()
                self.anchor2_truepose = states.pose[index]

if __name__ == '__main__':
    try:
        sim = (sys.argv[1] == "True" or sys.argv[1] == "true")
        if not sim:
            p = None
            ports = get_pozyx_ports()
            for i in range(len(ports)):
                try:
                    p = PozyxSerial(ports[i])
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
