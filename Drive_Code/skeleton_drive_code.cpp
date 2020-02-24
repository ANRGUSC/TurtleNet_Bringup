/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "turtlebot3_gazebo/turtlebot3_drive.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  ROS_ASSERT(init());
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  ROS_INFO("Shutting down...");
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Helper Functions
*******************************************************************************/

// return a vector of lidar readings
std::vector<double> GetDistances(int v_size) {
  std::vector<float> d(v_size); 
  // make a reading at every angle & add to vector
  return d; 
} 

// given a vector of lidar readings, figure out where the paths ( hallways are )
std::vector<int> FindPaths(const std::vector<float>& lidar_readings) {
  float hall_thresh = 7; // 7m clear means theres a hallway??
  std::vector<int> p; // all the paths
  // TODO: see if there are 10 degrees in a row clear
  // if yes, push back int value in middle of that range
  for(int i = 0; i < lidar_readings.size(); i++) {
    if(lidar_readings[i] > hall_thresh) {
      p.push_back(i); 
    }
  }
  return p; 
}

// given a new desired orientation, turn the burger!
void MakeTurn(int direction, Turtlebot3Drive* tb) {
  double turn_time = angular_speed / direction; 
  tb->updatecommandVelocity(0,angular_speed); 
  thread.sleep(turn_time); 
  tb->updatecommandVelocity(0,0); 
}

// drives straight for a specified amount of time
void DriveStraight(int drive_time) {
  double linear_speed = 1.2;
  tb->updatecommandVelocity(linear_speed,0); 
  thread.sleep(drive_time); 
  tb->updatecommandVelocity(0,0); 
}


/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop(Turtlebot3Drive* tb)
{
  static uint8_t turtlebot3_state_num = 0;

  // one Burger moving at a time
  std::vector<double> distances;

  int num_readings = 360 / precision; 

  distances = GetDistances(num_readings); 

  // the integer passed in is the angle you need to go 
  std::vector<int> paths;   // the available paths
  paths = FindPaths(distances);      
  int direction = -1;       // the direction you end up going

  // blocked, no hall detected (could be blocked by another burger)
  if(paths.size() == 0) {
    return false; 
  }
  else { // randomly pick one of these valid paths
    direction = paths[rand() % paths.size()]; 
  }

  // calls update command velocity to turn some distance (in degrees)
  // starts turning and stops turning 
  MakeTurn(direction, tb); 

  // drives straight for a specified amount of time
  int time_driving = 0;
  DriveStraight(time_driving);

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;
  int turns_without_moving = 0;  

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    // see if it did not moved
    if(!turtlebot3_drive.controlLoop()) {
      turns_without_moving++;
    }
    if(turns_without_moving > 5) {

    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
