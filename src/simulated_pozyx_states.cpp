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

/* Authors: Taehoon Lim (Darby) */

#include <ros/ros.h>
#include <string>
#include <std_msgs/Bool.h>

ros::Publisher pozyx_states_pub;

// void initRay()
// {
//     testRay = boost::dynamic_pointer_cast<RayShape>(world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulated_pozyx_state");
  ros::NodeHandle nh;

  pozyx_states_pub  = nh.advertise<std_msgs::Bool>("pozyx_state", 10);

  ros::Rate loop_rate(1);

  // initRay();

  while (ros::ok())
  {
    std_msgs::Bool state;
    state.data = true;
    pozyx_states_pub.publish(state);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
