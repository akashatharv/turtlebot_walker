/**============================================================================
 * @file       : robot.cpp
 * @author     : Akash Atharv
 * @version    : 1.0
 * @Copyright  : 3-Clause BSD
Copyright (c) 2018, Akash Atharv
 
Redistribution and use in source and binary forms, with or without  
modification, are permitted provided that the following conditions are 
met:
 
1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
 
2. Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the   
documentation and/or other materials provided with the distribution.
 
3. Neither the name of the copyright holder nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
 * @brief      :Source file for implementation of functions used for walker node
 *============================================================================
 */

#include "robot.hpp"

Robot::Robot() {
      pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
      sub = n.subscribe("/scan", 1000, &Robot::scanCallback, this);
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      pub.publish(msg);
}

Robot::~Robot() {
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      pub.publish(msg);
}

void Robot::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
      for (int i=0;i < msg->ranges.size(); i++) {
             if (msg->ranges[i] < 0.65) {
                     collision = true;
                     break;
              }
      collision = false;
      }
}

void Robot::run() {
      ros::Rate loop_rate(10);

      while(ros::ok()) {

           if(collision) {
                ROS_INFO("Collision is expected, Turning to avoid object");
                msg.linear.x = 0.0;
                msg.angular.z = 0.1;
           }

           else {
                ROS_INFO("No collision is expected, Moving straight");
                msg.linear.x = 0.1;
                msg.angular.z = 0;
           }

      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();

      }
}


