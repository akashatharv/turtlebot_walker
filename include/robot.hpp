/**============================================================================
 * @file       : robot.hpp
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
 * @brief      :Header file for declaration of functions used for walker node
 *============================================================================
 */
#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "ros/console.h"

class Robot {
 public:

      Robot();

      ~Robot();
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
      void run();
 private:
      bool collision;
      geometry_msgs::Twist msg;
      ros::NodeHandle n;
      ros::Subscriber sub;
      ros::Publisher pub;
};
#endif /* INCLUDE_ROBOT_HPP_ */
