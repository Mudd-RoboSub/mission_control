/*

    THRUSTERS

 HFL        HFR
 VFL        VFR



 HBL        HBR
 VBL        VBR


*/

#ifndef THRUST_MASTER
#define THRUST_MASTER

#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float64.h>
#include "mission_control/Thrust.h"

#endif

namespace Thrusters{


  // declare the subscribers
  ros::Subscriber surgeSub, swaySub, heaveSub, pitchSub, rollSub, yawSub;
  ros::Publisher thrustPub;
  double controlEffortSurge, controlEffortSway, controlEffortHeave,
         controlEffortRoll, controlEffortPitch, controlEffortYaw;

  void surgeCallback(const std_msgs::Float64& msg){ controlEffortSurge = msg.data;}
  void swayCallback(const std_msgs::Float64& msg){ controlEffortSway = msg.data;}
  void heaveCallback(const std_msgs::Float64& msg){ controlEffortHeave = msg.data;}
  void pitchCallback(const std_msgs::Float64& msg){ controlEffortPitch = msg.data;}
  void rollCallback(const std_msgs::Float64& msg){ controlEffortRoll = msg.data;}
  void yawCallback(const std_msgs::Float64& msg){ controlEffortYaw = msg.data;}



  std::vector<double> thrusters(8);


  //naming: H/F [Horizontal/Vertical]; F/B [Front/Back]; R/L [Right/Left]
  double& HFL = thrusters[0];
  double& HFR = thrusters[1];
  double& VFL = thrusters[2];
  double& VFR = thrusters[3];
  double& HBL = thrusters[4];
  double& HBR = thrusters[5];
  double& VBL = thrusters[6];
  double& VBR = thrusters[7];

  void move() {

      //right-rotate
      double swayRotated = 0.7071 * (controlEffortSurge + controlEffortSway);
      double surgeRotated = -0.7071 * (controlEffortSway - controlEffortSurge);

      HFL = surgeRotated - controlEffortYaw;
      HFR = swayRotated + controlEffortYaw;
      HBL = swayRotated - controlEffortYaw;
      HBR = surgeRotated + controlEffortYaw;

      VFL = controlEffortHeave - controlEffortPitch + controlEffortRoll;
      VFR = controlEffortHeave - controlEffortPitch - controlEffortRoll;
      VBL = controlEffortHeave + controlEffortPitch + controlEffortRoll;
      VBR = controlEffortHeave + controlEffortPitch - controlEffortRoll;

      ROS_INFO("surgeRotated %f, swayRotated %f", surgeRotated, swayRotated);

      double max = 0.;
      for (double thruster : thrusters) if (thruster > max) max = thruster;
      for (double thruster : thrusters) thruster /= max;
      //ifnan
      for (double thruster : thrusters) if(thruster != thruster) thruster = 0;

  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "ThrustMaster");
  ros::NodeHandle nh_;
  ros::Rate loopRate(10);

  // declare the publisher
  Thrusters::thrustPub = nh_.advertise<mission_control::Thrust>("thrusterValues", 1);

  Thrusters::surgeSub = nh_.subscribe("surgeControlEffort", 0, Thrusters::surgeCallback);
  Thrusters::swaySub = nh_.subscribe("swayControlEffort", 0, Thrusters::swayCallback);
  Thrusters::heaveSub = nh_.subscribe("heaveControlEffort", 0, Thrusters::heaveCallback);
  Thrusters::pitchSub = nh_.subscribe("pitchControlEffort", 0, Thrusters::pitchCallback);
  Thrusters::rollSub = nh_.subscribe("rollControlEffort", 0, Thrusters::rollCallback);
  Thrusters::yawSub = nh_.subscribe("yawControlEffort", 0, Thrusters::yawCallback);

  while(ros::ok) {
      Thrusters::move();

      // pack the publisher
      mission_control::Thrust msg;
      msg.HFL = Thrusters::HFL;
      msg.HFR = Thrusters::HFR;
      msg.VFL = Thrusters::VFL;
      msg.VFR = Thrusters::VFR;
      msg.HBL = Thrusters::HBL;
      msg.HBR = Thrusters::HBR;
      msg.VBL = Thrusters::VBL;
      msg.VBR = Thrusters::VBR;

      // publish the values
      Thrusters::thrustPub.publish(msg);

      ros::spinOnce();
      loopRate.sleep();
  }
}
