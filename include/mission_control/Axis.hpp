#ifndef AXIS
#define AXIS

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <mission_control/UpdateService.h>
#include <ros/ros.h>
#include "mission_control/pid/PidUtils.hpp"
#include <std_msgs/Float64.h>

class Axis{

public:
  Axis() = delete;

  //note: intentionally not by const referance; want a copy
  Axis(std::string axisName, ros::NodeHandle nh);

  //should call the service
  void updateController(const PidUtils::UpdateParams& param, const double& val);

  //just call updateController accordingly
  void setPidEnabled(const bool& enabled = true);
  void setPlantState(const double& val);
  void setSetpoint(const double& val, const size_t& stabilityBand = 100);
  void setInputType(const PidUtils::Inputs&);


  //for manually setting thrust
  void setPercentThrust(const double& val);


  // void setInputType(const PidUtils&::Inputs);

  bool isStable(const double& stableMargin = .1);


private:


  //used to keep track of steps to track stability
  double setpointBuffer_[2] = {0,0};

  //will sub to control effort, plant state
  std::string controlEffortTopic_, plantStateTopic_;

  //will publish percent thrust
  std::string percentThrustTopic_;
  double percentThrust_;


  //state of controller
  double plantState_;
  double controlEffort_;
  bool enabled_;

  double setpoint_;
  double stabilityBand_;

  PidUtils::Inputs inputType_;

  mission_control::UpdateService updateSrv;
  ros::ServiceClient client;
  ros::NodeHandle nh_;
  ros::Publisher plantPub_;



};



#endif
