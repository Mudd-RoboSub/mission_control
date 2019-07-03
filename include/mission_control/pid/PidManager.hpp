//stamps out six axes
//provides mux for calling all the functions
//additional functionality such as go-to-depth which only applied to a specific axis

#ifndef PID_MANAGER
#define PID_MANAGER

#include "mission_control/Axis.hpp"
#include "mission_control/EnabledService.h"
#include "mission_control/InputService.h"
#include "mission_control/SetpointService.h"
#include "mission_control/ThrustOverrideService.h"

class PidManager{


public:

  PidManager() = default;
  PidManager(ros::NodeHandle);
  void setPidEnabled(const PidUtils::Axes& axis, const bool& enabled = true);
  void setPlantState(const PidUtils::Axes& axis, const double& val);
  void setSetpoint(const PidUtils::Axes& axis, const double& val);
  void setInputType(const PidUtils::Axes& axis, const PidUtils::Inputs&);

private:

  //get corresponding axis from enum or string
  Axis& selectAxis(const PidUtils::Axes& axis);
  Axis& selectAxis(const std::string& axis);

  void setPercentThrust(const PidUtils::Axes& axis, const double& val);

  //to handle yaw wrapping
  double yawSetpoint_;
  ros::Subscriber yawSub_;


  void CheckStability();

  Axis axisSurge_, axisSway_, axisHeave_, axisRoll_, axisPitch_, axisYaw_;

  ros::NodeHandle nh_;
  ros::ServiceServer enabledService_, inputService_, setpointService_, thrustOverrideService_;


  bool enabledServiceCB(mission_control::EnabledService::Request &req,
                        mission_control::EnabledService::Response &res);

  bool inputServiceCB(mission_control::InputService::Request &req,
                      mission_control::InputService::Response &res);

  bool setpointServiceCB(mission_control::SetpointService::Request &req,
                         mission_control::SetpointService::Response &res);


  bool thrustOverrideCB(mission_control::ThrustOverrideService::Request &req,
                        mission_control::ThrustOverrideService::Response &res);

};


#endif
