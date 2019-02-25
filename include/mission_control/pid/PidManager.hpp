//stamps out six axes
//provides mux for calling all the functions
//additional functionality such as go-to-depth which only applied to a specific axis


#ifndef PID_MANAGER
#define PID_MANAGER

#include "mission_control/pid/PidUtils.hpp"
#include "mission_control/Axis.hpp"
#include <ros/ros.h>

class PidManager{


public:

  PidManager() = delete;
  PidManager(ros::NodeHandle);
  void setPidEnabled(const PidUtils::Axes& axis, const bool& enabled = true);
  void setPlantState(const PidUtils::Axes& axis, const double& val);
  void setSetpoint(const PidUtils::Axes& axis, const double& val);
  void setInputType(const PidUtils::Axes& axis, const PidUtils::Inputs&);

private:

  Axis& selectAxis(const PidUtils::Axes& axis);
  void setPercentThrust(const PidUtils::Axes& axis, const double& val);



  void CheckStability();

  Axis axisSurge_, axisSway_, axisHeave_, axisRoll_, axisPitch_, axisYaw_;

  ros::NodeHandle nh_;


};


#endif
