#ifndef AXIS
#define AXIS

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

class Axis{

public:
  Axis();

  //note: intentionally not by const referance; want a copy
  Axis(std::string axisName);

  //should call the service
  void updateController(const PidUtils::UpdateParams& param, const double& value);

  //just call updateController accordingly
  void setPidEnabled(const bool& enabled = true);
  void setPercentThrust(const double& val);
  void setPlantState(const double& val);
  void setSetpoint(const double& val, const size_t& stabilityBufferSize = 100);

  // void setInputType(const PidUtils&::Inputs);

  bool isStable(const double& stableMargin = .1);


  //main function: should be called in a loop! Does all neccesary pub/sub/etc.
  //call updateController
  void update();




private:


  //used to keep track of steps to track stability
  double setpointBuffer_[2] = {0,0};

  //will sub to control effort
  std::string controlEffortTopic_;

  //will publish percent thrust
  std::string percentThrustTopic_;
  double percentThrust_;

  double plantState_;
  double setpoint_;
  double controlEffort_;

  bool enabled_;



};



#endif
