#include "../include/mission_control/Axis.hpp"

Axis::Axis(){
  Axis("");
}

Axis::Axis(std::string axisName): plantState_(0), setpoint_(0),
                                         controlEffort_(0)
{

  std::transform(axisName.begin(), axisName.end(),axisName.begin(), ::tolower);

  if(axisName == ""){
    controlEffortTopic_ = "controlEffort";
    percentThrustTopic_ = "percentThrust";
  }
  else{
    controlEffortTopic_ = axisName + "ControlEffort";
    percentThrustTopic_ = axisName + "PercentThrust";
  }
}

// void updateController(const PidUtils::UpdateParams& param, const double& value){
//   switch (param) {
//     case PidUtils::INPUT_TYPE:
//
//     case PidUtils::PLANT_STATE:
//
//     case PidUtils::SETPOINT:
//
//     case PidUtils::ENABLED:
//   }
// }

void Axis::setPidEnabled(const bool& enabled = true){
  enabled_ = enabled;
}

void Axis::setPlantState(){
  plantState_ =
}

int main(){
  Axis a("hHaAFDi");
  Axis b;
}
