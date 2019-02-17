#include "../include/mission_control/Axis.hpp"


Axis::Axis(std::string axisName, ros::NodeHandle nh)
    : plantState_(0), setpoint_(0), controlEffort_(0), nh_(nh)
{

  client = nh_.serviceClient<mission_control::UpdateService>("update_controller");

  std::transform(axisName.begin(), axisName.end(),axisName.begin(), ::tolower);

  controlEffortTopic_ = axisName + "ControlEffort";
  percentThrustTopic_ = axisName + "PercentThrust";
  plantStateTopic_ = axisName = "PlantState";

  plantPub_ = nh_.advertise<std_msgs::Float64>(plantStateTopic_, 1);


}

void Axis::updateController(const PidUtils::UpdateParams& param, const double& val){

  ROS_INFO("VALUE %f", val);
  updateSrv.request.param = (int)param;
  updateSrv.request.value = val;
  if (client.call(updateSrv)){
    ROS_INFO("Sent service update request %f", val);
  }
  else{
    ROS_ERROR("Failed to call service add_two_ints");
  }
}

void Axis::setPidEnabled(const bool& enabled){
  enabled_ = enabled;
  updateController(PidUtils::ENABLED, (double)enabled);
}

void Axis::setSetpoint(const double& val, const size_t& stabilityBand){
  setpoint_ = val;
  stabilityBand_ = stabilityBand;

  updateController(PidUtils::SETPOINT, val);

}

void Axis::setInputType(const PidUtils::Inputs& input){
  inputType_ = input;
  updateController(PidUtils::INPUT_TYPE, (double)input);
}

void Axis::setPlantState(const double& val){
  std_msgs::Float64 msg;
  msg.data = val;
  plantPub_.publish(msg);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "Axis");
  ros::NodeHandle nh_;
  Axis a("hHaAFDi", nh_);
  ROS_INFO("YEEET");
  double b = 11;
  a.updateController(PidUtils::SETPOINT, b);
  for(int i = 0; i < 100000; ++i){
    ROS_INFO("Hmm");
    a.setPlantState(i);
  }
}
