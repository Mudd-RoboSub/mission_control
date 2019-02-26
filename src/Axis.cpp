#include "../include/mission_control/Axis.hpp"
//
// Axis::Axis(){
//   std::cerr << "boo" << std::endl;
// }


Axis::Axis(std::string axisName, const ros::NodeHandle& nh)
    : axisName_(axisName), plantState_(0), setpoint_(0), controlEffort_(0), nh_(nh)
{

  std::transform(axisName.begin(), axisName.end(),axisName.begin(), ::tolower);

  // switch(axisName){
  //   case("surge"): axisInt_ = (int)PidUtils::SURGE;
  //   case("sway"): axisInt_ = (int)PidUtils::SWAY;
  //   case("heave"): axisInt_ = (int)PidUtils::HEAVE;
  //   case("roll"): axisInt_ = (int)PidUtils::HEAVE;
  //   case("pitch"): axisInt_ = (int)PidUtils::PITCH;
  //   case("yaw"): axisInt_ = (int)PidUtils::YAW;
  //   default: axisInt_ = -1;
  // }


  //For bypassing PID Control
  percentThrustTopic_ = axisName + "PercentThrust";


  //PID Interface
  plantStateTopic_ = axisName + "PlantState";
  setpointTopic_ = axisName + "Setpoint";
  enabledTopic_ = axisName + "Enabled";
  inputTopic_ = axisName + "InputType";
  controlEffortTopic_ = axisName + "ControlEffort";

  //PID Publishers
  plantPub_ = nh_.advertise<std_msgs::Float64>(plantStateTopic_, 1);
  setpointPub_ = nh_.advertise<std_msgs::Float64>(setpointTopic_, 1);
  enabledPub_ = nh_.advertise<std_msgs::Bool>(enabledTopic_, 1);
  inputPub_ = nh_.advertise<std_msgs::Int32>(inputTopic_, 1);

  percentThrustPub_ = nh_.advertise<std_msgs::Float64>(percentThrustTopic_, 1);


}

//publishers for PID interface


void Axis::setPidEnabled(const bool& enabled){
  enabled_ = enabled;
  std_msgs::Bool msg;
  msg.data = enabled_;
  enabledPub_.publish(msg);
}

void Axis::setSetpoint(const double& val, const size_t& stabilityBand){
  setpoint_ = val;
  stabilityBand_ = stabilityBand;
  std_msgs::Float64 msg;
  msg.data = val;
  std::cout << "VAL" << val << std::endl;
  setpointPub_.publish(msg);
}

void Axis::setInputType(const PidUtils::Inputs& input){
  inputType_ = input;
  std_msgs::Int32 msg;
  msg.data = (int)input;
  inputPub_.publish(msg);
}

void Axis::setPlantState(const double& val){
  std_msgs::Float64 msg;
  msg.data = val;
  plantPub_.publish(msg);
}

void Axis::controlEffortCallback(const std_msgs::Float64& msg){
  controlEffort_ = msg.data;
}

// int main(int argc, char** argv){
//   ros::init(argc, argv, "Axis");
//   ros::NodeHandle nh_;
//   Axis a("hHaAFDi", nh_);
//   ROS_INFO("YEEET");
//   double b = 11;
//   a.updateController(PidUtils::SETPOINT, b);
//   for(int i = 0; i < 100000; ++i){
//     ROS_INFO("Hmm");
//     a.setPlantState(i);
//   }
// }
