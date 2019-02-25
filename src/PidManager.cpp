#include <mission_control/pid/PidManager.hpp>


PidManager::PidManager(ros::NodeHandle nh)
  : nh_{nh}{


  axisSurge_ = Axis("surge", nh);
  axisSway_ = Axis("sway", nh);
  axisHeave_ = Axis("heave", nh);
  axisRoll_ = Axis("roll", nh);
  axisPitch_ = Axis("pitch", nh);
  axisYaw_ = Axis("yaw", nh);


  ROS_INFO("Initialized PID Manager");



}


void PidManager::setPidEnabled(const PidUtils::Axes& axis, const bool& enabled){
  selectAxis(axis).setPidEnabled(enabled);
}

void PidManager::setPlantState(const PidUtils::Axes& axis, const double& val){
  selectAxis(axis).setPlantState(val);
}

void PidManager::setSetpoint(const PidUtils::Axes& axis, const double& val){
  selectAxis(axis).setSetpoint(val);
}

void PidManager::setInputType(const PidUtils::Axes& axis, const PidUtils::Inputs& input){
  selectAxis(axis).setInputType(input);
}

Axis& PidManager::selectAxis(const PidUtils::Axes& axis){
  switch(axis){
    case(PidUtils::SURGE):
      return axisSurge_;
    case(PidUtils::SWAY):
      return axisSway_;
    case(PidUtils::HEAVE):
      return axisHeave_;
    case(PidUtils::ROLL):
      return axisRoll_;
    case(PidUtils::PITCH):
      return axisPitch_;
    case(PidUtils::YAW):
      return axisYaw_;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "PidManager");
  ros::NodeHandle nh_;
  PidManager a(nh_);

  for(int i = 0; i < 1000; ++i){
    a.setSetpoint(PidUtils::HEAVE, 10.2);
    a.setPlantState(PidUtils::HEAVE, 11);
    a.setInputType(PidUtils::HEAVE, PidUtils::IMU_POS);
    a.setPidEnabled(PidUtils::HEAVE, false);
  }
  ros::Rate r(10);
  while(true){//stop
    r.sleep();
  }
}
