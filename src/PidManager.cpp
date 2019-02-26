#include <mission_control/pid/PidManager.hpp>


PidManager::PidManager(ros::NodeHandle nh)
  : nh_{nh}{


  axisSurge_ = Axis("surge", nh);
  axisSway_ = Axis("sway", nh);
  axisHeave_ = Axis("heave", nh);
  axisRoll_ = Axis("roll", nh);
  axisPitch_ = Axis("pitch", nh);
  axisYaw_ = Axis("yaw", nh);

  //pause for a bit to let the messages catch up
  ros::Duration(2).sleep();

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


  ros::Rate r(100);


  for(int i = 0; i < 100; ++i){
    std::cout << i;
    a.setSetpoint(PidUtils::SURGE, i);
    r.sleep();
  }

}
