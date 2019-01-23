#include "mission_control/pid/PidAxis.hpp"

PidAxis::PidAxis() : isFirstCallBack_(false){


  nhPriv_ = ros::NodeHandle("~");

  while(ros::ok() && ros::Time(0) == ros::Time::now()){
    ROS_INFO("Time is zero. Waiting for the beginning of time.");
    sleep(1);
  }



  //Register publisher
  controlEffortPub_ = nh_.advertise<std_msgs::Float64>(controlEffortTopic_, 1);

  // Get params if specified in launch file or as params on command-line, set
  // defaults
  nhPriv_.param<double>("Kp", kP_, 1.0);
  nhPriv_.param<double>("Ki", kI_, 0.0);
  nhPriv_.param<double>("Kd", kD_, 0.0);
  nhPriv_.param<double>("windup_limit", windupLimit_, 100.0);
  nhPriv_.param<double>("cutoff_frequency", cutoffFrequency_, -1.0);
  nhPriv_.param<std::string>("control_effort_topic", controlEffortTopic_, "effort");
  nhPriv_.param<int>("inputType", inputTypeInt_, -1);
  nhPriv_.param<int>("axis", axisInt_, -1);


  if(inputTypeInt_ == -1) inputType_ = "OTHER_INPUT";
  else inputType_ = inputs_.at(inputTypeInt_);

  if(axisInt_ == -1) axis_ = "OTHER_AXIS";
  else axis_ = axes_.at(axisInt_);

  //So the first time is registered as an input
  prevInputType_ = "OTHER_INPUT";

  ROS_INFO("I");

  loadParamsFromFile();

  ROS_INFO("J");

  getParamsFromMap();

  ROS_INFO("K");

  dynamic_reconfigure::Server<mission_control::PidAxisConfig> config_server;
  dynamic_reconfigure::Server<mission_control::PidAxisConfig>::CallbackType f;


  f = boost::bind(&PidAxis::reconfigureCallback, this, _1, _2);
  config_server.setCallback(f);


  //service to update stuff
  updateServiceServer_ = nh_.advertiseService("update_controller", &PidAxis::updateController, this);



  while(ros::ok()){

    //get change in time
    if(prevTime_.isZero()){
      prevTime_ = ros::Time::now();
      continue;
    }

    double timePassed = ros::Time::now().toSec() - prevTime_.toSec();
    prevTime_ = ros::Time::now();

    bool shouldUpdate = false;

    //ramp the setpoint, linearly
    if(setpointChanged_){
      double t = ros::Time::now().toSec() - setpointChangeTime_;
      setpoint_ = setpointStep_[0] + //starting val
                //produces correct slope so step is achieved in desired time
                ((setpointStep_[1] - setpointStep_[0])/setpointChangeTime_) * t;
      shouldUpdate = true;

      //if ramp is done
      if(setpoint_ >= setpointStep_[1]) setpointChanged_ = false;
    }
    else if(plantStateChanged_){
      shouldUpdate = true;
      plantStateChanged_ = false;
    }

    if(shouldUpdate) executeController(timePassed);

    //publish the congtrol effort
    controlEffortMsg_.data = controlEffort_;
    controlEffortPub_.publish(controlEffortMsg_);

    ros::spinOnce();

    //cpu protecc
    ros::Duration(0.001).sleep();

  }
}

//calculates control effort
void PidAxis::executeController(double timePassed){

  error_.at(1) = error_.at(0);
  error_.at(0) = getError();


  if(timePassed == 0){
    ROS_ERROR("No time change detected between loops.");
    return;
  }

  updateErrors(timePassed);

  controlEffort_ = kP_ * filteredError_.at(0) +
                   kI_ * errorIntegral_ +
                   kD_ * filteredErrorDeriv_.at(0);

}

//updates the error and it's integral and deriviatives. Adds neccesary filters.
void PidAxis::updateErrors(double timePassed){

  //if not specified by user. Again, not my code.
  if (cutoffFrequency_ != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    double tan_filt_ = tan((cutoffFrequency_ * 6.2832) * timePassed / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
      tan_filt_ = -0.01;
    if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
      tan_filt_ = 0.01;

    c_ = 1 / tan_filt_;
  }

  filteredError_.at(2) = filteredError_.at(1);
  filteredError_.at(1) = filteredError_.at(0);
  filteredError_.at(0) = (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_.at(2) + 2 * error_.at(1) + error_.at(0) -
                                                              (c_ * c_ - 1.414 * c_ + 1) * filteredError_.at(2) -
                                                              (-2 * c_ * c_ + 2) * filteredError_.at(1));

  // Take derivative of error
  // First the raw, unfiltered data:
  errorDeriv_.at(2) = errorDeriv_.at(1);
  errorDeriv_.at(1) = errorDeriv_.at(0);
  errorDeriv_.at(0) = (error_.at(0) - error_.at(1)) / timePassed;

  filteredErrorDeriv_.at(2) = filteredErrorDeriv_.at(1);
  filteredErrorDeriv_.at(1) = filteredErrorDeriv_.at(0);

  filteredErrorDeriv_.at(0) =
      (1 / (1 + c_ * c_ + 1.414 * c_)) *
      (errorDeriv_.at(2) + 2 * errorDeriv_.at(1) + errorDeriv_.at(0) -
       (c_ * c_ - 1.414 * c_ + 1) * filteredErrorDeriv_.at(2) - (-2 * c_ * c_ + 2) * filteredErrorDeriv_.at(1));


   //riemann that boi
   errorIntegral_ += timePassed * error_.at(0);


}


void PidAxis::updatePlantState(const double& state){
  plantState_ = state;
  plantStateChanged_ = true;
}

void PidAxis::updateSetpoint(const double& setpoint){
  setpointStep_[0] = setpointStep_[1];
  setpointStep_[1] = setpoint;
  setpointChanged_ = true;
}

void PidAxis::updateInputType(std::string input){
  prevInputType_ = inputType_;
  inputType_ = input;
  getParamsFromMap();
}

void PidAxis::loadParamsFromFile(){

  if(axis_ == "OTHER_AXIS" || inputType_ == "OTHER_INPUT"){
    ROS_WARN("Using non-recognized axis or input type. Setting constants to 0");
    kP_ = 0;
    kI_ = 0;
    kD_ = 0;
  }

  std::vector<double> values;
  std::string path;
  ROS_INFO("3");

  for(auto axis : axes_){
    for(auto input : inputs_){
      path = axis + "/" + input;
      nh_.getParam(path, values);
      std::pair<std::string, std::vector<double>> mapPair(path, values);

      paramMap_.insert(mapPair);
    }
  }

}

void PidAxis::getParamsFromMap(){

  if(axis_ == "OTHER_AXIS" || inputType_ == "OTHER_INPUT"){
    ROS_WARN("Invalid axis or input detected. Setting constants to 0");
    kP_ = 0;
    kI_ = 0;
    kD_ = 0;
    return;
  }

  std::string path = getConfigPath();
  std::vector<double> values = paramMap_.at(path);

  kP_ = values.at(0);
  kI_ = values.at(1);
  kD_ = values.at(2);

  ROS_INFO("Loaded default parameters for %s configuration:"
           "Kp=%f, Ki=%f, Kd=%f", path.c_str(), kP_, kI_, kD_);

}


//if a slider has changed, update the val

void PidAxis::reconfigureCallback(mission_control::PidAxisConfig& config, uint32_t level){

  inputType_ = inputs_.at(config.inputType);
  ROS_INFO("INteger %d, inputType_ %s", config.inputType, inputType_.c_str());
  
  if(config.Save){
    saveParams();
    config.Save = false;
    return;
  }
  else if(config.restoreDefaults){
    getParamsFromMap();
    config.Kp = kP_;
    config.Kd = kD_;
    config.Ki = kI_;
    config.restoreDefaults = false;
    return;
  }
  else if(inputType_ != prevInputType_){
    updateInputType(inputType_);
    config.Kp = kP_;
    config.Kd = kD_;
    config.Ki = kI_;
    return;
  }

  kP_ = config.Kp;
  kI_ = config.Ki;
  kD_ = config.Kd;
  ROS_INFO("Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f", kP_, kI_, kD_);

}


std::string PidAxis::getConfigPath(){

  std::string path = axis_ + "/" + inputType_;

  ROS_INFO("returning path: %s", path.c_str());

  return path;
}

void PidAxis::saveParams(){
  ROS_INFO("Saving parameters to map");
  std::string path;

  if(axis_ == "OTHER_AXIS" || inputType_ == "OTHER_INPUT"){
    ROS_ERROR("Cannot save parameters while using unsupported axis or input.");
    return;
  }

  path = getConfigPath();
  ROS_INFO("path: %s", path.c_str());
  paramMap_.at(path) = {kP_, kI_, kD_};
  for(auto i : paramMap_.at(path)){
    ROS_INFO("value: %f", i);
  }
}


void PidAxis::writeToFile(){
  std::ofstream tuneFile;
  std::string filePath = ros::package::getPath("mission_control") + "/include/mission_control/pid/tune.yaml";
  tuneFile.open(filePath);

  tuneFile << "#Don't edit this file unless you're sure!" << std::endl;

  for(auto i : axes_){
    tuneFile << i << ":" << std::endl;
    for(auto j : inputs_){
      std::string paramPath = i + "/" + j;
      ROS_INFO("Path to file: %s", paramPath.c_str());
      std::vector<double> newParams = paramMap_.at(paramPath);
      double kP, kI, kD;
      kP = newParams.at(0);
      kI = newParams.at(1);
      kD = newParams.at(2);
      tuneFile << "  " << j << ": [" << kP << "," << kI << "," << kD << "]" << std::endl;
    }
  }

  tuneFile.close();

}

//service callback
bool PidAxis::updateController(mission_control::UpdateService::Request &req,
                      mission_control::UpdateService::Response &res){



  int param = (int) req.param;

  if(param == PidUtils::INPUT_TYPE){
    int value = (int) req.value;
    if(value < 0 || value > 5){
      ROS_ERROR("Invalid input type (integer value %d). Aborting", value);
      res.success = false;
      return false;
    }
    else{
      updateInputType(inputs_.at(value));
    }
  }

  else if(param == PidUtils::PLANT_STATE){
    updatePlantState(req.value);
  }

  else if(param == PidUtils::SETPOINT){
    updateSetpoint(req.value);
  }

  else{
    ROS_ERROR("Input parameter (integer value %d) not accepted. Update aborted",
              param);
    res.success = false;
    return false;
  }
  ROS_INFO("Updated controller");
  res.success = true;

  return true;


}

PidAxis::~PidAxis(){
  writeToFile();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "PidAxisNode");

  PidAxis a;
  ros::spin();
  return 0;
}
