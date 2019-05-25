#include "mission_control/pid/Pid.hpp"

Pid::Pid(int axis, int input)
         : isFirstCallBack_(false), enabled_(true){

  nhPriv_ = ros::NodeHandle("~");

  while(ros::ok() && ros::Time(0) == ros::Time::now()){
    ROS_INFO("Time is zero. Waiting for the beginning of time.");
    sleep(1);
  }


  // Get params if specified in launch file or as params on command-line, set
  // defaults
  nhPriv_.param<double>("Kp", kP_, 1.0);
  nhPriv_.param<double>("Ki", kI_, 0.0);
  nhPriv_.param<double>("Kd", kD_, 0.0);
  nhPriv_.param<double>("windup_limit", windupLimit_, 100.0);
  nhPriv_.param<double>("cutoff_frequency", cutoffFrequency_, -1.0);
  nhPriv_.param<int>("inputType", inputTypeInt_, input);
  nhPriv_.param<int>("axis", axisInt_, axis);


  if(inputTypeInt_ == -1) inputType_ = "OTHER_INPUT";
  else inputType_ = inputs_.at(inputTypeInt_);

  if(axisInt_ == -1) axis_ = "OTHER_AXIS";
  else axis_ = axes_.at(axisInt_);

  std::string axisTopic = axis_;
  std::transform(axisTopic.begin(), axisTopic.end(),axisTopic.begin(), ::tolower);

  //Now we have enough to generate topic names
  plantStateTopic_ = axisTopic + "PlantState";
  setpointTopic_ = axisTopic + "Setpoint";
  enabledTopic_ = axisTopic + "Enabled";
  inputTopic_ = axisTopic + "InputType";
  controlEffortTopic_ = axisTopic + "ControlEffort";


  //So the first time is registered as an input
  prevInputType_ = "OTHER_INPUT";

  //Register publisher for control effort
  controlEffortPub_ = nh_.advertise<std_msgs::Float64>(controlEffortTopic_, 1);

  //and the subscribers
  plantStateSub_ = nh_.subscribe(plantStateTopic_, 0, &Pid::plantStateCallback, this);
  setpointSub_ = nh_.subscribe(setpointTopic_, 0, &Pid::setpointCallback, this);
  setpointSub_ = nh_.subscribe(setpointTopic_, 0, &Pid::setpointCallback, this);
  enabledSub_ = nh_.subscribe(enabledTopic_, 0, &Pid::enabledCallback, this);
  inputSub_ = nh_.subscribe(inputTopic_, 0, &Pid::inputCallback, this);



  if(!plantStateSub_){
    ROS_ERROR_STREAM("Plant state subscriber initialization failed. Quitting");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  //load in relevant parameters from yaml file, select appropriate set based on
  //config
  loadParamsFromFile();
  getParamsFromMap();

  //service registration
  dynamic_reconfigure::Server<mission_control::PidConfig> config_server;
  dynamic_reconfigure::Server<mission_control::PidConfig>::CallbackType f;


  f = boost::bind(&Pid::reconfigureCallback, this, _1, _2);
  config_server.setCallback(f);



  while(ros::ok()){


    double timePassed = ros::Time::now().toSec() - prevTime_.toSec();

    //get change in time
    if(timePassed == 0){
      //do bare minimum maintenance
      ros::Duration(.05).sleep();
      prevTime_ = ros::Time::now();
      continue;
    }

    prevTime_ = ros::Time::now();

    bool shouldUpdate = false;

    //ramp the setpoint, linearly
    if(setpointChanged_){
      if(setpointRampDuration_ == 0){
        setpoint_ = setpointStep_[1];
        setpointChanged_ = false;
      }
      else{
        double t = ros::Time::now().toSec() - setpointChangeTime_;
        setpoint_ = setpointStep_[0] + //starting val
                  //produces correct slope so step is achieved in desired time
                  ((setpointStep_[1] - setpointStep_[0])/setpointRampDuration_) * t;

        //if ramp complete
        if(setpointStep_[0] > setpointStep_[1] && setpoint_ > setpointStep_[1] ||
           setpointStep_[0] < setpointStep_[1] && setpoint_ < setpointStep_[1])
            setpoint_ = setpointStep_[1];

        shouldUpdate = true;

        //if ramp is done
        if(setpoint_ == setpointStep_[1]) setpointChanged_ = false;
      }
    }
    else if(plantStateChanged_){
      shouldUpdate = true;
      plantStateChanged_ = false;
    }

    if(shouldUpdate){
      prevControlEffort_ = controlEffort_;
    }
    executeController(timePassed);

    //publish the congtrol effort
    if(enabled_ && prevControlEffort_ != controlEffort_){
      controlEffortMsg_.data = controlEffort_;
      controlEffortPub_.publish(controlEffortMsg_);
    }
    else{
      errorIntegral_ = 0;
    }

    ros::spinOnce();

    //cpu protecc
    ros::Duration(0.01).sleep();

  }
}

//calculates control effort
void Pid::executeController(double timePassed){

  error_.at(1) = error_.at(0);
  error_.at(0) = getError();

  if(timePassed == 0){
    ROS_ERROR("No time change detected between loops.");
    return;
  }

  updateErrors(timePassed);

  controlEffort_ = kP_ * error_.at(0) +
                   kI_ * errorIntegral_ +
                   kD_ * errorDeriv_.at(0);

}

//updates the error and it's integral and deriviatives. Adds neccesary filters.
void Pid::updateErrors(double timePassed){

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
  filteredError_.at(0) = error_.at(0);
  //filters seem to be causing troubles. Workaround fix.
  //Shouldn't affect performance noticably since we filter data elsewhere
  /*(1 / (1 + c_ * c_ + 1.414 * c_)) * (error_.at(2) + 2 * error_.at(1) + error_.at(0) -
                                                              (c_ * c_ - 1.414 * c_ + 1) * filteredError_.at(2) -
                                                              (-2 * c_ * c_ + 2) * filteredError_.at(1));*/

  // Take derivative of error
  // First the raw, unfiltered data:
  errorDeriv_.at(2) = errorDeriv_.at(1);
  errorDeriv_.at(1) = errorDeriv_.at(0);
  errorDeriv_.at(0) = (error_.at(0) - error_.at(1)) / timePassed;

  filteredErrorDeriv_.at(2) = filteredErrorDeriv_.at(1);
  filteredErrorDeriv_.at(1) = filteredErrorDeriv_.at(0);

  filteredErrorDeriv_.at(0) = errorDeriv_.at(0);
  //hey bud your filters suck
      /*(1 / (1 + c_ * c_ + 1.414 * c_)) *
      (errorDeriv_.at(2) + 2 * errorDeriv_.at(1) + errorDeriv_.at(0) -
       (c_ * c_ - 1.414 * c_ + 1) * filteredErrorDeriv_.at(2) - (-2 * c_ * c_ + 2) * filteredErrorDeriv_.at(1));*/


   //riemann that boi
   errorIntegral_ += timePassed * error_.at(0);

   ROS_INFO("Error %f, Integral %f, Deriv %f", error_.at(0), errorIntegral_, errorDeriv_.at(0));
}


void Pid::updatePlantState(const double& state){
  ROS_INFO("Plant state changing to %f", state);
  plantState_ = state;
  plantStateChanged_ = true;
}

void Pid::updateSetpoint(const double& setpoint){
  ROS_INFO("Executing request to change setpoint for axis %s to %f",
            axis_.c_str(), setpoint);
  setpointStep_[0] = setpointStep_[1];
  setpointStep_[1] = setpoint;
  setpointChanged_ = true;
  setpointChangeTime_ = ros::Time::now().toSec();
}

void Pid::updateInputType(std::string input){
  ROS_INFO("Executing request to change input type for axis %s to %s",
            axis_.c_str(), input.c_str());
  prevInputType_ = inputType_;
  inputType_ = input;
  getParamsFromMap();
}

void Pid::loadParamsFromFile(){

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

void Pid::getParamsFromMap(){

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

void Pid::reconfigureCallback(mission_control::PidConfig& config, uint32_t level){

  inputType_ = inputs_.at(config.inputType);
  ROS_INFO("Integer %d, inputType_ %s", config.inputType, inputType_.c_str());

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


std::string Pid::getConfigPath(){

  std::string path = axis_ + "/" + inputType_;

  ROS_INFO("returning path: %s", path.c_str());

  return path;
}

void Pid::saveParams(){
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


void Pid::writeToFile(){
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

/* -------------  ROS CALLBACKS --------------------*/

void Pid::plantStateCallback(const std_msgs::Float64& msg){
  updatePlantState(msg.data);
}

void Pid::setpointCallback(const std_msgs::Float64& msg){
  updateSetpoint(msg.data);
}

void Pid::enabledCallback(const std_msgs::Bool& msg){
  enabled_ = msg.data;
  ROS_INFO("%s %s PID loop", enabled_? "Enabled" : "Disabled", axis_.c_str());
}

void Pid::inputCallback(const std_msgs::Int32& msg){
  if(msg.data < 0 || msg.data > 5){
    ROS_ERROR("Attempted to set input type with invalid integer value %d. "
              "Aborting", msg.data);
  }
  else
    updateInputType(inputs_.at(msg.data));
}


//destructor just writes current vals to file. It's kinda nice
Pid::~Pid(){
  writeToFile();
}


int main(int argc, char** argv){
  ros::init(argc, argv, "PidNode");

  Pid a(0, 0);
  ros::spin();
  return 0;
}
