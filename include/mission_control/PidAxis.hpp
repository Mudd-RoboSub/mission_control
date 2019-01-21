/**

\file PidAxis.hpp

\author Seth Isaacson

\brief Implements the Axis class, which contains a PID controller which
       governs a specific axis. Also includes a handy interface for updating
       relevant parameters.

\remark Despite what some of my peers my claim, at least in the context of this
        package, it is not only correct but REQUIRED to refer to a PID
        controller not as a "P. I. D. controller", but as a "pid controller",
        where pid is a single word rhyming with "kid" (@omari).

\remark Work largely based on the ROS PID package, written by Andy Zelenak and
        Paul Bouchier. Package documented at http://wiki.ros.org/pid. This is
        really just a major refactor of that code with some features added,
        and much of the code copy-pastad over.

*/

#ifndef PIDAXIS_H
#define PIDAXIS_H

#ifndef ROS
#define ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <mission_control/PidAxisConfig.h>
#include <dynamic_reconfigure/server.h>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <ros/package.h>


#endif

class PidAxis{

private:

  std::vector<std::string> axes_ = {"SURGE", "SWAY", "HEAVE", "ROLL", "PITCH", "YAW"};
  std::vector<std::string> inputs_ = {"IMU_POS", "IMU_ACCEL", "DEPTH", "CAM_FRONT", "CAM_BOTTOM"};


public:

  PidAxis();

  //no copy constructor, shouldn't be used
  PidAxis(const PidAxis&) = delete;
  //no assignment operator either
  PidAxis operator=(const PidAxis&) const = delete;

  //Saves parameters to file. Requires clean exit.
  ~PidAxis();

  void updateInputType(std::string input);

  void updatePlantState(const double&);
  void updateSetpoint(const double&);


private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_;

  //central PID parameters
  double kP_, kD_, kI_;

  std::string inputType_ = "OTHER_INPUT";
  std::string prevInputType_ = "OTHER_INPUT";
  int inputTypeInt_ = -1; //for rosparam usage, immediately casts as enum

  std::string axis_ = "OTHER_AXIS";
  int axisInt_ = -1;

  double controlEffort_ = 0;

  //Important stuff to a PID controller
  double setpoint_ = 0;
  double plantState_ = 0;
  std::vector<double> error_ = {0,0,0};
  std::vector<double> filteredError_ = {0,0,0};
  double errorIntegral_ = 0;
  std::vector<double> errorDeriv_ = {0,0,0};
  std::vector<double> filteredErrorDeriv_ = {0,0,0};

  //has one of parameters recently changed?
  bool setpointChanged_; //goes false once ramp is complete
  bool plantStateChanged_;

  double setpointChangeTime_ = 0;
  double setpointStep_[2]; //[oldVal, newVal]
  double setpointRampDuration_ = 0;

  ros::Time prevTime_;

  //max value for integral term to reach
  double windupLimit_;

  //filter stuff - swiped from ROS PID

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoffFrequency_ = -1;


  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at
  // 1/4 of the sample rate.
  double c_ = 1.;

  //control effort publishing
  std::string controlEffortTopic_;
  std_msgs::Float64 controlEffortMsg_;
  ros::Publisher controlEffortPub_;


  ///Get new kP, kI, kD values from config.
  void loadParamsFromFile();
  std::unordered_map<std::string, std::vector<double>> paramMap_;
  void getParamsFromMap();


  //private methods
  inline double getError(){return setpoint_ - plantState_;}

  ///do all the PID stuff
  void updateController(double timePassed);

  ///updates buffers and filters. The filters were taken DIRECTLY from the ROS
  ///PID package, which in turn cites Julius O. Smith III, Intro. to
  /// Digital Filters With Audio Applications.
  void updateErrors(double);

  void reconfigureCallback(mission_control::PidAxisConfig& config, uint32_t level);

  void saveParams();

  std::string getConfigPath();

  void writeToFile();
};



#endif
