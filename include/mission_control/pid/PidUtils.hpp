#ifndef PID_UTILS
#define PID_UTILS

#include <string>

namespace PidUtils{

  enum Axes {SURGE = 0, SWAY = 1, HEAVE = 2, ROLL = 3, PITCH = 4, YAW = 5};
  enum Inputs {IMU_POS = 0, IMU_ACCEL = 1, DEPTH = 2, CAM_FRONT = 3, CAM_BOTTOM = 4};
  enum UpdateParams {INPUT_TYPE = 0, PLANT_STATE = 1, SETPOINT = 2};

}

#endif
