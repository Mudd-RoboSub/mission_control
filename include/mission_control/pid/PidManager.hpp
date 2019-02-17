//stamps out six axes
//provides mux for calling all the functions
//additional functionality such as go-to-depth which only applied to a specific axis


#ifndef PID_MANAGER
#define PID_MANAGER

#include <PidUtils.hpp>

class PidManager{


private:

  UpdateController(const PidUtils::Axes& axis, const std::string& param, const double& val);

  CheckStability()

}


#endif
