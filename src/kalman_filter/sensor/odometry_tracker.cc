#include "odometry_tracker.h"

// #include "transform.h"
// #include "rigid_transform.h"
#include "glog/logging.h"

// #include <>

namespace sensor{


// OdometryState::OdometryTracker(const double time, 
//                                const transform::Rigid3d& odometer_pose, 
//                                const transform::Rigid3d& state_pose)    
//     : time(time), odometer_pose(odometer_pose), state_pose(state_pose)
// {
    
// }

OdometryTracker::OdometryTracker(const int window_size): window_size_(window_size) 
{
    //   CHECK_GT(window_size, 0);
}

bool OdometryTracker::empty() const 
{
    std::cout << "[OdometryTracker::empty][odometry_states_.size():] " << odometry_states_.size() << std::endl;
  return odometry_states_.empty(); 
}

const OdometryState& OdometryTracker::newest()  const
{
  return odometry_states_.back();
}


}