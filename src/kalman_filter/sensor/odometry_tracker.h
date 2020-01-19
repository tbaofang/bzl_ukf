#ifndef SENSOR_ODOMETRY_TRACKER_H_
#define SENSOR_ODOMETRY_TRACKER_H_

#include <deque>

#include "rigid_transform.h"
#include "customized_time.h"

namespace sensor {

struct OdometryState {
  OdometryState(common::Time time, const transform::Rigid3d& odometer_pose, const transform::Rigid3d& state_pose){};
  OdometryState() {}

  common::Time time = common::Time::min();
  // double time = 0.0;
  transform::Rigid3d odometer_pose = transform::Rigid3d::Identity();
  transform::Rigid3d state_pose = transform::Rigid3d::Identity();
};

class OdometryTracker
{
 public:
  using OdometryStates = std::deque<OdometryState>;

  explicit OdometryTracker(int window_size);

  void AddOdometryState(const OdometryState& odometry_state);

  bool empty() const;

//   OdometryStates& newest() ;
const OdometryState& newest() const;


 private:
  OdometryStates odometry_states_;
  size_t window_size_;

};


}

#endif
