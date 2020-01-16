#ifndef KALMAN_LOCAL_TRAJECTORY_BUILDER_H
#define KALMAN_LOCAL_TRAJECTORY_BUILDER_H


#include <iostream>

#include "kalman_local_trajectory_builder_options.pb.h"
#include "customized_time.h"
#include "pose_tracker.h"
#include "kalman_local_trajectory_builder_options.h"
// #include "rigid_transform.h"

namespace kalman_filter
{

class KalmanLocalTrajectoryBuilder
{
  public:
  KalmanLocalTrajectoryBuilder();
  ~KalmanLocalTrajectoryBuilder();

  void AddOdometryData(common::Time time, const transform::Rigid3d& pose);
  void AddOdometryData(double time, const transform::Rigid3d& pose);

  private:


  void CreateKalmanLocalTrajectoryBuilderOptions();


  std::unique_ptr<kalman_filter::PoseTracker> pose_tracker_;
  kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions options_;



}; 
  
} // namespace kalman_filter









#endif