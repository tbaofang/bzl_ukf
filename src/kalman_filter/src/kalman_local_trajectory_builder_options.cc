

#include "kalman_local_trajectory_builder_options.h"

// #include "cartographer/kalman_filter/pose_tracker.h"
// #include "cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"

namespace kalman_filter {

kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions 
CreateKalmanLocalTrajectoryBuilderOptions() 
{
    bool use_online_correlative_scan_matching =  false;
    double scan_matcher_variance = 2.34e-9;
    double odometer_translational_variance = 1e-7;
    double odometer_rotational_variance = 1e-7;

  
    double imu_gravity_variance = 0;
    int num_odometry_states = 1;
    double position_model_variance = 0.00654766;
    double orientation_model_variance = 5e-3;
    double velocity_model_variance = 0.53926;

  kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions options;

  options.set_use_online_correlative_scan_matching(use_online_correlative_scan_matching);
  options.set_scan_matcher_variance(scan_matcher_variance);
  options.set_odometer_translational_variance(odometer_translational_variance);
  options.set_odometer_rotational_variance(odometer_rotational_variance);

  options.set_imu_gravity_variance(imu_gravity_variance);
  options.set_num_odometry_states(num_odometry_states);
  options.set_position_model_variance(position_model_variance);
  options.set_orientation_model_variance(orientation_model_variance);
  options.set_velocity_model_variance(velocity_model_variance);

//   *options.mutable_real_time_correlative_scan_matcher_options() =
//       mapping_2d::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
//           parameter_dictionary
//               ->GetDictionary("real_time_correlative_scan_matcher")
//               .get());
//   *options.mutable_pose_tracker_options() =
//       kalman_filter::CreatePoseTrackerOptions(
//           parameter_dictionary->GetDictionary("pose_tracker").get());


  return options;
}

} 
