#ifndef KALAMN_FILTER_POSE_TRACKER_H_
#define KALAMN_FILTER_POSE_TRACKER_H_

#include <deque>
#include <memory>

#include "Eigen/Cholesky"
#include "Eigen/Core"

#include "transform.h"
#include "unscented_kalman_filter.h"
#include "gaussian_distribution.h"
#include "customized_time.h"
#include "pose_tracker_options.pb.h"
#include "kalman_local_trajectory_builder_options.pb.h"
#include "imu_tracker.h"
#include "odometry_tracker.h"

// using namespace transform;

namespace kalman_filter{

typedef Eigen::Matrix3d Pose2DCovariance;
typedef Eigen::Matrix<double, 6, 6> PoseCovariance;

struct PoseAndCovariance {
  transform::Rigid3d pose;
  PoseCovariance covariance;
};

PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance);

PoseCovariance BuildPoseCovariance(double translational_variance,
                                   double rotational_variance);



class PoseTracker {
public:
  enum {
    kMapPositionX = 0,
    kMapPositionY,
    kMapPositionZ,
    kMapOrientationX,
    kMapOrientationY,
    kMapOrientationZ,
    kMapVelocityX,
    kMapVelocityY,
    kMapVelocityZ,
    kDimension  // We terminate loops with this.  9
  };

  PoseTracker(const kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions& options, double time);
  virtual ~PoseTracker();
  
  using KalmanFilter = UnscentedKalmanFilter<double, kDimension>;
  using State = KalmanFilter::StateType;
  using StateCovariance = Eigen::Matrix<double, kDimension, kDimension>;
  using Distribution = GaussianDistribution<double, kDimension>;

  void AddPoseObservation(double time, const transform::Rigid3d& pose, const PoseCovariance& covariance);
  void AddOdometerPoseObservation(double time, const transform::Rigid3d& pose, const PoseCovariance& covariance);

  void AddImuLinearAccelerationObservation(double time, const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(double time, const Eigen::Vector3d& imu_angular_velocity);

  Distribution GetBelief(double time);



private:

  static Distribution KalmanFilterInit();

  const Distribution BuildModelNoise(double delta_t) const;

  void Predict(double time);

  //Computes a pose combining the given 'state' with the 'imu_tracker_' orientation.
  transform::Rigid3d RigidFromState(const PoseTracker::State& state);

  const kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions options_;
  double time_;
  KalmanFilter kalman_filter_;
  sensor::ImuTracker imu_tracker_;
  sensor::OdometryTracker odometry_tracker_;

};

}

#endif