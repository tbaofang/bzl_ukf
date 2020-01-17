# include "pose_tracker.h"

#include "Eigen/Geometry"
#include "glog/logging.h"
// #include "transform.h"

namespace kalman_filter{

namespace {

PoseTracker::State AddDelta(const PoseTracker::State& state, const PoseTracker::State& delta)
{
  std::cout << "[AddDelta][state:] " << state << std::endl;
  PoseTracker::State new_state = state + delta;
  transform::Rigid3d pose;

  const Eigen::Quaterniond orientation = transform::AngleAxisVectorToRotationQuaternion(
                Eigen::Vector3d(state[PoseTracker::kMapOrientationX], 
                state[PoseTracker::kMapOrientationY], 
                state[PoseTracker::kMapOrientationZ]));

  const Eigen::Vector3d rotation_vector(delta[PoseTracker::kMapOrientationX],
                                        delta[PoseTracker::kMapOrientationY],  
                                        delta[PoseTracker::kMapOrientationZ]);

  std::cout << "[][delta.transpose():] " << delta.transpose() << std::endl;
  std::cout << "[AddDelta][rotation_vector.norm():] " << rotation_vector.norm() << std::endl;

  // CHECK_LT(rotation_vector.norm(), M_PI / 2.) << "Sigma point is far from the mean, recovered delta may be incorrect.";

  const Eigen::Quaterniond rotation = transform::AngleAxisVectorToRotationQuaternion(rotation_vector);
  const Eigen::Vector3d new_orientation = transform::RotationQuaternionToAngleAxisVector(orientation * rotation);
  new_state[PoseTracker::kMapOrientationX] = new_orientation.x();
  new_state[PoseTracker::kMapOrientationY] = new_orientation.y();
  new_state[PoseTracker::kMapOrientationZ] = new_orientation.z();
  return new_state;    
}

PoseTracker::State ComputeDelta(const PoseTracker::State& origin, const PoseTracker::State& target) 
{
  PoseTracker::State delta = target - origin;
  const Eigen::Quaterniond origin_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(origin[PoseTracker::kMapOrientationX],
                          origin[PoseTracker::kMapOrientationY],
                          origin[PoseTracker::kMapOrientationZ]));
  const Eigen::Quaterniond target_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(target[PoseTracker::kMapOrientationX],
                          target[PoseTracker::kMapOrientationY],
                          target[PoseTracker::kMapOrientationZ]));
  const Eigen::Vector3d rotation =
      transform::RotationQuaternionToAngleAxisVector(
          origin_orientation.inverse() * target_orientation);
  delta[PoseTracker::kMapOrientationX] = rotation.x();
  delta[PoseTracker::kMapOrientationY] = rotation.y();
  delta[PoseTracker::kMapOrientationZ] = rotation.z();
  return delta;
}

// // Build a model matrix for the given time delta.
PoseTracker::State ModelFunction(const PoseTracker::State& state, const double delta_t) 
{
  std::cout << "[ModelFunction]" << std::endl;
  CHECK_GT(delta_t, 0.);

  PoseTracker::State new_state;
  new_state[PoseTracker::kMapPositionX] =
      state[PoseTracker::kMapPositionX] +
      delta_t * state[PoseTracker::kMapVelocityX];
  new_state[PoseTracker::kMapPositionY] =
      state[PoseTracker::kMapPositionY] +
      delta_t * state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapPositionZ] =
      state[PoseTracker::kMapPositionZ] +
      delta_t * state[PoseTracker::kMapVelocityZ];

  new_state[PoseTracker::kMapOrientationX] =
      state[PoseTracker::kMapOrientationX];
  new_state[PoseTracker::kMapOrientationY] =
      state[PoseTracker::kMapOrientationY];
  new_state[PoseTracker::kMapOrientationZ] =
      state[PoseTracker::kMapOrientationZ];

  new_state[PoseTracker::kMapVelocityX] = state[PoseTracker::kMapVelocityX];
  new_state[PoseTracker::kMapVelocityY] = state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapVelocityZ] = state[PoseTracker::kMapVelocityZ];

  return new_state;
}

} // namespace


PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance) {
  GaussianDistribution<double, 6> distribution(
      Eigen::Matrix<double, 6, 1>::Zero(), pose_and_covariance.covariance);
  Eigen::Matrix<double, 6, 6> linear_transform;
  linear_transform << transform.rotation().matrix(), Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), transform.rotation().matrix();
  return {transform * pose_and_covariance.pose,
          (linear_transform * distribution).GetCovariance()};
}

PoseTracker::Distribution PoseTracker::KalmanFilterInit() 
{
  State initial_state = State::Zero();
  // We are certain about the complete state at the beginning. We define the
  // initial pose to be at the origin and axis aligned. Additionally, we claim
  // that we are not moving.
  StateCovariance initial_covariance = 1e-9 * StateCovariance::Identity();
  return Distribution(initial_state, initial_covariance);
}

PoseTracker::PoseTracker(const kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions& options, double time)
            : options_(options), 
              time_(time),
              kalman_filter_(KalmanFilterInit(), AddDelta, ComputeDelta),
              imu_tracker_(options.imu_gravity_time_constant(), time),
              odometry_tracker_(options.num_odometry_states())
{
  std::cout << "[PoseTracker::PoseTracker][options.num_odometry_states():] " << options_.num_odometry_states() << std::endl;

}

PoseTracker::~PoseTracker(){}


const PoseTracker::Distribution PoseTracker::BuildModelNoise(const double delta_t) const 
{
  std::cout << "[PoseTracker::Distribution]" << std::endl;
  std::cout << "[PoseTracker::Distribution][position_model_variance:] " << options_.position_model_variance() << std::endl;
  std::cout << "[PoseTracker::Distribution][orientation_model_variance:] " << options_.orientation_model_variance() << std::endl;
  std::cout << "[PoseTracker::Distribution][velocity_model_variance:] " << options_.velocity_model_variance() << std::endl;
  // Position is constant, but orientation changes.
  StateCovariance model_noise = StateCovariance::Zero();

  model_noise.diagonal() <<
      // Position in map.
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,

      // Orientation in map.
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,

      // Linear velocities in map.
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t;

  return Distribution(State::Zero(), model_noise);
}


void PoseTracker::Predict(const double time) 
{
  std::cout << "[PoseTracker::Predict]" << std::endl;

  imu_tracker_.Advance(time);
  CHECK_LE(time_, time);
  const double delta_t = time - time_;
  if (delta_t == 0.) {
    return;
  }

  // BuildModelNoise(delta_t);
  // const State& state;
  // ModelFunction(state, delta_t);
  // auto model_function = [this, delta_t](const State& state) -> State { 
  //   std::cout << "[PoseTracker::Predict][delta_t:] " << delta_t << std::endl;
  //   return ModelFunction(state, delta_t);};

  // const PoseTracker::Distribution model_noise = BuildModelNoise(delta_t);

  // std::cout << "[UKF::Predict enter]" << std::endl;
  // kalman_filter_.Predict(model_function, model_noise);

  kalman_filter_.Predict([this, delta_t](const State& state) -> State {return ModelFunction(state, delta_t);},
                         BuildModelNoise(delta_t));

  time_ = time;


  // std::cout << "[PoseTracker::Predict][delta_t:] " << delta_t << std::endl;
}

transform::Rigid3d PoseTracker::RigidFromState(const PoseTracker::State& state) 
{
  std::cout << "[PoseTracker::RigidFromState][state.transpose():] " << state.transpose() << std::endl;
  return transform::Rigid3d(
      Eigen::Vector3d(state[PoseTracker::kMapPositionX],
                      state[PoseTracker::kMapPositionY],
                      state[PoseTracker::kMapPositionZ]),
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(state[PoseTracker::kMapOrientationX],
                          state[PoseTracker::kMapOrientationY],
                          state[PoseTracker::kMapOrientationZ])) *
          imu_tracker_.orientation());
}


void PoseTracker::AddPoseObservation(const double time,
                                     const transform::Rigid3d& pose,
                                     const PoseCovariance& covariance)
{
  Predict(time);

  // Noise covariance is taken directly from the input values.
  const GaussianDistribution<double, 6> delta(Eigen::Matrix<double, 6, 1>::Zero(), covariance);

  // RigidFromState(pose);
  // auto function = [this, &pose](const State& state) -> Eigen::Matrix<double, 6, 1> 
  //     {
  //       const transform::Rigid3d state_pose = RigidFromState(state);

  //       const Eigen::Vector3d delta_orientation =
  //           transform::RotationQuaternionToAngleAxisVector(
  //               pose.rotation().inverse() * state_pose.rotation());

  //       const Eigen::Vector3d delta_translation =
  //           state_pose.translation() - pose.translation();

  //       Eigen::Matrix<double, 6, 1> return_value;
  //       return_value << delta_translation, delta_orientation;

  //       return return_value;
  //     };

  // function;    

  kalman_filter_.Observe<6>(
      [this, &pose](const State& state) -> Eigen::Matrix<double, 6, 1> {
        const transform::Rigid3d state_pose = RigidFromState(state);
        const Eigen::Vector3d delta_orientation =
            transform::RotationQuaternionToAngleAxisVector(
                pose.rotation().inverse() * state_pose.rotation());
        const Eigen::Vector3d delta_translation =
            state_pose.translation() - pose.translation();
        Eigen::Matrix<double, 6, 1> return_value;
        return_value << delta_translation, delta_orientation;
        return return_value;
      },
      delta);
}


PoseTracker::Distribution PoseTracker::GetBelief(const double time) {
  Predict(time);
  return kalman_filter_.GetBelief();
}

void PoseTracker::AddOdometerPoseObservation(const double time, const transform::Rigid3d& odometer_pose,
                                             const PoseCovariance& covariance) 
{
    // std::cout << "[][odometry_tracker_.size():] " << odometry_tracker_.size() << std::endl;
  if (!odometry_tracker_.empty()) 
  {
    const auto& previous_odometry_state = odometry_tracker_.newest();
    const transform::Rigid3d delta = previous_odometry_state.odometer_pose.inverse() * odometer_pose;
    const transform::Rigid3d new_pose = previous_odometry_state.state_pose * delta;
    AddPoseObservation(time, new_pose, covariance);
  }

  const Distribution belief = GetBelief(time);

  // sensor::OdometryState odometry_state = sensor::OdometryState(time, odometer_pose, RigidFromState(belief.GetMean()));
  // odometry_tracker_.AddOdometryState( );

    odometry_tracker_.AddOdometryState({time, odometer_pose, RigidFromState(belief.GetMean())});
}


void PoseTracker::AddImuLinearAccelerationObservation(const double time, const Eigen::Vector3d& imu_linear_acceleration) 
{
  // imu_tracker_.Advance(time);
  // imu_tracker_.AddImuLinearAccelerationObservation(imu_linear_acceleration);
  // Predict(time);
}

void PoseTracker::AddImuAngularVelocityObservation(const double time, const Eigen::Vector3d& imu_angular_velocity) 
{
  imu_tracker_.Advance(time);
  // imu_tracker_.AddImuAngularVelocityObservation(imu_angular_velocity);
  // Predict(time);
}


PoseCovariance BuildPoseCovariance(const double translational_variance, const double rotational_variance) 
{
  const Eigen::Matrix3d translational = Eigen::Matrix3d::Identity() * translational_variance;
  const Eigen::Matrix3d rotational = Eigen::Matrix3d::Identity() * rotational_variance;
  PoseCovariance covariance;
  covariance << translational, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), rotational;
  return covariance;
}




}
