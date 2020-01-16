#ifndef CARTOGRAPHER_MAPPING_3D_KALMAN_LOCAL_TRAJECTORY_BUILDER_OPTIONS_H_
#define CARTOGRAPHER_MAPPING_3D_KALMAN_LOCAL_TRAJECTORY_BUILDER_OPTIONS_H_

// #include "common/lua_parameter_dictionary.h"
#include "kalman_local_trajectory_builder_options.pb.h"

namespace cartographer {
namespace mapping_3d {

kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions 
CreateKalmanLocalTrajectoryBuilderOptions();

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_KALMAN_LOCAL_TRAJECTORY_BUILDER_OPTIONS_H_
