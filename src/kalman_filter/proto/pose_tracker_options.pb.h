// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pose_tracker_options.proto

#ifndef PROTOBUF_pose_5ftracker_5foptions_2eproto__INCLUDED
#define PROTOBUF_pose_5ftracker_5foptions_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3004000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace cartographer {
namespace kalman_filter {
namespace proto {
class PoseTrackerOptions;
class PoseTrackerOptionsDefaultTypeInternal;
extern PoseTrackerOptionsDefaultTypeInternal _PoseTrackerOptions_default_instance_;
}  // namespace proto
}  // namespace kalman_filter
}  // namespace cartographer

namespace cartographer {
namespace kalman_filter {
namespace proto {

namespace protobuf_pose_5ftracker_5foptions_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static void InitDefaultsImpl();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_pose_5ftracker_5foptions_2eproto

// ===================================================================

class PoseTrackerOptions : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.kalman_filter.proto.PoseTrackerOptions) */ {
 public:
  PoseTrackerOptions();
  virtual ~PoseTrackerOptions();

  PoseTrackerOptions(const PoseTrackerOptions& from);

  inline PoseTrackerOptions& operator=(const PoseTrackerOptions& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PoseTrackerOptions(PoseTrackerOptions&& from) noexcept
    : PoseTrackerOptions() {
    *this = ::std::move(from);
  }

  inline PoseTrackerOptions& operator=(PoseTrackerOptions&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const PoseTrackerOptions& default_instance();

  static inline const PoseTrackerOptions* internal_default_instance() {
    return reinterpret_cast<const PoseTrackerOptions*>(
               &_PoseTrackerOptions_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(PoseTrackerOptions* other);
  friend void swap(PoseTrackerOptions& a, PoseTrackerOptions& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PoseTrackerOptions* New() const PROTOBUF_FINAL { return New(NULL); }

  PoseTrackerOptions* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const PoseTrackerOptions& from);
  void MergeFrom(const PoseTrackerOptions& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(PoseTrackerOptions* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional double position_model_variance = 1;
  bool has_position_model_variance() const;
  void clear_position_model_variance();
  static const int kPositionModelVarianceFieldNumber = 1;
  double position_model_variance() const;
  void set_position_model_variance(double value);

  // optional double orientation_model_variance = 2;
  bool has_orientation_model_variance() const;
  void clear_orientation_model_variance();
  static const int kOrientationModelVarianceFieldNumber = 2;
  double orientation_model_variance() const;
  void set_orientation_model_variance(double value);

  // optional double velocity_model_variance = 3;
  bool has_velocity_model_variance() const;
  void clear_velocity_model_variance();
  static const int kVelocityModelVarianceFieldNumber = 3;
  double velocity_model_variance() const;
  void set_velocity_model_variance(double value);

  // optional double imu_gravity_time_constant = 4;
  bool has_imu_gravity_time_constant() const;
  void clear_imu_gravity_time_constant();
  static const int kImuGravityTimeConstantFieldNumber = 4;
  double imu_gravity_time_constant() const;
  void set_imu_gravity_time_constant(double value);

  // optional double imu_gravity_variance = 5;
  bool has_imu_gravity_variance() const;
  void clear_imu_gravity_variance();
  static const int kImuGravityVarianceFieldNumber = 5;
  double imu_gravity_variance() const;
  void set_imu_gravity_variance(double value);

  // optional int32 num_odometry_states = 6;
  bool has_num_odometry_states() const;
  void clear_num_odometry_states();
  static const int kNumOdometryStatesFieldNumber = 6;
  ::google::protobuf::int32 num_odometry_states() const;
  void set_num_odometry_states(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:cartographer.kalman_filter.proto.PoseTrackerOptions)
 private:
  void set_has_position_model_variance();
  void clear_has_position_model_variance();
  void set_has_orientation_model_variance();
  void clear_has_orientation_model_variance();
  void set_has_velocity_model_variance();
  void clear_has_velocity_model_variance();
  void set_has_imu_gravity_time_constant();
  void clear_has_imu_gravity_time_constant();
  void set_has_imu_gravity_variance();
  void clear_has_imu_gravity_variance();
  void set_has_num_odometry_states();
  void clear_has_num_odometry_states();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  double position_model_variance_;
  double orientation_model_variance_;
  double velocity_model_variance_;
  double imu_gravity_time_constant_;
  double imu_gravity_variance_;
  ::google::protobuf::int32 num_odometry_states_;
  friend struct protobuf_pose_5ftracker_5foptions_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PoseTrackerOptions

// optional double position_model_variance = 1;
inline bool PoseTrackerOptions::has_position_model_variance() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PoseTrackerOptions::set_has_position_model_variance() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PoseTrackerOptions::clear_has_position_model_variance() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void PoseTrackerOptions::clear_position_model_variance() {
  position_model_variance_ = 0;
  clear_has_position_model_variance();
}
inline double PoseTrackerOptions::position_model_variance() const {
  // @@protoc_insertion_point(field_get:cartographer.kalman_filter.proto.PoseTrackerOptions.position_model_variance)
  return position_model_variance_;
}
inline void PoseTrackerOptions::set_position_model_variance(double value) {
  set_has_position_model_variance();
  position_model_variance_ = value;
  // @@protoc_insertion_point(field_set:cartographer.kalman_filter.proto.PoseTrackerOptions.position_model_variance)
}

// optional double orientation_model_variance = 2;
inline bool PoseTrackerOptions::has_orientation_model_variance() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void PoseTrackerOptions::set_has_orientation_model_variance() {
  _has_bits_[0] |= 0x00000002u;
}
inline void PoseTrackerOptions::clear_has_orientation_model_variance() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void PoseTrackerOptions::clear_orientation_model_variance() {
  orientation_model_variance_ = 0;
  clear_has_orientation_model_variance();
}
inline double PoseTrackerOptions::orientation_model_variance() const {
  // @@protoc_insertion_point(field_get:cartographer.kalman_filter.proto.PoseTrackerOptions.orientation_model_variance)
  return orientation_model_variance_;
}
inline void PoseTrackerOptions::set_orientation_model_variance(double value) {
  set_has_orientation_model_variance();
  orientation_model_variance_ = value;
  // @@protoc_insertion_point(field_set:cartographer.kalman_filter.proto.PoseTrackerOptions.orientation_model_variance)
}

// optional double velocity_model_variance = 3;
inline bool PoseTrackerOptions::has_velocity_model_variance() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void PoseTrackerOptions::set_has_velocity_model_variance() {
  _has_bits_[0] |= 0x00000004u;
}
inline void PoseTrackerOptions::clear_has_velocity_model_variance() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void PoseTrackerOptions::clear_velocity_model_variance() {
  velocity_model_variance_ = 0;
  clear_has_velocity_model_variance();
}
inline double PoseTrackerOptions::velocity_model_variance() const {
  // @@protoc_insertion_point(field_get:cartographer.kalman_filter.proto.PoseTrackerOptions.velocity_model_variance)
  return velocity_model_variance_;
}
inline void PoseTrackerOptions::set_velocity_model_variance(double value) {
  set_has_velocity_model_variance();
  velocity_model_variance_ = value;
  // @@protoc_insertion_point(field_set:cartographer.kalman_filter.proto.PoseTrackerOptions.velocity_model_variance)
}

// optional double imu_gravity_time_constant = 4;
inline bool PoseTrackerOptions::has_imu_gravity_time_constant() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void PoseTrackerOptions::set_has_imu_gravity_time_constant() {
  _has_bits_[0] |= 0x00000008u;
}
inline void PoseTrackerOptions::clear_has_imu_gravity_time_constant() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void PoseTrackerOptions::clear_imu_gravity_time_constant() {
  imu_gravity_time_constant_ = 0;
  clear_has_imu_gravity_time_constant();
}
inline double PoseTrackerOptions::imu_gravity_time_constant() const {
  // @@protoc_insertion_point(field_get:cartographer.kalman_filter.proto.PoseTrackerOptions.imu_gravity_time_constant)
  return imu_gravity_time_constant_;
}
inline void PoseTrackerOptions::set_imu_gravity_time_constant(double value) {
  set_has_imu_gravity_time_constant();
  imu_gravity_time_constant_ = value;
  // @@protoc_insertion_point(field_set:cartographer.kalman_filter.proto.PoseTrackerOptions.imu_gravity_time_constant)
}

// optional double imu_gravity_variance = 5;
inline bool PoseTrackerOptions::has_imu_gravity_variance() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void PoseTrackerOptions::set_has_imu_gravity_variance() {
  _has_bits_[0] |= 0x00000010u;
}
inline void PoseTrackerOptions::clear_has_imu_gravity_variance() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void PoseTrackerOptions::clear_imu_gravity_variance() {
  imu_gravity_variance_ = 0;
  clear_has_imu_gravity_variance();
}
inline double PoseTrackerOptions::imu_gravity_variance() const {
  // @@protoc_insertion_point(field_get:cartographer.kalman_filter.proto.PoseTrackerOptions.imu_gravity_variance)
  return imu_gravity_variance_;
}
inline void PoseTrackerOptions::set_imu_gravity_variance(double value) {
  set_has_imu_gravity_variance();
  imu_gravity_variance_ = value;
  // @@protoc_insertion_point(field_set:cartographer.kalman_filter.proto.PoseTrackerOptions.imu_gravity_variance)
}

// optional int32 num_odometry_states = 6;
inline bool PoseTrackerOptions::has_num_odometry_states() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void PoseTrackerOptions::set_has_num_odometry_states() {
  _has_bits_[0] |= 0x00000020u;
}
inline void PoseTrackerOptions::clear_has_num_odometry_states() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void PoseTrackerOptions::clear_num_odometry_states() {
  num_odometry_states_ = 0;
  clear_has_num_odometry_states();
}
inline ::google::protobuf::int32 PoseTrackerOptions::num_odometry_states() const {
  // @@protoc_insertion_point(field_get:cartographer.kalman_filter.proto.PoseTrackerOptions.num_odometry_states)
  return num_odometry_states_;
}
inline void PoseTrackerOptions::set_num_odometry_states(::google::protobuf::int32 value) {
  set_has_num_odometry_states();
  num_odometry_states_ = value;
  // @@protoc_insertion_point(field_set:cartographer.kalman_filter.proto.PoseTrackerOptions.num_odometry_states)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace proto
}  // namespace kalman_filter
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_pose_5ftracker_5foptions_2eproto__INCLUDED