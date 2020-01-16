#ifndef TRANSFORM_TRANSFORM_RIGID_TRANSFORM_H_
#define TRANSFORM_TRANSFORM_RIGID_TRANSFORM_H_

#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace transform{



template <typename FloatType>
class Rigid3 {
     public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3(): translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation): translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation): translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3<FloatType> Identity() {
    return Rigid3<FloatType>(Vector::Zero(), Quaternion::Identity());
  }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(), rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  std::string DebugString() const {
    std::string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append(", ");
    out.append(std::to_string(translation().z()));
    out.append("], q: [");
    out.append(std::to_string(rotation().w()));
    out.append(", ");
    out.append(std::to_string(rotation().x()));
    out.append(", ");
    out.append(std::to_string(rotation().y()));
    out.append(", ");
    out.append(std::to_string(rotation().z()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

using Rigid3d = Rigid3<double>;

}


#endif