//
// Created by whitby on 2025-03-01.
//

#ifndef MY_CARTOGRAPHER_TRANSFORM_TRANSFORM_H
#define MY_CARTOGRAPHER_TRANSFORM_TRANSFORM_H

#include "my_cartographer/common/math.hpp"
#include "my_cartographer/transform/proto/transform.pb.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <Eigen/Geometry>

#include <cmath>

namespace my_cartographer
{
  namespace transform
  {

    // Returns the non-negative rotation angle in radians of the 3D transformation
    // 'transform'.
    template <typename FloatType>
    FloatType GetAngle(const Rigid3<FloatType> &transform)
    {
      return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                       std::abs(transform.rotation().w()));
    }

    // Returns the yaw component in radians of the given 3D 'rotation'. Assuming
    // 'rotation' is composed of three rotations around X, then Y, then Z, returns
    // the angle of the Z rotation.
    template <typename T>
    T GetYaw(const Eigen::Quaternion<T> &rotation)
    {
      const Eigen::Matrix<T, 3, 1> direction =
          rotation * Eigen::Matrix<T, 3, 1>::UnitX();
      return atan2(direction.y(), direction.x());
    }

    // Returns the yaw component in radians of the given 3D transformation
    // 'transform'.
    template <typename T>
    T GetYaw(const Rigid3<T> &transform)
    {
      return GetYaw(transform.rotation());
    }

    // Returns an angle-axis vector (a vector with the length of the rotation angle
    // pointing to the direction of the rotation axis) representing the same
    // rotation as the given 'quaternion'.
    template <typename T>
    Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
        const Eigen::Quaternion<T> &quaternion)
    {
      Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
      // We choose the quaternion with positive 'w', i.e., the one with a smaller
      // angle that represents this orientation.
      if (normalized_quaternion.w() < 0.)
      {
        // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
        normalized_quaternion.w() = -1. * normalized_quaternion.w();
        normalized_quaternion.x() = -1. * normalized_quaternion.x();
        normalized_quaternion.y() = -1. * normalized_quaternion.y();
        normalized_quaternion.z() = -1. * normalized_quaternion.z();
      }
      // We convert the normalized_quaternion into a vector along the rotation axis
      // with length of the rotation angle.
      const T angle =
          2. * atan2(normalized_quaternion.vec().norm(), normalized_quaternion.w());
      constexpr double kCutoffAngle = 1e-7; // We linearize below this angle.
      const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / 2.);
      return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                    scale * normalized_quaternion.y(),
                                    scale * normalized_quaternion.z());
    }

    // Returns a quaternion representing the same rotation as the given 'angle_axis'
    // vector.
    template <typename T>
    Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
        const Eigen::Matrix<T, 3, 1> &angle_axis)
    {
      T scale = T(0.5);
      T w = T(1.);
      constexpr double kCutoffAngle = 1e-8; // We linearize below this angle.
      if (angle_axis.squaredNorm() > kCutoffAngle)
      {
        const T norm = angle_axis.norm();
        scale = sin(norm / 2.) / norm;
        w = cos(norm / 2.);
      }
      const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
      return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                                  quaternion_xyz.z());
    }

    // Projects 'transform' onto the XY plane.
    template <typename T>
    Rigid2<T> Project2D(const Rigid3<T> &transform)
    {
      return Rigid2<T>(transform.translation().template head<2>(),
                       GetYaw(transform));
    }

    // Embeds 'transform' into 3D space in the XY plane.
    template <typename T>
    Rigid3<T> Embed3D(const Rigid2<T> &transform)
    {
      return Rigid3<T>(
          {transform.translation().x(), transform.translation().y(), T(0)},
          Eigen::AngleAxis<T>(transform.rotation().angle(),
                              Eigen::Matrix<T, 3, 1>::UnitZ()));
    }

    // Conversions between Eigen and proto.
    Rigid2d ToRigid2(const proto::Rigid2d &transform);
    Eigen::Vector2d ToEigen(const proto::Vector2d &vector);
    Eigen::Vector3f ToEigen(const proto::Vector3f &vector);
    Eigen::Vector4f ToEigen(const proto::Vector4f &vector);
    Eigen::Vector3d ToEigen(const proto::Vector3d &vector);
    Eigen::Quaterniond ToEigen(const proto::Quaterniond &quaternion);
    proto::Rigid2d ToProto(const Rigid2d &transform);
    proto::Rigid2f ToProto(const Rigid2f &transform);
    proto::Rigid3d ToProto(const Rigid3d &rigid);
    Rigid3d ToRigid3(const proto::Rigid3d &rigid);
    proto::Rigid3f ToProto(const Rigid3f &rigid);
    proto::Vector2d ToProto(const Eigen::Vector2d &vector);
    proto::Vector3f ToProto(const Eigen::Vector3f &vector);
    proto::Vector4f ToProto(const Eigen::Vector4f &vector);
    proto::Vector3d ToProto(const Eigen::Vector3d &vector);
    proto::Quaternionf ToProto(const Eigen::Quaternionf &quaternion);
    proto::Quaterniond ToProto(const Eigen::Quaterniond &quaternion);

  } // namespace transform

} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_TRANSFORM_TRANSFORM_H