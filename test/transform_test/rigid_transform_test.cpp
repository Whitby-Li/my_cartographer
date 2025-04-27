//
// Created by whitby on 2025-03-02.
//

#include "my_cartographer/transform/rigid_transform.h"
#include "my_cartographer/transform/transform.h"
#include "test/transform_test/rigid_transform_test_helpers.h"

#include <gtest/gtest.h>

#include <random>

using namespace my_cartographer::transform;

template <typename T>
class RigidTransformTest : public ::testing::Test
{
protected:
  T eps()
  {
    return std::numeric_limits<T>::epsilon();
  }

  Rigid2<T> GetRandomRigid2()
  {
    const T x = T(0.7) * distribution_(prng_);
    const T y = T(0.7) * distribution_(prng_);
    const T theta = T(0.2) * distribution_(prng_);
    return Rigid2<T>(typename Rigid2<T>::Vector(x, y), theta);
  }

  Rigid3<T> GetRandomRigid3()
  {
    const T x = T(0.7) * distribution_(prng_);
    const T y = T(0.7) * distribution_(prng_);
    const T z = T(0.7) * distribution_(prng_);
    const T ax = T(0.7) * distribution_(prng_);
    const T ay = T(0.7) * distribution_(prng_);
    const T az = T(0.7) * distribution_(prng_);
    return Rigid3<T>(typename Rigid3<T>::Vector(x, y, z),
                     AngleAxisVectorToRotationQuaternion(typename Rigid3<T>::Vector(ax, ay, az)));
  }

  std::mt19937 prng_ = std::mt19937(42);
  std::uniform_real_distribution<T> distribution_ = std::uniform_real_distribution<T>(-1., 1.);
};

using ScalarTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(RigidTransformTest, ScalarTypes);

TYPED_TEST(RigidTransformTest, Identity2DTest)
{
  const auto pose = this->GetRandomRigid2();
  EXPECT_THAT(pose * Rigid2<TypeParam>(), IsNearly(pose, this->eps()));
  EXPECT_THAT(Rigid2<TypeParam>() * pose, IsNearly(pose, this->eps()));
  EXPECT_THAT(pose * Rigid2<TypeParam>::Identity(), IsNearly(pose, this->eps()));
  EXPECT_THAT(Rigid2<TypeParam>::Identity() * pose, IsNearly(pose, this->eps()));
}

TYPED_TEST(RigidTransformTest, Inverse2DTest)
{
  const auto pose = this->GetRandomRigid2();
  EXPECT_THAT(pose.inverse() * pose, IsNearly(Rigid2<TypeParam>::Identity(), this->eps()));
  EXPECT_THAT(pose * pose.inverse(), IsNearly(Rigid2<TypeParam>::Identity(), this->eps()));
}

TYPED_TEST(RigidTransformTest, Identity3DTest)
{
  const auto pose = this->GetRandomRigid3();
  EXPECT_THAT(pose * Rigid3<TypeParam>(), IsNearly(pose, this->eps()));
  EXPECT_THAT(Rigid3<TypeParam>() * pose, IsNearly(pose, this->eps()));
  EXPECT_THAT(pose * Rigid3<TypeParam>::Identity(), IsNearly(pose, this->eps()));
  EXPECT_THAT(Rigid3<TypeParam>::Identity() * pose, IsNearly(pose, this->eps()));
}

TYPED_TEST(RigidTransformTest, Inverse3DTest)
{
  const auto pose = this->GetRandomRigid3();
  EXPECT_THAT(pose.inverse() * pose, IsNearly(Rigid3<TypeParam>::Identity(), this->eps()));
  EXPECT_THAT(pose * pose.inverse(), IsNearly(Rigid3<TypeParam>::Identity(), this->eps()));
}