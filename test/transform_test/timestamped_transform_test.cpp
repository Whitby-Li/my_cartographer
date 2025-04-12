//
// Created by whitby on 2025-04-12.
//

#include "my_cartographer/transform/timestamped_transform.h"
#include "test/transform_test/rigid_transform_test_helpers.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace my_cartographer::transform;
using namespace my_cartographer;

namespace test
{

  TEST(TimestampedTransformTest, ToProtoAndBack)
  {
    const TimestampedTransform expected{
        common::FromUniversal(12345678),
        Rigid3d(Eigen::Vector3d(1., 2., 3.),
                Eigen::Quaterniond(1., 2., 3., 4.).normalized())};
    const TimestampedTransform actual = FromProto(ToProto(expected));
    EXPECT_EQ(expected.time, actual.time);
    EXPECT_THAT(actual.transform, IsNearly(expected.transform, 1e-6));
  }

} // namespace test