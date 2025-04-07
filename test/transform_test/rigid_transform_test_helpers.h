#ifndef MY_CARTOGRAPHER_TEST_RIGID_TRANSFORM_TEST_HELPERS_H
#define MY_CARTOGRAPHER_TEST_RIGID_TRANSFORM_TEST_HELPERS_H

#include "my_cartographer/common/port.hpp"
#include "my_cartographer/transform/rigid_transform.h"

#include <Eigen/Geometry>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace my_cartographer::transform;

template <typename T> Eigen::Transform<T, 2, Eigen::Affine> ToEigen(const Rigid2<T> &rigid2)
{
    return Eigen::Translation<T, 2>(rigid2.translation()) * rigid2.rotation();
}

template <typename T> Eigen::Transform<T, 3, Eigen::Affine> ToEigen(const Rigid3<T> &rigid3)
{
    return Eigen::Translation<T, 3>(rigid3.translation()) * rigid3.rotation();
}

MATCHER_P2(IsNearly, rigid, epsilon,
           std::string(std::string(negation ? "isn't nearly " : "is nearly ") + rigid.DebugString()))
{
    return ToEigen(arg).isApprox(ToEigen(rigid), epsilon);
}

#endif // MY_CARTOGRAPHER_TEST_RIGID_TRANSFORM_TEST_HELPERS_H
