//
// Created by whitby on 2025-03-01.
//

#include "my_cartographer/transform/rigid_transform.h"
#include "my_cartographer/common/lua_parameter_dictionary.h"

#include <Eigen/Geometry>
#include <glog/logging.h>

#include <vector>

namespace my_cartographer
{
  namespace transform
  {
    namespace
    {
      Eigen::Vector3d TranslationFromDictionary(
          common::LuaParameterDictionary *dictionary)
      {
        const std::vector<double> translation = dictionary->GetArrayValuesAsDoubles();
        CHECK_EQ(3, translation.size()) << "Need (x, y, z) for translation.";
        return Eigen::Vector3d(translation[0], translation[1], translation[2]);
      }
    } // namespace

    Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw)
    {
      return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    }

    Rigid3d FromDictionary(common::LuaParameterDictionary *dictionary)
    {
      const Eigen::Vector3d translation = TranslationFromDictionary(dictionary->GetDictionary("translation").get());

      auto rotation_dictionary = dictionary->GetDictionary("rotation");
      if (rotation_dictionary->HasKey("w"))
      {
        const Eigen::Quaterniond rotation(rotation_dictionary->GetDouble("w"),
                                          rotation_dictionary->GetDouble("x"),
                                          rotation_dictionary->GetDouble("y"),
                                          rotation_dictionary->GetDouble("z"));
        CHECK_NEAR(rotation.norm(), 1., 1e-9);
        return Rigid3d(translation, rotation);
      }
      else
      {
        const std::vector<double> rotation = rotation_dictionary->GetArrayValuesAsDoubles();
        CHECK_EQ(3, rotation.size()) << "Need (roll, pitch, yaw) for rotation.";
        return Rigid3d(translation, RollPitchYaw(rotation[0], rotation[1], rotation[2]));
      }
    }

  } // namespace transform
} // namespace my_cartographer