//
// Created by whitby on 2025-03-29.
//

#ifndef MY_CARTOGRAPHER_SENSOR_DATA_H
#define MY_CARTOGRAPHER_SENSOR_DATA_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <absl/memory/memory.h>

namespace my_cartographer {
namespace map {

  class TrajectoryBuilderInterface;

  class Data
  {
  public:
    explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
    virtual ~Data() {}

    virtual common::Time GetTime() const = 0;
    const std::string &GetSensorId() const { return sensor_id_; }
    virtual void AddToTrajectoryBuilder(
        map::TrajectoryBuilderInterface *trajectory_builder) = 0;

  protected:
    const std::string sensor_id_;
  };

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H