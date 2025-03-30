//
// Created by whitby on 2025-03-29.
//

#ifndef MY_CARTOGRAPHER_MAP_DISPATCHABLE_HPP
#define MY_CARTOGRAPHER_MAP_DISPATCHABLE_HPP

#include "my_cartographer/map/trajectory_builder_interface.h"
#include "my_cartographer/map/data.h"

namespace my_cartographer
{
  namespace map
  {

    template <typename DataType>
    class Dispatchable : public Data
    {
    public:
      Dispatchable(const std::string &sensor_id, const DataType &data)
          : Data(sensor_id), data_(data) {}

      common::Time GetTime() const override { return data_.time; }
      void AddToTrajectoryBuilder(
          mapping::TrajectoryBuilderInterface *const trajectory_builder) override
      {
        trajectory_builder->AddSensorData(sensor_id_, data_);
      }
      const DataType &data() const { return data_; }

    private:
      const DataType data_;
    };

    template <typename DataType>
    std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
        const std::string &sensor_id, const DataType &data)
    {
      return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
    }

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_DISPATCHABLE_HPP