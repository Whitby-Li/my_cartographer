//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_LOCAL_SLAM_RESULT_DATA_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_LOCAL_SLAM_RESULT_DATA_H

#include "my_cartographer/map/pose_graph.h"
#include "my_cartographer/sensor/data.h"

namespace my_cartographer
{
  namespace map
  {
    class LocalSlamResultData : public sensor::Data
    {
    public:
      LocalSlamResultData(const std::string &sensor_id, common::Time time)
          : Data(sensor_id), time_(time) {}

      common::Time GetTime() const override { return time_; }
      virtual void AddToPoseGraph(int trajectory_id,
                                  PoseGraph *pose_graph) const = 0;

    private:
      common::Time time_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_LOCAL_SLAM_RESULT_DATA_H