//
// Created by whitby on 2025-03-08.
//

#ifndef MY_CARTOGRAPHER_SENSOR_LANDMARK_DATA_H
#define MY_CARTOGRAPHER_SENSOR_LANDMARK_DATA_H

#include "my_cartographer/common/port.hpp"
#include "my_cartographer/common/time.h"
#include "my_cartographer/sensor/proto/sensor.pb.h"
#include "my_cartographer/transform/rigid_transform.h"

namespace my_cartographer
{
    namespace sensor
    {
        struct LandmarkObservation
        {
            std::string id;
            transform::Rigid3d landmark_to_tracking_transform;
            double translation_weight;
            double rotation_weight;
        };

        struct LandmarkData
        {
            common::Time time;
            std::vector<LandmarkObservation> landmark_observations;
        };

        // Converts 'landmark_data' to a proto::LandmarkData.
        proto::LandmarkData ToProto(const LandmarkData &landmark_data);

        // Converts 'proto' to LandmarkData.
        LandmarkData FromProto(const proto::LandmarkData &proto);
        
    }  // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_SENSOR_LANDMARK_DATA_H