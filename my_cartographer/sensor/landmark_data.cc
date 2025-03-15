//
// Created by whitby on 2025-03-08.
//

#include "my_cartographer/sensor/landmark_data.h"
#include "my_cartographer/transform/transform.h"

namespace my_cartographer
{
    namespace sensor
    {
        proto::LandmarkData ToProto(const LandmarkData &landmark_data)
        {
            proto::LandmarkData proto;
            proto.set_timestamp(common::ToUniversal(landmark_data.time));
            for (const auto &observation : landmark_data.landmark_observations)
            {
                auto *item = proto.add_landmark_observations();
                item->set_id(observation.id);
                *item->mutable_landmark_to_tracking_transform() =
                    transform::ToProto(observation.landmark_to_tracking_transform);
                item->set_translation_weight(observation.translation_weight);
                item->set_rotation_weight(observation.rotation_weight);
            }
            return proto;
        }

        LandmarkData FromProto(const proto::LandmarkData &proto)
        {
            LandmarkData landmark_data;
            landmark_data.time = common::FromUniversal(proto.timestamp());
            for (const auto &item : proto.landmark_observations())
            {
                landmark_data.landmark_observations.push_back({
                    item.id(),
                    transform::ToRigid3(item.landmark_to_tracking_transform()),
                    item.translation_weight(),
                    item.rotation_weight(),
                });
            }
            return landmark_data;
        }

    } // namespace sensor
} // namespace my_cartographer