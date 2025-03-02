//
// Created by whitby on 2025-03-01.
//

#ifndef MY_CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H
#define MY_CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/transform/rigid_transform.h"
#include "my_cartographer/transform/proto/timestamped_transform.pb.h"

namespace my_cartographer
{
    namespace transform
    {
        struct TimestampedTransform
        {
            common::Time time;
            transform::Rigid3d transform;
        };

        TimestampedTransform FromProto(const proto::TimestampedTransform &proto);

        proto::TimestampedTransform ToProto(const TimestampedTransform &transform);

        TimestampedTransform Interpolate(const TimestampedTransform &start,
                                         const TimestampedTransform &end,
                                         const common::Time time);
    } // namespace transform
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H