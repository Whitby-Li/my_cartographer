//
// Created by whitby on 2025-03-08.
//

#ifndef MY_CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_HPP
#define MY_CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_HPP

#include "my_cartographer/sensor/proto/sensor.pb.h"
#include "my_cartographer/transform/transform.h"

#include <Eigen/Core>
#include <glog/logging.h>

#include <vector>

namespace my_cartographer
{
    namespace sensor
    {

        // Stores 3d position of a point observed by a rangefinder sensor.
        struct RangefinderPoint
        {
            Eigen::Vector3f position;
        };

        // Stores 3d position of a point with its reletive measurement time.
        // See point_cloud.h for more details.
        struct TimedRangefinderPoint
        {
            Eigen::Vector3d position;
            float time;
        };

        template <class T>
        inline RangefinderPoint operator*(const transform::Rigid3<T> &lhs, const RangefinderPoint &rhs)
        {
            RangefinderPoint result = rhs;
            result.position = lhs * rhs.position;
            return result;
        }

        template <class T>
        inline TimedRangefinderPoint operator*(const transform::Rigid3<T> &lhs, const TimedRangefinderPoint &rhs)
        {
            TimedRangefinderPoint result = rhs;
            result.position = lhs * rhs.position;
            return result;
        }

        inline bool operator==(const RangefinderPoint &lhs, const RangefinderPoint &rhs)
        {
            return lhs.position == rhs.position;
        }

        inline bool operator==(const TimedRangefinderPoint &lhs, const TimedRangefinderPoint &rhs)
        {
            return lhs.position == rhs.position && lhs.time == rhs.time;
        }

        inline RangefinderPoint FromProto(const proto::RangefinderPoint &rangefinder_point_proto)
        {
            proto::RangefinderPoint proto;
            *proto.mutable_position() = transform::ToProto(rangefinder_point.position);
            return proto;
        }

        inline proto::RangefinderPoint ToProto(const RangefinderPoint &rangefinder_point)
        {
            proto::RangefinderPoint proto;
            *proto.mutable_position() = transform::ToProto(rangefinder_point.position);
            return proto;
        }

        inline TimedRangefinderPoint FromProto(const proto::TimedRangefinderPoint &timed_rangefinder_point_proto)
        {
            return {transform::ToEigen(timed_rangefinder_point_proto.position()),
                    timed_rangefinder_point_proto.time()};
        }

        inline proto::TimedRangefinderPoint ToProto(const TimedRangefinderPoint &timed_rangefinder_point)
        {
            proto::TimedRangefinderPoint proto;
            *proto.mutable_position() = transform::ToProto(timed_rangefinder_point.position);
            proto.set_time(timed_rangefinder_point.time);
            return proto;
        }

        inline RangefinderPoint ToRangefinderPoint(const TimedRangefinderPoint &timed_rangefinder_point)
        {
            return {timed_rangefinder_point.position};
        }

        inline TimedRangefinderPoint ToTimedRangefinderPoint(const RangefinderPoint &rangefinder_point, const float time)
        {
            return {rangefinder_point.position, time};
        }

    } // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_HPP