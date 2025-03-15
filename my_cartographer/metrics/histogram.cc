//
// Created by whitby on 2025-03-02.
//

#include "my_cartographer/metrics/histogram.h"

#include <glog/logging.h>

namespace my_cartographer
{
    namespace metrics
    {
        namespace
        {

            // Implementation of histogram that does nothing.
            class NullHistogram : public Histogram
            {
            public:
                void Observe(double) override {}
            };

        } // namespace

        Histogram *Histogram::Null()
        {
            static NullHistogram null_histogram;
            return &null_histogram;
        }

        Histogram::BucketBoundaries Histogram::FixedWidth(double width,
                                                          int num_finite_buckets)
        {
            BucketBoundaries result;
            double boundary = 0;
            for (int i = 0; i < num_finite_buckets; ++i)
            {
                boundary += width;
                result.push_back(boundary);
            }
            return result;
        }

        Histogram::BucketBoundaries Histogram::ScaledPowersOf(double base,
                                                              double scale_factor,
                                                              double max_value)
        {
            CHECK_GT(base, 1);
            CHECK_GT(scale_factor, 0);
            BucketBoundaries result;
            double boundary = scale_factor;
            while (boundary < max_value)
            {
                result.push_back(boundary);
                boundary *= base;
            }
            return result;
        }

    } // namespace metrics
} // namespace my_cartographer