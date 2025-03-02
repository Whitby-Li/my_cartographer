//
// Created by whitby on 2025-03-02.
//

#include "my_cartographer/metrics/gauge.h"

namespace my_cartographer
{
    namespace metrics
    {

        namespace
        {

            // Implementation of gauge that does nothing.
            class NullGauge : public Gauge
            {
            public:
                void Increment() override {};
                void Increment(double) override {};
                void Decrement() override {};
                void Decrement(double) override {};
                void Set(double) override {};
            };

        } // namespace

        Gauge *Gauge::Null()
        {
            static NullGauge null_gauge;
            return &null_gauge;
        }

    } // namespace metrics
} // namespace my_cartographer