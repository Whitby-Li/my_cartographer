//
// Created by whitby on 2025-03-02.
//

#include "my_cartographer/metrics/counter.h"

namespace my_cartographer
{
    namespace metrics
    {
        
        namespace
        {

            // Implementation of counter that does nothing.
            class NullCounter : public Counter
            {
            public:
                void Increment() override {};
                void Increment(double) override {};
            };

        } // namespace

        Counter *Counter::Null()
        {
            static NullCounter null_counter;
            return &null_counter;
        }

    } // namespace metrics
} // namespace my_cartographer