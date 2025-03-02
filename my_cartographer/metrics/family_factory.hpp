#ifndef MY_CARTOGRAPHER_METRICS_FAMILY_FACTORY_HPP
#define MY_CARTOGRAPHER_METRICS_FAMILY_FACTORY_HPP

#include "my_cartographer/metrics/counter.h"
#include "my_cartographer/metrics/gauge.h"
#include "my_cartographer/metrics/histogram.h"

#include <memory>
#include <string>

namespace my_cartographer
{
    namespace metrics
    {

        template <typename MetricType>
        class NullFamily;

        template <typename MetricType>
        class Family
        {
        public: // Family instance that does nothing. Safe for use in static
                // initializers.
            static Family<MetricType> *Null()
            {
                static NullFamily<MetricType> null_family;
                return &null_family;
            }

            virtual ~Family() = default;

            virtual MetricType *Add(const std::map<std::string, std::string> &labels) = 0;
        };

        template <typename MetricType>
        class NullFamily : public Family<MetricType>
        {
        public:
            MetricType *Add(const std::map<std::string, std::string> &labels) override
            {
                return MetricType::Null();
            }
        };

        class FamilyFactory
        {
        public:
            virtual ~FamilyFactory() = default;

            virtual Family<Counter> *NewCounterFamily(const std::string &name,
                                                      const std::string &description) = 0;
            virtual Family<Gauge> *NewGaugeFamily(const std::string &name,
                                                  const std::string &description) = 0;
            virtual Family<Histogram> *NewHistogramFamily(
                const std::string &name, const std::string &description,
                const Histogram::BucketBoundaries &boundaries) = 0;
        };

    } // namespace metrics
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_METRICS_FAMILY_FACTORY_HPP