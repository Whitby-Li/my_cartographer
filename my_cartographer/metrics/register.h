//
// Created by whitby on 2025-06-08.
//

#ifndef MY_CARTOGRAPHER_METRICS_REGISTER_H
#define MY_CARTOGRAPHER_METRICS_REGISTER_H

#include "my_cartographer/metrics/family_factory.hpp"

namespace my_cartographer
{
  namespace metrics
  {

    void RegisterAllMetrics(FamilyFactory *registry);

  } // namespace metrics
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_METRICS_REGISTER_H