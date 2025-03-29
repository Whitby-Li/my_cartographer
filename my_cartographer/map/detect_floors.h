//
// Created by whitby on 2025-03-25.
//

#ifndef MY_CARTOGRAPHER_DETECT_FLOORS_H
#define MY_CARTOGRAPHER_DETECT_FLOORS_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/map/proto/trajectory.pb.h"

namespace my_cartographer
{
  namespace map
  {

    struct Timespan
    {
      common::Time start;
      common::Time end;
    };

    struct Floor
    {
      // The spans of time we spent on this floor.
      // Since we might have walked up and down many times in this,
      // there can be many spans of time we spent on a particular floor.
      std::vector<Timespan> timespans;

      // The median z-value of this floor
      double z;
    };

    // Uses a heuristic which looks at z-values of the poses to detect individual floors of a building.
    // This requires that floors are *mostly* on the same z-height and that the level changes happen *relatively* abrubtly,
    // e.g. by taking the stairs.
    std::vector<Floor> DetectFloors(const proto::Trajectory &trajectory);

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_DETECT_FLOORS_H