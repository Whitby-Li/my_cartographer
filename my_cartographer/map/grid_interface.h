//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_GRID_INTERFACE_H
#define MY_CARTOGRAPHER_MAP_GRID_INTERFACE_H

#include <memory>
#include <utility>

namespace my_cartographer
{
  namespace map
  {

    class GridInterface
    {
      // todo(kdaun) move mutual functions of Grid2D/3D here
    public:
      virtual ~GridInterface() {}
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_GRID_INTERFACE_H