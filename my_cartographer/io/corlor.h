//
// Created by whitby on 2025-05-05.
//

#ifndef MY_CARTOGRAPHER_IO_CORLOR_H_
#define MY_CARTOGRAPHER_IO_CORLOR_H_

#include <array>

#include "my_cartographer/common/math.hpp"
#include "my_cartographer/common/port.hpp"

namespace my_cartographer
{
  namespace io
  {

    using Uint8Color = std::array<uint8, 3>;
    using FloatColor = std::array<float, 3>;

    // A function for on-demand generation of a color palette, with every two
    // direct successors having large contrast.
    FloatColor GetColor(int id);

    inline uint8 FloatComponentToUint8(float c)
    {
      return static_cast<uint8>(common::RoundToInt(common::Clamp(c, 0.f, 1.f) * 255));
    }

    inline float Uint8ComponentToFloat(uint8 c) { return c / 255.f; }

    inline Uint8Color ToUint8Color(const FloatColor &color)
    {
      return {{FloatComponentToUint8(color[0]), FloatComponentToUint8(color[1]),
               FloatComponentToUint8(color[2])}};
    }

    inline FloatColor ToFloatColor(const Uint8Color &color)
    {
      return {{Uint8ComponentToFloat(color[0]), Uint8ComponentToFloat(color[1]),
               Uint8ComponentToFloat(color[2])}};
    }

  } // namespace io
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_IO_CORLOR_H_