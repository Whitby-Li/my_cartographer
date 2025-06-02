//
// Created by whitby on 2025-06-02.
//

#pragma once

#include <Eigen/Geometry>
#include <cairo/cairo.h>

#include "my_cartographer/io/image.h"
#include "my_cartographer/io/proto_stream_deserializer.h"
#include "my_cartographer/map/id.hpp"
#include "my_cartographer/map/proto/serialization.pb.h"
#include "my_cartographer/map/value_conversion_tables.h"
#include "my_cartographer/transform/rigid_transform.h"

namespace my_cartographer
{
  namespace io
  {

    struct PaintSubmapSlicesResult
    {
      PaintSubmapSlicesResult(my_cartographer::io::UniqueCairoSurfacePtr surface,
                              Eigen::Array2f origin)
          : surface(std::move(surface)), origin(origin) {}
      my_cartographer::io::UniqueCairoSurfacePtr surface;

      // Top left pixel of 'surface' in map frame.
      Eigen::Array2f origin;
    };

    struct SubmapSlice
    {
      SubmapSlice()
          : surface(my_cartographer::io::MakeUniqueCairoSurfacePtr(nullptr)) {}

      // Texture data.
      int width;
      int height;
      int version;
      double resolution;
      my_cartographer::transform::Rigid3d slice_pose;
      my_cartographer::io::UniqueCairoSurfacePtr surface;
      // Pixel data used by 'surface'. Must outlive 'surface'.
      std::vector<uint32_t> cairo_data;

      // Metadata.
      my_cartographer::transform::Rigid3d pose;
      int metadata_version = -1;
    };

    struct SubmapTexture
    {
      struct Pixels
      {
        std::vector<char> intensity;
        std::vector<char> alpha;
      };
      Pixels pixels;
      int width;
      int height;
      double resolution;
      my_cartographer::transform::Rigid3d slice_pose;
    };

    struct SubmapTextures
    {
      int version;
      std::vector<SubmapTexture> textures;
    };

    PaintSubmapSlicesResult PaintSubmapSlices(
        const std::map<my_cartographer::map::SubmapId, SubmapSlice> &submaps,
        double resolution);

    void FillSubmapSlice(
        const my_cartographer::transform::Rigid3d &global_submap_pose,
        const my_cartographer::map::proto::Submap &proto,
        SubmapSlice *const submap_slice,
        my_cartographer::map::ValueConversionTables *conversion_tables);

    void DeserializeAndFillSubmapSlices(
        ProtoStreamDeserializer *deserializer,
        std::map<my_cartographer::map::SubmapId, SubmapSlice> *submap_slices,
        map::ValueConversionTables *conversion_tables);

    // Unpacks cell data as provided by the backend into 'intensity' and 'alpha'.
    SubmapTexture::Pixels UnpackTextureData(const std::string &compressed_cells,
                                            int width, int height);

    // Draw a texture into a cairo surface. 'cairo_data' will store the pixel data
    // for the surface and must therefore outlive the use of the surface.
    UniqueCairoSurfacePtr DrawTexture(const std::vector<char> &intensity,
                                      const std::vector<char> &alpha, int width,
                                      int height,
                                      std::vector<uint32_t> *cairo_data);
  }
}