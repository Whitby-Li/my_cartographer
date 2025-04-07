//
// Created by whitby on 2025-04-06.
//

#include "my_cartographer/map/internal/2d/scan_matching/interpolated_tsdf_2d.hpp"
#include "my_cartographer/sensor/point_cloud.h"

#include <Eigen/Geometry>
#include <ceres/ceres.h>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matchking
    {

      // Computes a cost for matching the 'point_cloud' in the 'grid' at
      // a 'pose'. The cost increases with the signed distance of the matched point
      // location in the 'grid'.
      class TSDFMatchCostFunction2D
      {
      public:
        TSDFMatchCostFunction2D(const double residual_scaling_factor,
                                const sensor::PointCloud &point_cloud,
                                const TSDF2D &grid)
            : residual_scaling_factor_(residual_scaling_factor),
              point_cloud_(point_cloud),
              interpolated_grid_(grid) {}

        template <typename T>
        bool operator()(const T *const pose, T *residual) const
        {
          const Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
          const Eigen::Rotation2D<T> rotation(pose[2]);
          const Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
          Eigen::Matrix<T, 3, 3> transform;
          transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

          T summed_weight = T(0);
          for (size_t i = 0; i < point_cloud_.size(); ++i)
          {
            // Note that this is a 2D point. The third component is a scaling factor.
            const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                               (T(point_cloud_[i].position.y())),
                                               T(1.));
            const Eigen::Matrix<T, 3, 1> world = transform * point;
            const T point_weight = interpolated_grid_.GetWeight(world[0], world[1]);
            summed_weight += point_weight;
            residual[i] =
                T(point_cloud_.size()) * residual_scaling_factor_ *
                interpolated_grid_.GetCorrespondenceCost(world[0], world[1]) *
                point_weight;
          }
          if (summed_weight == T(0))
            return false;
          for (size_t i = 0; i < point_cloud_.size(); ++i)
          {
            residual[i] /= summed_weight;
          }
          return true;
        }

      private:
        TSDFMatchCostFunction2D(const TSDFMatchCostFunction2D &) = delete;
        TSDFMatchCostFunction2D &operator=(const TSDFMatchCostFunction2D &) = delete;

        const double residual_scaling_factor_;
        const sensor::PointCloud &point_cloud_;
        const InterpolatedTSDF2D interpolated_grid_;
      };

      ceres::CostFunction *CreateTSDFMatchCostFunction2D(
          const double scaling_factor, const sensor::PointCloud &point_cloud,
          const TSDF2D &tsdf)
      {
        return new ceres::AutoDiffCostFunction<TSDFMatchCostFunction2D,
                                               ceres::DYNAMIC /* residuals */,
                                               3 /* pose variables */>(
            new TSDFMatchCostFunction2D(scaling_factor, point_cloud, tsdf),
            point_cloud.size());
      }

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer