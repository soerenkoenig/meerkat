//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include <vector>

#include "owl/math/matrix.hpp"
#include "owl/color/color.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename Scalar>
      class dense_tsdf_volume
      {
      public:
        using scalar = Scalar;
        using vector = math::vector<scalar, 3>;
        using index = math::vector<std::int32_t, 3>;
        using color = color::rgb32f;


        dense_tsdf_volume(scalar length, std::size_t resolution, scalar sdf_trunc, bool with_color,
          const vector &origin = vector::zero())
          : voxel_length_(length / static_cast<scalar>(resolution)), half_voxel_length_(voxel_length_ / 2),
            resolution_(resolution), length_(length), sdf_trunc_(sdf_trunc), with_color_(with_color), origin_(origin),
            voxel_num_(resolution * resolution * resolution), tsdf_(voxel_num_, 0),
            color_(with_color ? voxel_num_ : 0, color(0, 0, 0)), weight_(voxel_num_)
        {}

        void clear()
        {
          std::memset(tsdf_.data(), 0, voxel_num_ * 4);
          std::memset(weight_.data(), 0, voxel_num_ * 4);
          if (with_color_)
          {
            std::memset(color_.data(), 0, voxel_num_ * 12);
          }
        }

        vector center(const index &voxel_index) const
        {
          return {half_voxel_length_ + voxel_length_ * voxel_index.x(),
                  half_voxel_length_ + voxel_length_ * voxel_index.y(),
                  half_voxel_length_ + voxel_length_ * voxel_index.z()};
        }

        vector normal(const vector &p) const
        {
          vector n;
          const scalar half_gap = scalar(0.99) * voxel_length_;
          for (int i = 0; i < 3; i++)
          {
            vector p0 = p;
            p0(i) -= half_gap;
            vector p1 = p;
            p1(i) += half_gap;
            n(i) = tdsf(p1) - tdsf(p0);
          }
          return n.normalized();
        }


      private:


        inline scalar tsdf(const index &idx) const
        {
          return tsdf_[(idx.x() * resolution_ + idx.y()) * resolution_ + idx.z()];
        }

        scalar tdsf(const vector &p) const
        {
          index idx;
          vector p_grid = p / voxel_length_ - vector::constant(scalar{0.5});
          for (int i = 0; i < 3; i++)
          {
            idx(i) = static_cast<int>(std::floor(p_grid(i)));
          }

          vector r = p_grid - vector(idx);
          return (1 - r(0)) * (
            (1 - r(1)) * (
              (1 - r(2)) * tsdf(idx + index(0, 0, 0))) +
            r(2) * tsdf(idx + index(0, 0, 1))
          ) + r(1) * (
            (1 - r(2)) * tsdf(idx + index(0, 1, 0)) +
            r(2) * tsdf((idx + index(0, 1, 1))
            )) + r(0) * (
            (1 - r(1)) * (
              (1 - r(2)) * tsdf(idx + index(1, 0, 0)) +
              r(2) * tsdf(idx + index(1, 0, 1))
            ) + r(1) * (
              (1 - r(2)) * tsdf(idx + index(1, 1, 0)) +
              r(2) * tsdf(idx + index(1, 1, 1))
            ));
        }

        /**
         *
         * @return side length of a single voxel cell
         */
        const scalar &voxel_length() const
        {
          return voxel_length_;
        }

      private:
        scalar voxel_length_;
        scalar half_voxel_length_;
        scalar sdf_trunc_;
        bool with_color_;
        math::vector<scalar, 3> origin_;
        double length_;
        std::size_t resolution_;
        std::size_t voxel_num_;
        std::vector<Scalar> tsdf_;
        std::vector<color> color_;
        std::vector<float> weight_;
      };
    }
  }
}
