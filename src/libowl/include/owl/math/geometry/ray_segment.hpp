//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include "owl/math/geometry/ray.hpp"
#include "owl/math/geometry/interval.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename Scalar, std::size_t Dimension>
      class ray_segment : public ray<Scalar, Dimension>
      {
      public:
        using scalar = Scalar;
        using vector = math::vector<Scalar, Dimension>;
        using point = geometry::point<Scalar, Dimension>;
        using interval = geometry::interval<Scalar>;

        ray_segment() = default;

        ray_segment(const geometry::ray<Scalar, Dimension> &r, const scalar &t_min = 0,
          const scalar t_max = interval::largest)
          : ray<Scalar, Dimension>(r), t_range{t_min, t_max}
        {}

        ray_segment(const vector &origin, const vector &direction, const scalar &t_min = 0,
          const scalar t_max = interval::largest)
          : ray<Scalar, Dimension>(origin, direction), t_range{t_min, t_max}
        {}

        interval t_range = {0, interval::largest};
      };

      using ray_segment2f = ray_segment<float, 2>;
      using ray_segment2d = ray_segment<double, 2>;
      using ray_segment3f = ray_segment<float, 3>;
      using ray_segment3d = ray_segment<double, 3>;
    }
  }
}




