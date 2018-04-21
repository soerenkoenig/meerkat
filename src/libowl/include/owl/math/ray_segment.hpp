//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include "owl/math/ray.hpp"
#include "owl/math/interval.hpp"

namespace owl
{
  namespace math
  {
    template <typename Scalar, std::size_t Dimension>
    class ray_segment: public ray<Scalar, Dimension>
    {
    public:
      using scalar = Scalar;
      using vector = vector<Scalar,Dimension>;
      using interval = math::interval<Scalar>;

      ray_segment() = default;

      ray_segment(const ray<Scalar,Dimension>& r, const scalar& t_min = 0, const scalar t_max = interval::largest)
        : ray(r)
        , t_range_{t_min, t_max}
      {}

      ray_segment(const vector& origin, const vector& direction, const scalar& t_min = 0, const scalar t_max = interval::largest)
        : ray(origin, direction)
        , t_range_{t_min, t_max}
      {}

      interval t_range = {0, interval::largest};
    };
  }
}




