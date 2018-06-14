//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once
#include <random>
#include <vector>
#include <array>
#include "owl/math/geometry/point.hpp"
#include "owl/math/geometry/interval.hpp"
#include "owl/utils/random_utils.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename Scalar, typename Engine = std::mt19937>
      std::vector<point<Scalar, 3>> random_points(std::size_t n, const box<Scalar> &b =
        {point<Scalar, 3>::origin(), point<Scalar, 3>::one()},
        Engine &&engine = utils::create_seeded_engine())
      {
        std::array<std::uniform_real_distribution<Scalar>, 3> dist;

        for (std::size_t i = 0; i < 3; ++i)
          dist[i].param(
            typename std::uniform_real_distribution<Scalar>::param_type(b.lower_bound[i], b.upper_bound[i]));

        std::vector<point<Scalar, 3>> points(n);
        for (auto &p : points)
          for (std::size_t i = 0; i < 3; ++i)
            p[i] = dist[i](engine);

        return points;
      };
    }
  }
}

