//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once
#include "owl/math/matrix.hpp"

namespace owl
{
  namespace math
  {
    template <typename Scalar, std::size_t Dimension>
    const vector<Scalar,Dimension>& closest_point(const vector<Scalar,Dimension>& prim, const vector<Scalar,Dimension>&)
    {
      return prim;
    };

    template <typename Primitive, typename Point>
    auto closest_point(const Primitive &prim, const Point &p)
    {
      return prim.closest_point(p);
    };

    template <typename Primitive, typename Point>
    auto sqr_distance(const Primitive &prim, const Point &p)
    {
      return sqr_distance(prim.closest_point(p), p);
    };

    template <typename Primitive, typename Point>
    auto distance(const Primitive &prim, const Point &p)
    {
      return std::sqrt(sqr_distance(prim, p));
    };
  }
}

