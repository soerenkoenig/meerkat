//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once
#include "owl/math/geometry/point.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {

      template<typename Primitive>
      auto closest_point(const Primitive &prim, const typename point_t<Primitive>::type &p)
      {
        return prim.closest_point(p);
      };

      template<typename Scalar,std::size_t Dimension>
      Scalar sqr_distance(const point<Scalar,Dimension> &p1, const point<Scalar,Dimension> &p2)
      {
        return (p1 - p2).sqr_length();
      };

      template<typename Primitive>
      typename scalar_t<Primitive>::type sqr_distance(const Primitive &prim, const typename point_t<Primitive>::type &p)
      {
        return sqr_distance(closest_point<Primitive>(prim,p), p);
      };

      template<typename Primitive>
      auto distance(const Primitive &prim, const typename point_t<Primitive>::type &p)
      {
        return std::sqrt(sqr_distance(prim, p));
      };
    }
  }
}

