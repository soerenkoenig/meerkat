//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once


#include "owl/optional.hpp"
#include "ray_segment.hpp"
#include "primitive_traits.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename Primitive,
        typename Scalar = typename scalar_t<Primitive>::type,
        std::size_t Dimension = dimension_t<Primitive>::value>
      std::optional<Scalar> closest_intersection(const Primitive &prim, const ray<Scalar, Dimension> &r)
      {
        return prim.closest_intersection(r);
      };

      template<typename Primitive, typename Scalar = typename scalar_t<Primitive>::type,
        std::size_t Dimension = dimension_t<Primitive>::value>
      std::optional<Scalar> closest_intersection(const Primitive &prim, const ray_segment<Scalar, Dimension> &rs)
      {

        auto t = prim.closest_intersection(static_cast<ray<Scalar, Dimension>>(rs));
        if (!t || !rs.t_range.inside(*t))
          return std::nullopt;
        return t;
      };

      template<typename Primitive, typename Scalar = typename scalar_t<Primitive>::type,
        std::size_t Dimension = dimension_t<Primitive>::value>
      std::optional<vector<Scalar, Dimension>>
      closest_intersection_point(const Primitive &prim, const ray<Scalar, Dimension> &r)
      {
        auto t = closest_intersection(prim, r);
        if (!t)
          return std::nullopt;
        return r(*t);
      };

      template<typename Primitive, typename Scalar = typename scalar_t<Primitive>::type,
        std::size_t Dimension = dimension_t<Primitive>::value>
      std::optional<vector<Scalar, Dimension>>
      closest_intersection_point(const Primitive &prim, const ray_segment<Scalar, Dimension> &rs)
      {
        auto t = closest_intersection(prim, rs);
        if (!t)
          return std::nullopt;
        return rs(*t);
      };


      template<typename Primitive, typename Scalar = typename scalar_t<Primitive>::type,
        std::size_t Dimension = dimension_t<Primitive>::value>
      bool intersects(const Primitive &prim, const ray<Scalar, Dimension> &r)
      {
        return !!closest_intersection(prim, r);
      };

      template<typename Primitive, typename Scalar = typename scalar_t<Primitive>::type,
        std::size_t Dimension = dimension_t<Primitive>::value>
      bool intersects(const Primitive &prim, const ray_segment<Scalar, Dimension> &rs)
      {
        return !!closest_intersection(prim, rs);
      };
    }
  }
}
