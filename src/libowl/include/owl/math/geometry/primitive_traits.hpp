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
#include "owl/math/geometry/interval.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename T>
      struct dimension_t
      {
        static constexpr std::size_t value = 0;
      };

      template<>
      struct dimension_t<float>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<double>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::uint8_t>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::uint16_t>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::uint32_t>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::uint64_t>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::int8_t>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::int16_t>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::int32_t>
      {
        static constexpr std::size_t value = 1;
      };
      template<>
      struct dimension_t<std::int64_t>
      {
        static constexpr std::size_t value = 1;
      };


      template<typename T>
      struct scalar_t
      {
        using type = typename T::scalar;
      };

      template<>
      struct scalar_t<float>
      {
        using type = float;
      };
      template<>
      struct scalar_t<double>
      {
        using type = double;
      };
      template<>
      struct scalar_t<std::uint8_t>
      {
        using type = std::uint8_t;
      };
      template<>
      struct scalar_t<std::uint16_t>
      {
        using type = std::uint16_t;
      };
      template<>
      struct scalar_t<std::uint32_t>
      {
        using type = std::uint32_t;
      };
      template<>
      struct scalar_t<std::uint64_t>
      {
        using type = std::uint64_t;
      };
      template<>
      struct scalar_t<std::int8_t>
      {
        using type = std::int8_t;
      };
      template<>
      struct scalar_t<std::int16_t>
      {
        using type = std::int16_t;
      };
      template<>
      struct scalar_t<std::int32_t>
      {
        using type = std::int32_t;
      };
      template<>
      struct scalar_t<std::int64_t>
      {
        using type = std::int64_t;
      };

      template<typename Scalar, std::size_t Dimension>
      struct dimension_t<math::vector<Scalar, Dimension>>
      {
        static constexpr std::size_t value = Dimension;
      };

      template<typename Scalar, std::size_t Dimension>
      struct scalar_t<math::vector<Scalar, Dimension>>
      {
        using type = Scalar;
      };

      template<typename T>
      struct vector_t
      {
        using type = typename T::vector;
      };

      template<typename Scalar, std::size_t Dimension>
      struct vector_t<math::vector<Scalar, Dimension>>
      {
        using type = math::vector<Scalar, Dimension>;
      };

      template<typename T>
      struct point_t
      {
        using type = typename T::point;
      };

      template<typename Scalar, std::size_t Dimension>
      struct point_t<math::vector<Scalar, Dimension>>
      {
        using type = point<Scalar, Dimension>;
      };
      template<typename Scalar, std::size_t Dimension>
      struct point_t<point<Scalar, Dimension>>
      {
        using type = point<Scalar, Dimension>;
      };


    }
  }
}
