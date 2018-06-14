//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2017 Sören König. All rights reserved.
//

#pragma once
#include "owl/optional.hpp"
#include "owl/math/geometry/point.hpp"
#include "ray.hpp"
#include "interval.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template <typename Scalar, std::size_t Dimension>
      class line_segment
      {
      public:
        using scalar = Scalar;
        using vector = vector<Scalar, Dimension>;
        using point = point<Scalar, Dimension>;

        line_segment() = default;

        line_segment(const point& start, const point& end)
          : vertices{start, end}
        {
        }

        bool operator==(const line_segment& other) const
        {
          return vertices == other.vertices;
        }

        bool operator!=(const line_segment& other) const
        {
          return vertices != other.vertices;
        }

        bool operator<(const line_segment& other) const
        {
          return vertices < other.vertices;
        }

        bool operator<=(const line_segment& other) const
        {
          return vertices <= other.vertices;
        }

        bool operator>(const line_segment& other) const
        {
          return vertices > other.vertices;
        }

        bool operator>=(const line_segment& other) const
        {
          return vertices >= other.vertices;
        }


        void closest_point_barycentric(const point& p, scalar& l0, scalar& l1) const
        {
          vector edge0 = end - start;
          vector d0 = p - start;
          vector d1 = p - end;

          auto a = dot(edge0, d0);
          auto b = dot(-edge0, d1);
          auto c = dot(edge0, edge0);

          if(a < 0)
          {
            l1 = 0;
            l0 = 1;
          }
          else if(b < 0)
          {
            l1 = 1;
            l0 = 0;
          }
          else
          {
            l1 = a/c;
            l0 = 1-l1;
          }
        }

        point closest_point(const point& q) const
        {
          Scalar l0,l1;
          closest_point_barycentric(q,l0,l1);
          return l0 * start + l1 * end;
        };


        union
        {
          std::array<point, 2> vertices;
          struct
          {
            point start;
            point end;
          };
        };
      };

      template <typename Scalar, std::size_t Dimension>
      point<Scalar,Dimension> reference_point(const line_segment<Scalar,Dimension>& l)
      {
        return l.start + (l.end - l.start) / Scalar(2);
      }

      template <typename Scalar, std::size_t Dimension, bool LowerBoundOpen = false, bool UpperBoundOpen = false>
      interval<Scalar, Dimension, LowerBoundOpen, UpperBoundOpen> bounds(const line_segment<Scalar, Dimension>& l)
      {
        interval<Scalar,Dimension, LowerBoundOpen, UpperBoundOpen> b;
        b.insert(l.start);
        b.insert(l.end);
        return b;
      }

      template <typename Scalar, std::size_t Dimension>
      std::ostream& operator<<(std::ostream& out, const line_segment<Scalar, Dimension>& l)
      {
        return out << "(" <<l.start << ") -> ("<< l.end << ")";
      }

      using line_segment3f = line_segment<float, 3>;
      using line_segment3d = line_segment<double, 3>;
      using line_segment2f = line_segment<float, 2>;
      using line_segment2d = line_segment<double, 2>;
    }
  }
}

