//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2018 Sören König. All rights reserved.
//

#pragma once

#include <array>
#include "owl/math/geometry/point.hpp"
#include "owl/math/geometry/interval.hpp"
#include "owl/math/geometry/ray.hpp"
#include "owl/math/utils.hpp"
#include "owl/math/statistics.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename Scalar, std::size_t Dimension>
      class triangle
      {
        static_assert(Dimension > 1, "invalid template parameter N");
      public:
        using scalar = Scalar;
        using vector = math::vector<Scalar, Dimension>;
        using point = point<Scalar, Dimension>;
        using barycentric = math::vector<Scalar, 3>;
        using ray = ray<Scalar, Dimension>;
        using real = std::conditional_t<std::is_same<scalar, double>::value, double, float>;

        triangle() = default;

        triangle(const point &v0, const point &v1, const point &v2)
          : vertices{{v0, v1, v2}}
        {}


        template<bool Cond = (Dimension == 3), typename = std::enable_if_t<Cond>>
        vector normal() const
        {
          return normalize(cross(v1 - v0, v2 - v0));
        }

        vector bary_2_cart(const barycentric &lambda) const
        {
          return lambda.x() * v0 + lambda.y() * v1 + lambda.z() * v2;
        }

        template<bool Cond = (Dimension == 2), typename = std::enable_if_t<Cond>>
        bool inside(const point &pnt) const
        {
          auto lambdas = edge_funcs(pnt);
          for (const auto &l: lambdas)
            if (l < 0)
              return false;
          return true;
        }

        template<bool Cond = (Dimension > 2), typename = std::enable_if_t<Cond>, typename = void>
        bool inside(const point &pnt, std::size_t u, std::size_t v) const
        {
          auto lambdas = edge_funcs(pnt, u, v);
          for (const auto &l: lambdas)
            if (l < 0)
              return false;
          return true;
        }

        point closest_point_barycentric(const point &p) const
        {
          vector e0 = v1 - v0;
          vector e1 = v2 - v0;
          vector v = v0 - p;

          scalar a = dot(e0, e0);
          scalar b = dot(e0, e1);
          scalar c = dot(e1, e1);
          scalar d = dot(e0, v);
          scalar e = dot(e1, v);

          scalar det = a * c - b * b;
          scalar s = b * e - c * d;
          scalar t = b * d - a * e;

          if (s + t < det)
          {
            if (s < 0)
            {
              if (t < 0)
              {
                if (d < 0)
                {
                  s = -d / a;
                  s = (std::min)((std::max)(s, (scalar) 0), (scalar) 1);
                  t = 0;
                } else
                {
                  s = 0;
                  t = -e / c;
                  t = (std::min)((std::max)(t, (scalar) 0), (scalar) 1);

                }
              } else
              {
                s = 0;
                t = -e / c;
                t = (std::min)((std::max)(t, (scalar) 0), (scalar) 1);
              }
            } else if (t < 0)
            {
              s = -d / a;
              s = (std::min)((std::max)(s, (scalar) 0), (scalar) 1);
              t = 0.f;
            } else
            {
              scalar invDet = 1 / det;
              s *= invDet;
              t *= invDet;
            }
          } else
          {
            if (s < 0)
            {
              scalar tmp0 = b + d;
              scalar tmp1 = c + e;
              if (tmp1 > tmp0)
              {
                scalar numer = tmp1 - tmp0;
                scalar denom = a - 2 * b + c;
                s = numer / denom;
                s = (std::min)((std::max)(s, (scalar) 0), (scalar) 1);
                t = 1 - s;
              } else
              {
                t = -e / c;
                t = (std::min)((std::max)(t, (scalar) 0), (scalar) 1);
                s = 0.f;
              }
            } else if (t < 0.f)
            {
              if (a + d > b + e)
              {
                scalar numer = c + e - b - d;
                scalar denom = a - 2 * b + c;
                s = numer / denom;
                s = (std::min)((std::max)(s, (scalar) 0), (scalar) 1);

                t = 1 - s;
              } else
              {
                s = -e / c;
                s = (std::min)((std::max)(s, scalar(0)), (scalar) 1);
                t = 0;
              }
            } else
            {
              scalar numer = c + e - b - d;
              scalar denom = a - 2 * b + c;

              s = numer / denom;
              s = (std::min)((std::max)(s, (scalar) 0), (scalar) 1);
              t = 1.f - s;
            }
          }

          return {1 - s - t, s, t};
        }

        point closest_point(const point &p) const
        {
          return bary_2_cart(closest_point_barycentric(p));
        }


        std::optional<scalar> closest_intersection(const ray &r) const
        {
          vector tvec, pvec, qvec;

          const vector &dir = r.direction();
          vector e1 = v1 - v0;
          vector e2 = v2 - v0;

          pvec = cross(dir, e2);
          scalar det = dot(e1, pvec);

          scalar eps2 = 0;

          if (det == 0)
            return std::nullopt;

          scalar inv_det = scalar{1} / det;
          tvec = r.origin - v0;

          scalar u = dot(tvec, pvec) * inv_det;
          if (u < 0 || u > 1)
            return std::nullopt;

          qvec = cross(tvec, e1);

          scalar v = dot(dir, qvec) * inv_det;
          if (v < 0 || u + v > 1)
            return std::nullopt;

          return dot(e2, qvec) * inv_det;
        }

        bool operator==(const triangle &other) const
        {
          return vertices == other.vertices;
        }

        bool operator!=(const triangle &other) const
        {
          return vertices != other.vertices;
        }

        bool operator<(const triangle &other) const
        {
          return vertices < other.vertices;
        }

        bool operator<=(const triangle &other) const
        {
          return vertices <= other.vertices;
        }

        bool operator>(const triangle &other) const
        {
          return vertices > other.vertices;
        }

        bool operator>=(const triangle &other) const
        {
          return vertices >= other.vertices;
        }


      private:

        template<bool Cond = (Dimension == 2), typename = std::enable_if_t<Cond>>
        inline scalar edge_func(const point &a, const point &b, const point &c)
        {
          return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
        }

        template<bool Cond = (Dimension > 2), typename = std::enable_if_t<Cond>, typename = void>
        inline scalar edge_func(const point &a, const point &b, const point &c, std::size_t u, std::size_t v)
        {
          return (c[u] - a[u]) * (b[v] - a[v]) - (c[v] - a[v]) * (b[u] - a[u]);
        }

        template<bool Cond = (Dimension == 2), typename = std::enable_if_t<Cond>>
        inline math::vector<real, 3> edge_func(const point &pnt) const
        {
          return math::vector<scalar, 3>(edge_function(v1, v2, pnt),
                                         edge_func(v2, v0, pnt), edge_func(v0, v1, pnt));
        }

        template<bool Cond = (Dimension > 2), typename = std::enable_if_t<Cond>, typename = void>
        inline math::vector<real, 3> edge_func(const point &pnt, std::size_t u, std::size_t v) const
        {
          return math::vector<scalar, 3>(edge_function(v1, v2, pnt, u, v),
                                         edge_func(v2, v0, pnt, u, v), edge_func(v0, v1, pnt, u, v));
        }

      public:

        union
        {
          std::array<point, 3> vertices;
          struct
          {
            point v0, v1, v2;
          };
        };
      };


      template<typename Scalar, std::size_t Dimension>
      point<Scalar, Dimension> reference_point(const triangle<Scalar, Dimension> &tri)
      {
        return mean(tri.vertices);
      }

      template<typename Scalar, std::size_t Dimension, bool LowerBoundOpen = false, bool UpperBoundOpen = false>
      interval <Scalar, Dimension, LowerBoundOpen, UpperBoundOpen> bounds(const triangle<Scalar, Dimension> &tri)
      {
        interval<Scalar, Dimension, LowerBoundOpen, UpperBoundOpen> b;
        b.insert(tri.v0);
        b.insert(tri.v1);
        b.insert(tri.v2);
        return b;
      }

      template<typename Scalar, std::size_t Dimension>
      std::ostream &operator<<(std::ostream &out, const triangle<Scalar, Dimension> &tri)
      {
        return out << "(" << tri.v0 << ") , (" << tri.v1 << "), (" << tri.v2 << ")";
      }

      using triagnle2f = triangle<float, 2>;
      using triangle3f = triangle<float, 3>;
      using triangle2d = triangle<double, 2>;
      using triangle3d = triangle<double, 3>;
    }
  }
}
