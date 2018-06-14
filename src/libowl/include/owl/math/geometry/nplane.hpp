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

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename Scalar, std::size_t Dimension>
      class nplane
      {
      public:
        using scalar = Scalar;
        using vector = vector<Scalar, Dimension>;
        using point = point<Scalar, Dimension>;
        using homog_vector = math::vector<Scalar, Dimension + 1>;

        template<typename S1>
        using enable_if_scalar_t = typename vector::template enable_if_scalar_t<S1>;

        nplane() = default;

        nplane(const homog_vector &arr)
          : data_(arr)
        {
          normalize();
        }

        template<typename S, typename... Args, typename = enable_if_scalar_t<S> >
        nplane(S &&nx, Args &&... args)
          : data_{std::forward<S>(nx), std::forward<Args>(args)...}
        {
          normalize();
        }


        std::optional<Scalar> closest_intersection(const ray<Scalar, Dimension> r) const
        {
          Scalar denom = dot(normal(), r.direction());
          if (denom == 0)
            return std::nullopt;

          Scalar t = (dot(normal(), r.origin) - distance());
          if (t < 0)
            return std::nullopt;
          return t / denom;
        }

        const vector &normal() const
        {
          return reinterpret_cast<const vector &>(data_);
        }

        scalar distance() const
        {
          return -data_[3];
        }

        scalar operator()(const vector &vec)
        {
          return dot(normal(), vec) + data_[3];
        }

        scalar operator()(const homog_vector &hvec)
        {
          return dot(data_, hvec);
        }

      private:
        void normalize()
        {
          auto len = normal().length();
          if (len != 0)
            data_ /= len;
        }

        homog_vector data_;
      };

      template<typename Scalar>
      using line = nplane<Scalar, 2>;

      template<typename Scalar>
      using plane = nplane<Scalar, 3>;


      using planef = plane<float>;
      using planed = plane<double>;
      using linef = line<float>;
      using lined = line<double>;


      template<typename Scalar, std::size_t Dimension>
      nplane<Scalar, Dimension> nplane_from_point_and_normal(const point<Scalar, Dimension> &p,
        const vector<Scalar, Dimension> &normal)
      {
        vector<Scalar, Dimension + 1> coeffs;
        coeffs << normal, -dot<Scalar,Dimension,1>(normal, p);
        return nplane<Scalar, Dimension>(coeffs);
      }

      template<typename Scalar, std::size_t Dimension>
      Scalar distance(const nplane<Scalar, Dimension> &pl, const vector<Scalar, Dimension> &pnt)
      {
        return dot(pl.normal(), pnt) - pl.distance();
      }

    }
  }
}
