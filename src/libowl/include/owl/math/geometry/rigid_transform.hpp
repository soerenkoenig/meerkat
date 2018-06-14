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
#include "owl/math/geometry/quaternion.hpp"
#include "owl/math/geometry/euler_angles.hpp"
#include "owl/math/geometry/trafos.hpp"
#include "owl/math/angle.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template<typename Scalar>
      class rigid_transform
      {
      public:
        using scalar = Scalar;
        using vector = vector<Scalar, 3>;
        using quaternion = quaternion<Scalar>;

        constexpr rigid_transform(quaternion rotation = quaternion::identity(), vector translation =vector::zero())
          : rotation(rotation), translation(translation)
        {
        }

        square_matrix<scalar,4> matrix() const
        {
          return translate<Scalar>(translation) * square_matrix<Scalar,4>(rotation);
        };

        static rigid_transform identity()
        {
          return rigid_transform();
        }

        void invert()
        {
          rotation.invert();
          translation = -rotation.rotate(translation);
        }

        rigid_transform inverse() const
        {
          rigid_transform trafo = *this;
          trafo.invert();
          return trafo;
        }

        void reset()
        {
          *this = identity();
        }

        rigid_transform operator*(const rigid_transform& other) const
        {
          rigid_transform res;
          res.rotation = rotation * other.rotation;
          res.translation = rotation.rotate(other.translation) + translation;
          return res;
        }

        rigid_transform& operator*=(const rigid_transform& other)
        {
          return *this = *this * other;
        }

        quaternion rotation;
        vector translation;
      };
    }
  }
}




