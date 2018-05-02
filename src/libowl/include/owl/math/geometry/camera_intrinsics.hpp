//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include <iostream>

#include "owl/math/matrix.hpp"
#include "owl/math/geometry/interval.hpp"
#include "owl/math/geometry/trafos.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {
      template <typename Scalar>
      class camera_intrinsics
      {
      public:
        using scalar = Scalar;
        using interval = interval<float>;

        camera_intrinsics() = default;

        camera_intrinsics(std::size_t w, std::size_t h, Scalar fx, Scalar fy, Scalar cx,
        Scalar cy, Scalar skew = 0, Scalar znear = Scalar{1}, Scalar zfar = Scalar{100})
        {
          set_intrinsics(w, h, fx, fy, cx, cy, skew, znear, zfar);
        }

        void set_intrinsics(std::size_t w, std::size_t h, Scalar fx, Scalar fy, Scalar cx,
          Scalar cy, Scalar skew = 0, Scalar znear = Scalar{1}, Scalar zfar = Scalar{100})
        {
          width = w; height = h;
          z_range ={znear, zfar};
          intrinsic_matrix_ << fx , skew, cx,
                                0, fy, cy,
                                0,  0,  1;
        }

        const std::pair<Scalar,Scalar> focal_lengths() const
        {
          return std::make_pair(fx(), fy());
        }

        math::vector<Scalar,2> principal_point() const
        {
          return {cx(), cy()};
        }

        Scalar& cx()
        {
          return intrinsic_matrix_(0, 2);
        }

        const Scalar& cx() const
        {
          return intrinsic_matrix_(0, 2);
        }

        Scalar& cy()
        {
          return intrinsic_matrix_(1, 2);
        }

        const Scalar& cy() const
        {
          return intrinsic_matrix_(1, 2);
        }

        Scalar& z_near()
        {
          return z_range.lower_bound;
        }

        const Scalar& z_near() const
        {
          return z_range.lower_bound;
        }

        Scalar& z_far()
        {
          return z_range.upper_bound;
        }

        const Scalar& z_far() const
        {
          return z_range.upper_bound;
        }

        Scalar& fx()
        {
          intrinsic_matrix_(0, 0);
        }

        const Scalar& fx() const
        {
          intrinsic_matrix_(0, 0);
        }

        Scalar& fy()
        {
          intrinsic_matrix_(1, 1);
        }

        const Scalar& fy() const
        {
          intrinsic_matrix_(1, 1);
        }

        Scalar& skew()
        {
          return intrinsic_matrix_(0, 1);
        }

        const Scalar& skew() const
        {
          return intrinsic_matrix_(0, 1);
        }

        const math::square_matrix<Scalar,3>& camera_matrix() const
        {
          return intrinsic_matrix_;
        };

        std::pair<std::size_t, std::size_t> resolution() const
        {
          return {width, height};
        };

        math::square_matrix<Scalar,4> projection_matrix() const
        {
          assert(skew() == 1);
          auto l = -z_near() * cx() / fx();
          auto r =  z_near()  * (width - cx()) / fx();
          auto b = -z_near() * cy() / fy();
          auto t =  z_near()  *(height - cy()) / fy();
          return frustrum(l, r, b, t, z_near(), z_far());
        };


        std::size_t width = 0;
        std::size_t height = 0;
        interval z_range = {1, 100};
        math::square_matrix<Scalar,3> intrinsic_matrix_;

      };

      template <typename Scalar>
      std::ostream& operator<<(std::ostream& out, const camera_intrinsics<Scalar>& cam)
      {
        return out << "resolution: "<<cam.width << " x " << cam.height << "\n" << "z_range: "<< cam.z_range << "\n"
                   << "K: " << cam.camera_matrix();
      }
    }
  }
}




