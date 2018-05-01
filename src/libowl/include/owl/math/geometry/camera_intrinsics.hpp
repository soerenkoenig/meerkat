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
    namespace geomemetry
    {
      template <typename Scalar>
      class camera_intrinsics
      {
      public:

        camera_intrinsics() = default;

        camera_intrinsics(std::size_t w, std::size_t h, Scalar fx, Scalar fy, Scalar cx,
        Scalar cy)
        {
          set_intrinsics(w, h, fx, fy, cx, cy);
        }

        void set_intrinsics(std::size_t w, std::size_t h, Scalar fx, Scalar fy, Scalar cx,
          Scalar cy)
        {
          width_ = w; height_ = h;
          intrinsic_matrix_ << fx , 0, cx,
                                0, fy, cy,
                                0,  0,  1;
        }

        const std::pair<Scalar,Scalar>  focal_length() const
        {
          return std::make_pair(intrinsic_matrix_(0, 0), intrinsic_matrix_(1, 1));
        }

        math::vector<Scalar,2> principal_point() const
        {
          return {intrinsic_matrix_(0, 2), intrinsic_matrix_(1, 2)};
        }

        Scalar& focal_length_x()
        {
          intrinsic_matrix_(0, 0);
        }

        const Scalar& focla_length_x() const
        {
          intrinsic_matrix_(0, 0);
        }

        Scalar& focal_length_y()
        {
          intrinsic_matrix_(1, 1);
        }

        const Scalar& focal_length_y() const
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

      private:
        std::size_t width_ = 0;
        std::size_t height_ = 0;
        math::square_matrix<Scalar,3> intrinsic_matrix_;
      };
    }
  }
}




