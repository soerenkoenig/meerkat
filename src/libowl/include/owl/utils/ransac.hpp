//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2017 Sören König. All rights reserved.
//

#pragma once

#include "owl/export.hpp"

namespace owl
{
  namespace math
  {
    OWL_API std::size_t num_ransac_iterations(std::size_t n_min, const double p_out, const double p_surety = 0.99)
    {
      std::size_t iter = (std::size_t) ::ceil(log(1.0 - p_surety) /
        log(1.0 - pow(1.0 - p_out,static_cast<double>(n_min)));
      if (iter == 0)
        iter = 1;
      return iter;
    }
  }
}

