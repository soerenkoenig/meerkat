//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#include <cmath>
#include <algorithm>
#include "owl/math/constants.hpp"
namespace owl
{
  namespace math
  {
    template <typename Scalar>
    Scalar hz_2_mel(Scalar value_in_hz)
    {
      return 1127 * std::log(1 + value_in_hz / 700);
    }

    template <typename Scalar>
    Scalar mel_2_hz(Scalar value_in_mel)
    {
      return 700 * std::exp(value_in_mel / 1127) - 700;
    }


    /// Fill sequence [begin, end) with a sampled hamming window
    template <typename Scalar, typename OutIter>
    void hamming(OutIter begin, OutIter end)
    {
      auto n = std::distance(begin, end);
      for(std::size_t i = 0; i < n; ++i)
        *begin++ = 0.54 - 0.46 * std::cos(2 * constants::two_pi<Scalar> * i / (n-1) );
    }
  }
}
