//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include <numeric>

namespace owl
{
  namespace math
  {

    template <typename Scalar, typename ValueRange>
    auto mean(ValueRange&& values)
    {
      std::decay_t<decltype(*std::begin(values))> zero{};
      auto sum = std::accumulate(std::begin(values),std::end(values),zero);
      return sum / Scalar(std::size(values));
    }
  }
}
