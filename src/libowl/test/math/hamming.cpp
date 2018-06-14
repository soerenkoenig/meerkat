//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#include <iostream>
#include "owl/math/hamming.hpp"
#include "catch/catch.hpp"

namespace test
{

  TEST_CASE( "hamming", "[math]" )
  {
    std::vector<float> window(64);
    owl::math::hamming<float>(window.begin(), window.end());

  }
}




