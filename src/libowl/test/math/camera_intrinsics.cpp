//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#include <iostream>

#include "owl/math/geometry/camera_intrinsics.hpp"
#include "catch/catch.hpp"


namespace test
{
  TEST_CASE("camera intrinsics", "[math]")
  {
    using namespace owl::math;
    using namespace owl::math::geometry;

    camera_intrinsics<float> cam;

    std::cout << cam << std::endl;
  }
}
