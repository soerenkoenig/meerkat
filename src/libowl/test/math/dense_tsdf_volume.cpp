//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#include "owl/math/geometry/dense_tsdf_volume.hpp"
#include "catch/catch.hpp"


namespace test
{
  TEST_CASE("dense_tsdf_volume", "[math]")
  {
    using namespace owl::math;
    using namespace owl::math::geometry;

    dense_tsdf_volume<float> volume(1, 12, 0.1f, false);
    std::cout << volume.normal({0.5, 0.5, 0.5}) << std::endl;
  }
}
