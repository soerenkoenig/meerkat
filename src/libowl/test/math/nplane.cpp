#include "owl/math/geometry/nplane.hpp"
#include "owl/math/approx.hpp"
#include "catch/catch.hpp"

namespace test
{
  TEST_CASE( "plane", "[math]" )
  {
    using namespace owl::math;
    using namespace owl::math::geometry;

    ray3f r1(point3f(0.0f, 0.0f, 0.0f), vector3f(1, 1, 1));
    auto  pl = nplane_from_point_and_normal(point3f(0.0f, 1.0f, 0.0f), vector3f(1, 0, 0));
   // CHECK(distance(pl, r1(*intersect(r1, pl))) == 0);
  }
}

