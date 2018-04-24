#include "owl/math/geometry/ray.hpp"
#include "owl/math/approx.hpp"
#include "catch/catch.hpp"

namespace test
{
  TEST_CASE( "ray", "[math]" )
  {
    using namespace owl::math;
    using namespace owl::math::geometry;

    ray3f r1(vector3f(0,0,0),vector3f(1,1,1));
    CHECK(r1(2) == vector3f(2,2,2));
  }
}