
#include "owl/math/geometry/interval.hpp"
#include "owl/math/geometry/line_segment.hpp"
#include "owl/math/geometry/nplane.hpp"
#include "owl/math/geometry/distance.hpp"
#include "catch/catch.hpp"


namespace test
{
  TEST_CASE("distance", "[math]")
  {

    using namespace owl::math;
    using namespace owl::math::geometry;

    CHECK(distance(rectanglef({0,0}, {1,2}), vector2f(-1,0)) == 1);
    CHECK(distance(boxd({-1,-1,-1}, {1,1,1}), vector3d(0,0,0)) == 0);
  }
}