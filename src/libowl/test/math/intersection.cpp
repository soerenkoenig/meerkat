
#include "owl/math/geometry/interval.hpp"
#include "owl/math/geometry/line_segment.hpp"
#include "owl/math/geometry/nplane.hpp"
#include "owl/math/geometry/triangle.hpp"
#include "owl/math/geometry/intersection.hpp"
#include "catch/catch.hpp"


namespace test
{
  TEST_CASE("intersection", "[math]")
  {
    using namespace owl::math;
    using namespace owl::math::geometry;

    CHECK(closest_intersection_point(rectanglef({0,-1}, {1,2}), ray2f({-2,0}, {1,0})) == vector2f(0,0));

    CHECK(closest_intersection_point(triangle3f({0,0,0},{1,0,0},{0,1,0}), ray_segment3f({10,10,10},{-1,-1,-1})) == vector3f{0,0,0});
  }
}

