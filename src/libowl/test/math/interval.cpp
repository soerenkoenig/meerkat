#include "owl/math/geometry/interval.hpp"
#include "owl/math/approx.hpp"
#include "catch/catch.hpp"

namespace test
{
  TEST_CASE( "interval", "[math]" )
  {
    using namespace owl::math;
    using namespace owl::math::geometry;

    box<float> b;
    b.insert(point3f(-1.0f,-1.0f,-1.0f));
    b.insert(point3f(1,1,1));
    auto x = b.extents().max_element_index();
    CHECK(b.inside(point3f(0,0,0)));
    interval<int,1,false,true> i1(0,1);
    interval<int,1,false,true> i2(1,2);
    auto i3 = i1.intersect(i2);

    interval<int,1,false,false> i4(0,1);
    interval<int,1,false,false> i5(1,2);
    auto i6 = i4.intersect(i5);
    CHECK(!i6.empty());



    interval<float> range;

  }
}
