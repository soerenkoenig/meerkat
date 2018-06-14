
#include "owl/math/geometry/rigid_transform.hpp"
#include "catch/catch.hpp"


namespace test
{
  TEST_CASE("transformation", "[math]")
  {
    using namespace owl::math;
    using namespace owl::math::geometry;

    rigid_transform<float> trafo1;
    CHECK(trafo1.matrix() == matrix44f::identity());


    trafo1.rotation = axis_angle<float>{vector3f(1, 0, 0), degrees<float>(20)};
    trafo1.translation = {1, 2, 3};

    rigid_transform<float> trafo2;


    trafo2.rotation = axis_angle<float>{vector3f(0, 1, 0), degrees<float>(-10)};
    trafo2.translation = {-1, 3, 2};

    std::cout << trafo1.matrix() <<std::endl;

    CHECK(approx((trafo1 * trafo2).matrix()).margin(0.0001) == trafo1.matrix() * trafo2.matrix() );

  };
}

