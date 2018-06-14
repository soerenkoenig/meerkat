
#include <iostream>
#include "owl/math/statistics.hpp"
#include "catch/catch.hpp"


namespace test
{
  TEST_CASE("statistics", "[math]")
  {
    using namespace owl::math;
    std::vector<int> values ={1,2,3};
    CHECK( mean<float>(values) == 2);



  };
}
