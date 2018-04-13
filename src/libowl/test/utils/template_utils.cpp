#include <sstream>

#include "owl/utils/binary_serialization.hpp"
#include "catch/catch.hpp"

namespace test
{
  TEST_CASE("is_one_of", "[utils]" )
  {
    CHECK((owl::utils::is_one_of<int, float, double, int>()));
    CHECK_FALSE((owl::utils::is_one_of<int, float, double>()));
  }
}
