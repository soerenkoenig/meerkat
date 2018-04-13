#include <type_traits>

#include "owl/utils/container_utils.hpp"
#include "catch/catch.hpp"

namespace test
{

  TEST_CASE( "is_range", "[utils]" )
  {
    using namespace owl::utils;
    CHECK(is_container<std::vector<int>>::value);

    CHECK(is_container<double[5]>::value);

    CHECK_FALSE(is_container<int>::value);
  }

  TEST_CASE( "container_traits", "[]" )
  {
    using namespace owl::utils;

    CHECK((std::is_same<container_traits<std::vector<int>>::value_type, int>::value));
    CHECK((std::is_same<container_traits<std::vector<int>>::size_type, std::vector<int>::size_type>::value));

    CHECK((std::is_same<container_traits<int[4]>::value_type, int>::value));
    CHECK((std::is_same<container_traits<int[4]>::size_type, std::size_t>::value));

  }
}
