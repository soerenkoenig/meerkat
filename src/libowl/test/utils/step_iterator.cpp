#include <type_traits>
#include <list>
#include "owl/utils/step_iterator.hpp"
#include "catch/catch.hpp"

namespace test
{
  TEST_CASE( "is_random_access_iterator", "[utils]" )
  {
    using namespace owl::utils;
    CHECK(is_random_access_iterator<std::vector<int>::iterator>::value);
    CHECK((is_random_access_iterator<step_iterator<std::vector<int>::iterator,2>>::value));

    CHECK_FALSE(is_random_access_iterator<std::list<int>::iterator>::value);
  }

  TEST_CASE( "step_iterator", "[utils]" )
  {
    using namespace owl::utils;
    std::vector<int> numbers = {0, 1, 2, 3, 4, 5, 6, 7};

    auto it = make_step_iterator<3>(numbers.begin());
    CHECK(it == numbers.begin());
    CHECK(++it == std::next(numbers.begin(),3));
    CHECK(*it == 3);
    CHECK(--it == numbers.begin());
  }
}

