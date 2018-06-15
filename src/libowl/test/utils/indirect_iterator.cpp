#include <type_traits>
#include <vector>
#include <iostream>
#include "owl/utils/indirect_iterator.hpp"
#include "catch/catch.hpp"

namespace test
{

  TEST_CASE( "indirect_iterator", "[utils]" )
  {
    using namespace owl::utils;
    std::vector<int*> numbers;
    std::generate_n(std::back_inserter(numbers),10,
                    [i = 0]() mutable { return new int(i++); });

    indirect_iterator<std::vector<int*>::iterator> first = numbers.begin();
    indirect_iterator<std::vector<int*>::iterator> last = numbers.end();

    auto range = make_indirect_range(numbers);
    


  }
}

