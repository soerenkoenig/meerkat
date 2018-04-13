#include "owl/utils/base64.hpp"
#include "catch/catch.hpp"

namespace test
{

  TEST_CASE( "base64", "[utils]" )
  {
    using namespace owl::utils;

    buffer bytes = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    std::string encoded_string = base64_encode(bytes);
    buffer b = base64_decode(encoded_string);
    CHECK(base64_decode(encoded_string) == bytes);
  }
}



