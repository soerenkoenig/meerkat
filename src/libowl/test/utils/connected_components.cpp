#include <iostream>

#include "owl/utils/connected_components.hpp"
#include "owl/math/approx.hpp"
#include "catch/catch.hpp"


namespace test
{

  TEST_CASE( "connected_components", "[math]" )
  {
    using namespace owl::utils;
    struct element_tag {};
    using element_handle = handle<element_tag>;

    connected_components<element_handle> comps(10);
    CHECK(comps.num_components() == 10);
    CHECK(comps.num_elements() == 10);
    auto e1 = comps.element(1);
    auto e2 = comps.element(6);
    CHECK_FALSE(comps.is_connected(e1,e2));
    auto c = comps.connect(e1, e2);
    CHECK(comps.is_connected(e1,e2));
    CHECK(comps.num_components() == 9);
    CHECK(comps.num_elements(c) == 2);
  }
}
