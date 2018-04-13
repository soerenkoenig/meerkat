#include "owl/math/physical_properties.hpp"
#include "owl/math/approx.hpp"
#include "catch/catch.hpp"

#include <iostream>

namespace test
{
  TEST_CASE( "physical_properties", "[math]" )
  {
    using namespace owl::math;
    physical_properties<float> prop;

    prop.add_quad(
      vector3f( -0.5, -0.5, -0.5),
      vector3f( -0.5,  0.5, -0.5),
      vector3f(  0.5,  0.5, -0.5),
      vector3f(  0.5, -0.5, -0.5));

    prop.add_quad(
      vector3f(  0.5, -0.5, 0.5 ),
      vector3f(  0.5,  0.5, 0.5 ),
      vector3f( -0.5,  0.5, 0.5 ),
      vector3f( -0.5, -0.5, 0.5 ));

    prop.add_quad(
      vector3f( 0.5, -0.5, -0.5 ),
      vector3f( 0.5,  0.5, -0.5 ),
      vector3f( 0.5,  0.5,  0.5 ),
      vector3f( 0.5, -0.5,  0.5 ));
    
    prop.add_quad(
      vector3f( -0.5, -0.5,  0.5 ),
      vector3f( -0.5,  0.5,  0.5 ),
      vector3f( -0.5,  0.5, -0.5 ),
      vector3f( -0.5, -0.5, -0.5 ));


    prop.add_quad(
      vector3f(  0.5,  0.5,  0.5 ),
      vector3f(  0.5,  0.5, -0.5 ),
      vector3f( -0.5,  0.5, -0.5 ),
      vector3f( -0.5,  0.5,  0.5 ));

    prop.add_quad(
      vector3f(  0.5, -0.5, -0.5 ),
      vector3f(  0.5, -0.5,  0.5 ),
      vector3f( -0.5, -0.5,  0.5 ),
      vector3f( -0.5, -0.5, -0.5 ));
    
    CHECK(approx(prop.volume()) == 1);
    CHECK(approx(prop.surface_area()) == 6);
    CHECK(approx(prop.centroid()) == vector3f::zero());

  }


}


