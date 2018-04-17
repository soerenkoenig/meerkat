

#include "owl/math/mesh.hpp"
#include "owl/math/point_utils.hpp"
#include "owl/math/aabb_tree.hpp"
#include "owl/math/mesh.hpp"
#include "catch/catch.hpp"

#include <map>

namespace test
{
  TEST_CASE( "aabb_tree", "[math]" )
  {
    using namespace owl::math;

    std::vector<vector3f> points = random_points<float>(10000000);
    aabb_tree<vector3f> point_tree = make_aabb_tree(points);
    CHECK(point_tree.num_leaf_nodes() + point_tree.num_split_nodes() == point_tree.num_nodes());


    //mesh<float> m;

    //aabb_tree<face_handle,>

    std::map<int,int>::value_compare m;


  }
}
