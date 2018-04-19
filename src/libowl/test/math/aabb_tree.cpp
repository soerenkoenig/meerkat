

#include "owl/math/mesh.hpp"
#include "owl/math/mesh_primitives.hpp"
#include "owl/math/point_utils.hpp"
#include "owl/math/aabb_tree.hpp"
#include "owl/math/mesh.hpp"
#include "catch/catch.hpp"


namespace test
{
  TEST_CASE( "aabb_tree", "[math]" )
  {
    using namespace owl::math;
    std::size_t n = 10000;
    std::vector<vector3f> points = random_points<float>(n);
    aabb_tree<vector3f> point_tree = make_aabb_tree(points);
    CHECK(point_tree.num_leaf_nodes() + point_tree.num_split_nodes() == point_tree.num_nodes());


    knn_searcher<vector3f> searcher(points);

    vector3f q(1,2,3);
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<std::size_t> dist(0, n-1);
    for(std::size_t i = 0; i < 10; ++i)
    {
      std::size_t j = dist(eng);
      auto neighbors = searcher.closest_k_primitives(5, points[j]);
      CHECK(std::find_if(neighbors.begin(), neighbors.end(), [&](const auto &r)
        { return r.primitive == points[j]; }) != neighbors.end());
    }

    mesh<float> m = create_icosaeder<float>();


    auto deref = [&](const vertex_handle& v){ return m.position(v);};

    knn_searcher<vertex_handle, decltype(deref)> searcher2(m.vertices(), deref );

    auto x = searcher2;

    auto closest_vertices = searcher2.closest_k_primitives(4,vector3f(10,0,0));

    for(const auto& v : closest_vertices)
      std::cout << v.primitive << " "<< v.distance() <<" "<< transpose(m.position(v.primitive)) << std::endl;


  }
}
