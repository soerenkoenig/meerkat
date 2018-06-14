#include "owl/math/geometry/mesh.hpp"
#include "owl/math/geometry/mesh_primitives.hpp"
#include "owl/math/geometry/point_utils.hpp"
#include "owl/math/geometry/triangle.hpp"
#include "owl/math/geometry/aabb_tree.hpp"
#include "catch/catch.hpp"


namespace test
{
  /*
  TEST_CASE( "aabb_tree", "[math]" )
  {
    using namespace owl::math;
    using namespace owl::math::geometry;
    std::size_t n = 100000;
    std::vector<point3f> points = random_points<float>(n);
    aabb_tree<point3f> point_tree = make_aabb_tree(points);
    CHECK(point_tree.num_leaf_nodes() + point_tree.num_split_nodes() == point_tree.num_nodes());

    knn_searcher<point3f> searcher(points);

    vector3f q(1, 2, 3);
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<std::size_t> dist(0, n - 1);
    for (std::size_t i = 0; i < 10; ++i)
    {
      std::size_t j = dist(eng);
      auto neighbors = searcher.closest_k_primitives(5, points[j]);
      CHECK(std::find_if(neighbors.begin(), neighbors.end(), [&](const auto &r)
      { return r.primitive == points[j]; }) != neighbors.end());
    }

    mesh<float> m = create_icosaeder<float>();

    auto deref = [&](const vertex_handle &v)
    { return m.position(v); };

    knn_searcher<vertex_handle, decltype(deref)> searcher2(m.vertices(), deref);

    auto x = searcher2;
    auto ans = x.closest_primitive(vector3f(0, 0, 0));
    CHECK(ans != std::nullopt);

    auto closest_vertices = searcher2.closest_k_primitives(4, vector3f(10, 0, 0));

    for (const auto &v : closest_vertices)
      std::cout << v.primitive << " " << v.distance() << " " << m.position(v.primitive) << std::endl;

    auto result = searcher2.query_ball(vector3f(1, 0, 0), 1);


    std::vector<point2f> points2 = {{1, 2}, {4, 5}};

    knn_searcher<point2f> searcher3(points2);

    //search closest point in points2
    auto result3 = searcher3.closest_primitive({1, 3});
    CHECK(result3->primitive == point2f{1, 2});
    CHECK(result3->distance() == 1);

    std::vector<std::size_t> indices = {0, 1};
    auto index_2_point = [&](std::size_t i) { return points2[i]; };
    knn_searcher<std::size_t, decltype(index_2_point)> searcher4(indices, index_2_point);

    //search index of closest point in indices
    auto result4 = searcher4.closest_primitive({1, 3});
    CHECK(result4->primitive == 0);
    CHECK(result4->distance() == 1);

    //search closest line_segment
    std::vector<line_segment3f> line_segments = {line_segment3f({0, 0, 0}, {1, 1, 1}), line_segment3f({1, 1, 1}, {2, 2, 2})};
    knn_searcher<line_segment3f> searcher5(line_segments);

    auto result5 = searcher5.closest_primitive({-1, 0, 0});
    CHECK(result5->distance() == 1);


    std::vector<triangle3f> triangles = {triangle3f({0, 0, 0}, {1, 0, 0}, {0.5, 1, 0}),
                                         triangle3f({0, 0 ,0}, {0.5, 1, 0}, {1, 1, 0})};

    knn_searcher<triangle3f> searcher6(triangles);
    auto result6 = searcher6.closest_primitive({1,1,0});

    CHECK(result6->primitive == triangles[1]);



  }
   */
}
