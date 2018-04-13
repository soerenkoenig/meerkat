#include <sstream>

#include "owl/utils/binary_serialization.hpp"
#include "catch/catch.hpp"

namespace test
{
  struct person
  {
    std::string name;
    int age;

    bool write_binary(std::ostream& out) const
    {
      return owl::utils::write_binary(out, std::tie(name, age));
    }

    bool read_binary(std::istream& in)
    {
      return owl::utils::read_binary(in, std::tie(name, age));
    }

    bool operator==(const person& other) const
    {
      return std::tie(name, age) == std::tie(other.name, other.age);
    }
  };

  TEST_CASE( "binary_serialization", "[utils]" )
  {
    std::stringstream  str;
    using namespace owl::utils;
    std::set<int> name1 = {1, 2, 3, 4, 5, 6, 7, 8};
    std::string name = u8"hello world";

    CHECK(write_binary(str, name1));
    CHECK(write_binary(str, name));

    std::set<int> data2;
    CHECK(read_binary(str, data2));
    CHECK(name1 == data2);
    std::string name2;
    CHECK(read_binary(str, name2));
    CHECK(name == name2);

    std::map<int, double> datam = {{1, 1.3}, {6, 8.6}};
    CHECK(write_binary(str, datam));
    std::map<int, double> datam2;
    CHECK(read_binary(str, datam2));
    CHECK(datam == datam2);

    std::vector<bool> vecb = {true, false, true};
    std::vector<bool> vecb2;
    CHECK(write_binary(str, vecb));
    CHECK(read_binary(str, vecb2));
    CHECK(vecb == vecb2);

    double pnt[5] ={1.0, 2.0, 3.0, 4.0, 5.0};
    double pnt2[5];
    CHECK(write_binary(str, pnt));
    CHECK(read_binary(str, pnt2));
    CHECK((std::equal(std::begin(pnt), std::end(pnt), std::begin(pnt2))));

    auto t = std::make_tuple(1, 2, 3);
    decltype(t) t2;
    CHECK(write_binary(str, t));
    CHECK(read_binary(str, t2));
    CHECK(t == t2);

    person p1{"peter", 5};
    person p2;
    CHECK(write_binary(str, p1));
    CHECK(read_binary(str, p2));
    CHECK(p1 == p2);
  }
}


