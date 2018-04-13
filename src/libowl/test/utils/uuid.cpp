#include "owl/utils/uuid.hpp"
#include "catch/catch.hpp"

namespace test
{
  TEST_CASE( "uuid", "[utils]" )
  {
    using namespace owl::utils;
    uuid uid1 = random_uuid();
    uuid uid2 = random_uuid();
    uuid zero = uuid::zero();
    CHECK(to_string(zero) == "00000000-0000-0000-0000-000000000000");
    CHECK((uid1 != uid2));
    CHECK(uid1.variant() == uuid::variant_rfc_4122);
    CHECK(uid1.version() == uuid::version_random_number_based);
    std::string guid2 = "a8e6ada1-a2b4-4418-8ffe-1fa20a3ca05a";
    std::string guid = "{a8e6ada1-a2b4-4418-8ffe-1fa20a3ca05a}";
    std::string guid4 = "(a8e6ada1-a2b4-4418-8ffe-1fa20a3ca05a)";
    uuid u3 = parse_uuid(guid);
    CHECK(to_string(u3) == guid2);
    std::stringstream ss;
    ss << guid2;
    uuid uid4;
    ss >> uid4;
    CHECK(to_string(uid4) == guid2);

    uuid u4 = parse_uuid(guid4);
    CHECK(to_string(u4) == guid2);
  }
}

