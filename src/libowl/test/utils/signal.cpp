#include "owl/utils/signal.hpp"
#include "catch/catch.hpp"


namespace test
{



  class a
  {
  public:
    a(owl::utils::signal<int (int, int)>& sig)
    {
      _connections += sig.connect(owl::utils::slot(this,&a::doit));
    }
    int doit(int x, int y)
    {
      if( x*y < 5)
        return 1;
      return 3;
    }

    owl::utils::connection_list _connections;
  };

  TEST_CASE( "combiner", "[utils]" )
  {
    using namespace owl::utils;
    {
      combiner_last<int> c;
      CHECK(c(3));
      CHECK(c.get() == 3);
      CHECK(c(4));
      CHECK(c.get() == 4);
    }

    {
      combiner_vector<int> c;
      CHECK(c(3));
      CHECK(c.get().size() == 1);
      CHECK(c.get()[0] == 3);
      CHECK(c(4));
      CHECK(c.get().size() == 2);
      CHECK(c.get()[1] == 4);
    }

    signal<int(int x, int y)> sig;
    {
      scoped_connection con = sig.connect([](int x, int y){ return x;});
      CHECK(sig(1,2) == 1);
      CHECK(sig(2,2) == 2);
      CHECK(sig.num_connections() == 1);
    }
    CHECK(sig.num_connections() == 0);
    {
      a i(sig);
      CHECK(sig(3,4) == 3);
    }

    CHECK(sig.num_connections() == 0);

    scoped_connection con;
    {
      signal<int(int x, int y)> sig;
      con = sig.connect([](int x, int y){ std::cout << "xy" << std::endl; return x;});
      CHECK(con.is_connected());
    }
    CHECK_FALSE(con.is_connected());

  }
}
