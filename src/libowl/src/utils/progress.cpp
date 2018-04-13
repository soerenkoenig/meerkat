#include "owl/utils/progress.hpp"

namespace owl
{
  namespace utils
  {
  
    thread_local std::stack<progress::current_info> progress::current_progress_ = {};
  }
}
