
//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2017 Sören König. All rights reserved.
//

#pragma once

#include <vector>
#include <string>

#include "owl/export.hpp"
#include "owl/utils/container_utils.hpp"
#include "owl/utils/buffer.hpp"

namespace owl
{
  namespace utils
  {
  
    OWL_API std::string base64_encode(const buffer& buf);
  
    OWL_API buffer base64_decode(const std::string& encoded_string);
  }
}
