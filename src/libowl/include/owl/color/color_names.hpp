#pragma once

#include <vector>
#include "owl/export.hpp"
#include "owl/optional.hpp"
#include "owl/color/color.hpp"
#include "owl/color/color_conversion.hpp"

namespace owl
{
  namespace color
  {
    struct OWL_API color_names
    {
      static const std::vector<std::string>& names();
      static const std::vector<rgb8u>& values();

      template<typename Channel>
      static std::optional<rgb<Channel>> lookup(const std::string& name)
      {
        auto it = std::lower_bound(names().begin(), names().end(), name);
        if(*it != name)
          return std::nullopt;
        return convert<rgb<Channel> >(*std::next(values().begin(), std::distance(names().begin(), it)));
      }
    };
  }
}
