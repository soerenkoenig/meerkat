//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include <type_traits>
#include <iterator>
#include "owl/utils/iterator_range.hpp"
#include "owl/utils/template_utils.hpp"

namespace owl
{
  namespace utils
  {
    struct use_default
    {
    };

    template<typename Iterator, typename Value = use_default, typename Category = use_default, typename Reference = use_default, typename Difference = use_default>
    class indirect_iterator
    {
      using traits = std::iterator_traits<Iterator>;
    public:

      using value_type = std::conditional_t<std::is_same_v<Value, use_default>,
        std::remove_cv_t<typename pointee<typename traits::value_type>::type>,
        std::remove_cv_t<Value>>;

      using reference = std::conditional_t<std::is_same_v<Reference, use_default>,
        decltype(*std::declval<Iterator>()), Reference>;

      using pointer = std::conditional_t<std::is_same_v<Value, use_default>,
        value_type *, Value *>;

      using difference_type = std::conditional_t<std::is_same_v<Difference, use_default>,
        typename traits::difference_type, Difference>;

      using iterator_category = std::conditional_t<std::is_same_v<Category, use_default>,
        typename traits::iterator_category, Category>;


      indirect_iterator()
      {}

      indirect_iterator(Iterator x)
        : iterator_{x}
      {
      }

      template<class Iterator2, class Value2, class Category2, class Reference2, class Difference2>
      indirect_iterator(indirect_iterator<Iterator2, Value2, Category2, Reference2, Difference2> const &y,
        enable_if_convertible_t<Iterator2, Iterator> * = 0)
        : iterator_{y.iterator_}
      {}

      Iterator const &base() const
      {
        return iterator_;
      }

      reference operator*() const
      {
        return **iterator_;
      }

      indirect_iterator &operator++()
      {
        ++iterator_;
        return *this;
      }

      indirect_iterator &operator--()
      {
        --iterator_;
        return *this;
      }

    private:
      Iterator iterator_;
    };

    template<typename Range>
    auto make_indirect_range(Range &&range)
    {
      return iterator_range<indirect_iterator<decltype(std::begin(range))>>(std::begin(range), std::end(range));
    }
  }

}

