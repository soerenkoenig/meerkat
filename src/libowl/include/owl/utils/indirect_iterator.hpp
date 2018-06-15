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
        std::add_lvalue_reference_t<value_type>, Reference>;

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

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category, std::bidirectional_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      indirect_iterator &operator--()
      {
        --iterator_;
        return *this;
      }

      indirect_iterator operator++(int)
      {
        auto tmp = *this;
        operator++();
        return tmp;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category, std::bidirectional_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      indirect_iterator operator--(int)
      {
        auto tmp = *this;
        operator--();
        return tmp;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category, std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      indirect_iterator& operator+=(difference_type n)
      {
        std::advance(iterator_, n);
        return *this;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category,std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      indirect_iterator& operator-=(difference_type n)
      {
        std::advance(iterator_, - n);
        return *this;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category,std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      indirect_iterator operator+(difference_type n) const
      {
        auto it = *this;
        it += n;
        return it;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category,std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      indirect_iterator operator-(difference_type n) const
      {
        auto it = *this;
        it -= n;
        return it;
      }

      bool operator==(const indirect_iterator& other) const
      {
        return iterator_ == other.iterator_;
      }

      bool operator!=(const indirect_iterator& other) const
      {
        return iterator_ != other.iterator_;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category,std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      bool operator<(const indirect_iterator& other) const
      {
        return iterator_ < other.iterator_;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category,std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      bool operator<=(const indirect_iterator& other) const
      {
        return iterator_ <= other.iterator_;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category,std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      bool operator>(const indirect_iterator& other) const
      {
        return iterator_ > other.iterator_;
      }

      template<bool Cond = std::is_base_of_v<indirect_iterator::iterator_category,std::random_access_iterator_tag>,
        std::enable_if_t<Cond, int> = 0>
      bool operator>=(const indirect_iterator& other) const
      {
        return iterator_ >= other.iterator_;
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

