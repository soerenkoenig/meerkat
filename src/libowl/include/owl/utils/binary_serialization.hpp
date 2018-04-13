//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2017 Sören König. All rights reserved.
//

#pragma once

#include <iostream>
#include <tuple>
#include <type_traits>
#include "owl/utils/container_utils.hpp"

namespace owl
{
  namespace utils
  {
    template <typename T, typename = void>
    struct binary_serialization
    {
      constexpr static bool is_serializable = false;
    };
    
    template <typename T>
    bool write_binary(std::ostream& out, const T& value)
    {
      static_assert(binary_serialization<T>::is_serializable,
                    "binary serialization not supported for this type");
      return binary_serialization<T>::write(out, value);
    }
    
    template <typename T>
    bool read_binary(std::istream& in, T& value)
    {
      static_assert(binary_serialization<T>::is_serializable,
                   "binary serialization not supported for this type");
      return binary_serialization<T>::read(in, value);
    }
    
    template <typename T>
    bool read_binary(std::istream& in, T&& value)
    {
      static_assert(binary_serialization<T>::is_serializable,
                    "binary serialization not supported for this type");
      return binary_serialization<T>::read(in, value);
    }
    
    namespace detail
    {
      DEFINE_HAS_SIGNATURE(has_size_from_type, T::size, std::size_t (*)());
      DEFINE_HAS_SIGNATURE(has_size_from_instance_lref, T::size, std::size_t (*)(const typename T::value_type&));
      DEFINE_HAS_SIGNATURE(has_size_from_instance_rref, T::size, std::size_t (*)(typename T::value_type&&));
      template <typename T>
      using has_size_from_instance = std::conditional_t<has_size_from_instance_lref<T>::value || has_size_from_instance_rref<T>::value,std::true_type, std::false_type>;
    }
    
    template <typename T>
    std::size_t size_binary(T&& value)
    {
      static_assert(detail::has_size_from_instance<T>::value);
      return binary_serialization<T>::size(std::forward<T>(value));
    }
    
    template <typename T>
    constexpr std::size_t size_binary()
    {
      static_assert(detail::has_size_from_type<T>::value);
      return binary_serialization<T>::size();
    }
    
    
    namespace detail
    {
      DEFINE_HAS_SIGNATURE(has_write_binary, T::write_binary, bool (T::*)(std::ostream&) const);
      DEFINE_HAS_SIGNATURE(has_read_binary, T::read_binary, bool (T::*)(std::istream&));
    }
    
    template <typename T>
    struct binary_serialization<T, std::enable_if_t<detail::has_write_binary<T>::value && detail::has_read_binary<T>::value>>
    {
      using value_type = T;
      
      constexpr static bool is_serializable = true;
    
      static bool read(std::istream& in, T& value)
      {
        return value.read_binary(in);
      }
    
      static bool write(std::ostream& out, const T& value)
      {
        return value.write_binary(out);
      }
    };
    
    template <typename T>
    struct binary_serialization<T, std::enable_if_t<std::is_arithmetic<T>::value && !std::is_same<bool, T>::value>>
    {
      using value_type = T;
    
      constexpr static bool is_serializable = true;
      
      constexpr static std::size_t size()
      {
        return sizeof(value_type);
      }
    
      static bool read(std::istream& in, value_type& value)
      {
        in.read(reinterpret_cast<char*>(&value), size());
        return in.good();
      }
    
      static bool write(std::ostream& out, const value_type& value)
      {
        out.write(reinterpret_cast<const char*>(&value), size());
        return out.good();
      }
    };
    
    template <>
    struct binary_serialization<bool, void>
    {
      constexpr static bool is_serializable = true;
      
      constexpr static std::size_t size()
      {
        return 1;
      }
    
      static bool read(std::istream& in, bool& value)
      {
        std::uint8_t temp;
        if(!read_binary(in,temp))
          return false;
        value = temp == 0 ? false : true;
        return true;
      }
    
      static bool write(std::ostream& out, const bool& value)
      {
        return write_binary(out, value == true ? std::uint8_t{1} : std::uint8_t{0});
      }
    };
    
    template <typename T>
    struct binary_serialization<T, std::enable_if_t<std::is_enum<T>::value>>
    {
      constexpr static bool is_serializable = true;
    
      static bool read(std::istream& in, T& value)
      {
        return read_binary(in, static_cast<std::underlying_type_t<T>&>(value));
      }
    
      static bool write(std::ostream& out, const T& value)
      {
        return write_binary(out, static_cast<const std::underlying_type_t<T>&>(value));
      }
    };
    
    template <typename T1, typename T2>
    struct binary_serialization<std::pair<T1, T2>, void>
    {
      using value_type = std::pair<T1, T2>;
    
      constexpr static bool is_serializable = binary_serialization<T1>::is_serializable && binary_serialization<T2>::is_serializable;
    
      static bool read(std::istream& in, value_type& value)
      {
        if (!read_binary(in, value.first) || !read_binary(in, value.second))
          return false;

        return true;
      }
    
      static bool write(std::ostream& out, const value_type& value)
      {
        return write_binary(out, value.first) && write_binary(out, value.second);
      }
    };
    
    namespace detail
    {
      template <typename T>
      struct elem_decay
      {
        using type = T;
      };
      
      template <typename... Args>
      struct elem_decay<std::tuple<Args...>>
      {
        using type = std::tuple<std::decay_t<Args>...>;
      };
      
      template <typename T1, typename T2>
      struct elem_decay<std::pair<T1, T2>>
      {
        using type = std::pair<std::decay_t<T1>, std::decay_t<T2>>;
      };
    }
    
    template <typename... Args>
    struct binary_serialization<std::tuple<Args...>, void>
    {
      using value_type = std::tuple<Args...>;
    
      constexpr static bool is_serializable = (... && binary_serialization<std::decay_t<Args>>::is_serializable);
    
      static bool read(std::istream& in, value_type& value)
      {
        return read_impl(in,value, std::index_sequence_for<Args...>());
      }
    
      static bool write(std::ostream& out, const value_type& value)
      {
        return write_impl(out, value, std::index_sequence_for<Args...>());
      }
    private:
      template <std::size_t... Is>
      static bool read_impl(std::istream& in, value_type& value, std::index_sequence<Is...>)
      {
        return (... && read_binary(in, std::get<Is>(value)));
      }
    
      template <std::size_t... Is>
      static bool write_impl(std::ostream& out, const value_type& value, std::index_sequence<Is...>)
      {
        return (... && write_binary(out, std::get<Is>(value)));
      }
    };
    
    
    template <typename Container>
    struct binary_serialization<Container, std::enable_if_t<is_associative_container<Container>::value>>
    {
      using value_type = Container;
      using elem_type = typename detail::elem_decay<typename Container::value_type>::type;
    
      constexpr static bool is_serializable = binary_serialization<elem_type>::is_serializable;
    
      static bool read(std::istream& in, value_type& value)
      {
        value.clear();
      
        std::int64_t count;
        if (!read_binary(in, count))
          return false;
      
        for (std::size_t i = 0; i < static_cast<std::size_t>(count); ++i)
        {
          elem_type elem;
          if (!read_binary(in, elem))
            return false;
          value.insert(elem);
        }
        return true;
      }
    
      static bool write(std::ostream& out, const value_type& value)
      {
        std::int64_t count = static_cast<std::int64_t>(value.size());
        if (!write_binary(out, count))
         return false;
      
        for (auto&& elem : value)
        {
          if (!write_binary(out, elem))
            return false;
        }
        return true;
      }
    };
    
    template <typename T, std::size_t N>
    struct binary_serialization<std::array<T, N>, void>
    {
      using value_type = std::array<T, N>;
      
      constexpr static bool is_serializable = binary_serialization<T>::is_serializable;
      
      static bool read(std::istream& in, value_type& value)
      {
        for (auto&& elem : value)
        {
          if (!read_binary(in, elem)) return false;
        }
        return true;
      }
      
      static bool write(std::ostream& out, const value_type& value)
      {
        for (auto&& elem : value)
        {
          if (!write_binary(out, elem)) return false;
        }
        return true;
      }
    };
    
    template <typename T, std::size_t N>
    struct binary_serialization<T[N], void>
    {
      using value_type = T[N];
    
      constexpr static bool is_serializable = binary_serialization<T>::is_serializable;
    
      static bool read(std::istream& in, value_type& value)
      {
        for (auto&& elem : value)
        {
          if (!read_binary(in, elem)) return false;
        }
        return true;
      }
    
      static bool write(std::ostream& out, const value_type& value)
      {
        for (auto&& elem : value)
        {
          if (!write_binary(out, elem)) return false;
        }
        return true;
      }
    };
    
    template <typename Container>
    struct binary_serialization<Container, std::enable_if_t<is_sequence_container<Container>::value
      && !is_array<Container>::value>>
    {
      using value_type = Container;
      using elem_type =
      std::decay_t<decltype(*std::begin(std::declval<Container>()))>;
      
      constexpr static bool is_serializable =
      binary_serialization<elem_type>::is_serializable;
      
      static bool read(std::istream& in, value_type& value)
      {
        value.clear();
        
        std::int64_t count;
        if (!read_binary(in, count)) return false;
        
        value.resize(static_cast<std::size_t>(count));
        
        for (auto&& elem : value)
        {
          if (!read_binary(in, elem)) return false;
        }
        return true;
      }
      
      static bool write(std::ostream& out, const value_type& value)
      {
        std::int64_t count = static_cast<std::int64_t>(value.size());
        if (!write_binary(out, count)) return false;
    
        for (auto&& elem : value)
        {
          if (!write_binary(out, elem)) return false;
        }
        return true;
      }
    };
    
    template <>
    struct binary_serialization<std::vector<bool>, void>
    {
      constexpr static bool is_serializable = true;
    
      static bool read(std::istream& in, std::vector<bool>& value)
      {
        value.clear();

        std::int64_t count;
        if (!read_binary(in, count)) return false;

        value.resize(static_cast<std::size_t>(count));

        for (std::vector<bool>::reference elem : value)
        {
          bool e;
          if (!read_binary(in, e)) return false;
          elem = e;
        }
        return true;
      }
    
      static bool write(std::ostream& out, const std::vector<bool>& value)
      {
        std::int64_t count = static_cast<std::int64_t>(value.size());
        if (!write_binary(out, count))
          return false;

        for (bool elem : value)
        {
          if (!write_binary(out, elem))
            return false;
        }
        return true;
      }
    };
  }
}
