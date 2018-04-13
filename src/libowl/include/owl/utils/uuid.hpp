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
#include <array>
#include <cstdint>
#include <random>
#include <limits>
#include <algorithm>

#include "owl/export.hpp"
#include "owl/utils/hash_utils.hpp"

namespace owl
{
  namespace utils
  {
    struct OWL_API uuid
    {
    public:
      using container_type = std::array<std::uint8_t, 16>;
      using value_type = container_type::value_type;
      using reference = container_type::reference;
      using const_reference = container_type::const_reference;
      using iterator = container_type::iterator;
      using const_iterator = container_type::const_iterator;
      using size_type = container_type::size_type;
      using difference_type = container_type::difference_type;
      
      iterator begin() noexcept;
      const_iterator begin() const noexcept;
      iterator end() noexcept;
      const_iterator end() const noexcept;
      
      constexpr size_type size() noexcept
      {
        return 16;
      }
    
      bool operator==(uuid const& other) noexcept;
      
      bool operator!=( uuid const& other) noexcept;
      
      bool operator<( uuid const& other) noexcept;
      
      bool operator>(uuid const& other) noexcept;
      
      bool operator<=(uuid const& other) noexcept;
      
      bool operator>=(uuid const& other) noexcept;
      
      enum variant_type
      {
        variant_ncs, // NCS backward compatibility
        variant_rfc_4122, // defined in RFC 4122 document
        variant_microsoft, // Microsoft Corporation backward compatibility
        variant_future // future definition
      };
      
      variant_type variant() const noexcept;
      
      enum version_type
      {
        version_unknown = -1,
        version_time_based = 1,
        version_dce_security = 2,
        version_name_based_md5 = 3,
        version_random_number_based = 4,
        version_name_based_sha1 = 5
      };
      
      version_type version() const noexcept;
    
      static constexpr uuid zero() { return {}; };
    
      std::array<std::uint8_t,16> data = {};
    };
    
    
    template <typename ch, typename char_traits>
    std::basic_ostream<ch, char_traits>& operator<<(std::basic_ostream<ch, char_traits> &os, const uuid& u)
    {
      auto flags = os.flags();
      auto fillstate = os.fill();
      
      const typename std::basic_ostream<ch, char_traits>::sentry ok(os);
      if (ok)
      {
        const std::streamsize width = os.width(0);
        const std::streamsize uuid_width = 36;
        const std::ios_base::fmtflags flags = os.flags();
        const typename std::basic_ios<ch, char_traits>::char_type fill = os.fill();
        if (flags & (std::ios_base::right | std::ios_base::internal))
        {
          for (std::streamsize i = uuid_width; i<width; i++)
            os << fill;
        }
        
        os << std::hex;
        os.fill(os.widen('0'));
        
        std::size_t i=0;
        for (uuid::const_iterator i_data = u.begin(); i_data!=u.end(); ++i_data, ++i)
        {
          os.width(2);
          os << static_cast<unsigned int>(*i_data);
          if (i == 3 || i == 5 || i == 7 || i == 9)
            os << os.widen('-');
        }
        
        if (flags & std::ios_base::left)
        {
          for (std::streamsize i=uuid_width; i<width; i++)
            os << fill;
        }
        
        os.width(0); //used the width so reset it
      }
      os.flags(flags);
      os.fill(fillstate);
      return os;
    }
    
    template <typename ch, typename char_traits>
    std::basic_istream<ch, char_traits>& operator>>(std::basic_istream<ch, char_traits> &is, uuid &u)
    {
      const typename std::basic_istream<ch, char_traits>::sentry ok(is);
      if (ok)
      {
        unsigned char data[16];
        
        typedef std::ctype<ch> ctype_t;
        ctype_t const& ctype = std::use_facet<ctype_t>(is.getloc());
        
        ch xdigits[16];
        {
          char szdigits[] = "0123456789ABCDEF";
          ctype.widen(szdigits, szdigits + 16, xdigits);
        }
        ch* xdigits_end = xdigits + 16;
        
        ch c;
        for (std::size_t i=0; i < u.size() && is; ++i)
        {
          is >> c;
          c = ctype.toupper(c);
          
          ch* f = std::find(xdigits, xdigits_end, c);
          if (f == xdigits_end)
          {
            is.setstate(std::ios_base::failbit);
            break;
          }
          
          unsigned char byte = static_cast<unsigned char>(std::distance(&xdigits[0], f));
          
          is >> c;
          c = ctype.toupper(c);
          f = std::find(xdigits, xdigits_end, c);
          if (f == xdigits_end) {
            is.setstate(std::ios_base::failbit);
            break;
          }
          
          byte <<= 4;
          byte |= static_cast<unsigned char>(std::distance(&xdigits[0], f));
          
          data[i] = byte;
          
          if (is)
          {
            if (i == 3 || i == 5 || i == 7 || i == 9)
            {
              is >> c;
              if (c != is.widen('-')) is.setstate(std::ios_base::failbit);
            }
          }
        }
        
        if (is)
          std::copy(data, data+16, u.begin());
      }
      return is;
    }
    
    std::string to_string(const uuid& u);
    
    std::wstring to_wstring(const uuid& u);
   
    uuid random_uuid();
  
    namespace detail
    {
    
      template <typename CharIterator>
      typename std::iterator_traits<CharIterator>::value_type
      get_next_char(CharIterator& begin, CharIterator end)
      {
        if (begin == end)
        {
            throw std::runtime_error("invalid uuid string");
        }
        return *begin++;
      }

      unsigned char get_value(char c);

      unsigned char get_value(wchar_t c);

      bool is_dash(char c);
    
      bool is_dash(wchar_t c);
    
      // return closing brace
      bool is_open_brace(char c);
    
      bool is_open_brace(wchar_t c);
    
      void check_close_brace(char c, char open_brace);
    
      void check_close_brace(wchar_t c, wchar_t open_brace);

      template <typename CharIterator>
      uuid parse_uuid(CharIterator begin, CharIterator end)
      {
        typedef typename std::iterator_traits<CharIterator>::value_type char_type;

        // check open brace
        char_type c = get_next_char(begin, end);
        bool has_open_brace = is_open_brace(c);
        char_type open_brace_char = c;
        if (has_open_brace)
        {
            c = get_next_char(begin, end);
        }

        bool has_dashes = false;

        uuid u;
        int i=0;
        for(uuid::iterator it_byte=u.begin(); it_byte!=u.end(); ++it_byte, ++i)
        {
          if(it_byte != u.begin())
          {
            c = get_next_char(begin, end);
          }
        
          if(i == 4)
          {
            has_dashes = is_dash(c);
            if (has_dashes)
              c = get_next_char(begin, end);
          }
        
          if(has_dashes)
          {
            if (i == 6 || i == 8 || i == 10)
            {
              if (is_dash(c))
                c = get_next_char(begin, end);
              else
                throw std::runtime_error("invalid uuid string");
            }
          }

          *it_byte = get_value(c);

          c = get_next_char(begin, end);
          *it_byte <<= 4;
          *it_byte |= get_value(c);
        }

        // check close brace
        if (has_open_brace)
        {
          c = get_next_char(begin, end);
          check_close_brace(c, open_brace_char);
        }
        return u;
      }
    }
  
    uuid parse_uuid(const char* const s);

    uuid parse_uuid(const wchar_t* const s);
  
    template <typename ch, typename char_traits, typename alloc>
    uuid parse_uuid(const std::basic_string<ch, char_traits, alloc>& s)
    {
      return detail::parse_uuid(s.begin(), s.end());
    }
  
  
  }
}

namespace std
{
  template<>
  struct OWL_API hash<owl::utils::uuid>
  {
    std::size_t operator()(const owl::utils::uuid& value) const
    {
      return owl::utils::hash_value(value.begin(), value.end());
    }
  };
}
