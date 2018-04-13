#include "owl/utils/uuid.hpp"

#include "owl/utils/random_utils.hpp"

namespace owl
{
  namespace utils
  {
  
    uuid::iterator uuid::begin() noexcept
    {
      return data.begin();
    }
 
    uuid::const_iterator uuid::begin() const noexcept
    {
      return data.cbegin();
    }
    
    uuid::iterator uuid::end() noexcept
    {
      return data.end();
    }
    
    uuid::const_iterator uuid::end() const noexcept
    {
      return data.cend();
    }
  
    bool uuid::operator==(uuid const& other) noexcept
    {
      return data == other.data;
    }
  
    bool uuid::operator!=( uuid const& other) noexcept
    {
      return data != other.data;
    }
  
    bool uuid::operator<( uuid const& other) noexcept
    {
      return data < other.data;
    }
    
    bool uuid::operator>(uuid const& other) noexcept
    {
      return data > other.data;
    }
    
    bool uuid::operator<=(uuid const& other) noexcept
    {
      return data <= other.data;
    }
      
    bool uuid::operator>=(uuid const& other) noexcept
    {
      return data >= data;
    }
    
    uuid::variant_type uuid::variant() const noexcept
    {
      // variant is stored in octet 7
      // which is index 8, since indexes count backwards
      std::uint8_t octet7 = data[8]; // octet 7 is array index 8
      if ( (octet7 & 0x80) == 0x00 )
      { // 0b0xxxxxxx
        return variant_ncs;
      }
      else if ( (octet7 & 0xC0) == 0x80 )
      { // 0b10xxxxxx
        return variant_rfc_4122;
      }
      else if ( (octet7 & 0xE0) == 0xC0 )
      { // 0b110xxxxx
        return variant_microsoft;
      }
      else
      {
        return variant_future;
      }
    }
      
    uuid::version_type uuid::version() const noexcept
    {
      // version is stored in octet 9
      // which is index 6, since indexes count backwards
      std::uint8_t octet9 = data[6];
      if ( (octet9 & 0xF0) == 0x10 )
      {
        return version_time_based;
      }
      else if ( (octet9 & 0xF0) == 0x20 )
      {
        return version_dce_security;
      }
      else if ( (octet9 & 0xF0) == 0x30 )
      {
        return version_name_based_md5;
      }
      else if ( (octet9 & 0xF0) == 0x40 )
      {
        return version_random_number_based;
      }
      else if ( (octet9 & 0xF0) == 0x50 )
      {
        return version_name_based_sha1;
      }
      else
      {
        return version_unknown;
      }
    }
      
     
    namespace detail
    {
       char to_char(size_t i) {
        if (i <= 9) {
          return static_cast<char>('0' + i);
        } else {
          return static_cast<char>('a' + (i-10));
        }
      }
    
       wchar_t to_wchar(size_t i) {
        if (i <= 9) {
          return static_cast<wchar_t>(L'0' + i);
        } else {
          return static_cast<wchar_t>(L'a' + (i-10));
        }
      }
    
    } // namespace detail
 
  
    std::string to_string(const uuid& u)
    {
      std::string result;
      result.reserve(36);
      
      std::size_t i=0;
      for (uuid::const_iterator it_data = u.begin(); it_data!=u.end(); ++it_data, ++i)
      {
        const size_t hi = ((*it_data) >> 4) & 0x0F;
        result += detail::to_char(hi);
        
        const size_t lo = (*it_data) & 0x0F;
        result += detail::to_char(lo);
        
        if (i == 3 || i == 5 || i == 7 || i == 9)
          result += '-';
      }
      return result;
    }
    
    
    std::wstring to_wstring(uuid const& u)
    {
      std::wstring result;
      result.reserve(36);
      
      std::size_t i=0;
      for (uuid::const_iterator it_data = u.begin(); it_data!=u.end(); ++it_data, ++i)
      {
        const size_t hi = ((*it_data) >> 4) & 0x0F;
        result += detail::to_wchar(hi);
        
        const size_t lo = (*it_data) & 0x0F;
        result += detail::to_wchar(lo);
        
        if (i == 3 || i == 5 || i == 7 || i == 9)
          result += L'-';
      }
      return result;
    }
     
    
    uuid random_uuid()
    {
      uuid u;
      
      int i=0;
      using Engine = std::mt19937;
      using Distribution = std::uniform_int_distribution<unsigned int>;
      
      static auto generator = std::bind(Distribution(std::numeric_limits<unsigned int>::min(),
        std::numeric_limits<unsigned int>::max()), create_seeded_engine<Engine>());
    
      unsigned int random_value = generator();
      for (uuid::iterator it=u.begin(); it!=u.end(); ++it, ++i)
      {
        if (i == sizeof(unsigned int))
        {
          random_value = generator();
          i = 0;
        }
        
        *it = static_cast<uuid::value_type>((random_value >> (i * 8)) & 0xFF);
      }
      
      // set variant
      // must be 0b10xxxxxx
      *(u.begin() + 8) &= 0xBF;
      *(u.begin() + 8) |= 0x80;
      
      // set version
      // must be 0b0100xxxx
      *(u.begin() + 6) &= 0x4F; //0b01001111
      *(u.begin() + 6) |= 0x40; //0b01000000
      
      return u;
    }
    
    namespace detail
    {
      unsigned char get_value(char c)
      {
         static char const*const digits_begin = "0123456789abcdefABCDEF";
         static char const*const digits_end = digits_begin + 22;

         static unsigned char const values[] =
             { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,10,11,12,13,14,15
             , static_cast<unsigned char>(-1) };

         char const* d = std::find(digits_begin, digits_end, c);
         return values[d - digits_begin];
      }

      unsigned char get_value(wchar_t c)
      {
        static wchar_t const*const digits_begin = L"0123456789abcdefABCDEF";
        static wchar_t const*const digits_end = digits_begin + 22;
     
        static unsigned char const values[] =
            { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,10,11,12,13,14,15
            , static_cast<unsigned char>(-1) };

        wchar_t const* d = std::find(digits_begin, digits_end, c);
        return values[d - digits_begin];
      }

      bool is_dash(char c)
      {
        return c == '-';
      }
    
      bool is_dash(wchar_t c)
      {
        return c == L'-';
      }
    
      // return closing brace
      bool is_open_brace(char c)
      {
        return (c == '{') || (c == '(');
      }
    
      bool is_open_brace(wchar_t c)
      {
        return (c == L'{') || (c == L'(');
      }
    
      void check_close_brace(char c, char open_brace)
      {
        if ((open_brace == '{' && c == '}') || (open_brace == '(' && c == ')'))
        {
            //great
        }
        else
        {
          throw std::runtime_error("invalid uuid string");
        }
      }
    
      void check_close_brace(wchar_t c, wchar_t open_brace)
      {
        if ((open_brace == L'{' && c == L'}')|| (open_brace == L'(' && c == L')'))
        {
         //great
        }
        else
        {
          throw std::runtime_error("invalid uuid string");
        }
      }
    }

    uuid parse_uuid(const char* const s)
    {
      return detail::parse_uuid(s, s+std::strlen(s));
    }

    uuid parse_uuid(const wchar_t* const s)
    {
      return detail::parse_uuid(s, s+std::wcslen(s));
    }
    
  }
}
