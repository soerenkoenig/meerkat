#include "owl/utils/base64.hpp"

namespace owl
{
  namespace utils
  {
    static const std::string base64_chars =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz"
      "0123456789+/";
    
    
    static inline bool is_base64(unsigned char c)
    {
      return (isalnum(c) || (c == '+') || (c == '/'));
    }
  
    std::size_t base64_encoded_size(std::size_t n)
    {
      return ((4 * n / 3) + 3) & ~3;
    }
  
    std::size_t base64_decoded_size(const std::string& encoded_string)
    {
      std::size_t padded_size = 0;
      auto it = encoded_string.rbegin();
      while(it != encoded_string.rend() && (*it--  == '='))
        padded_size++;
    
      return (encoded_string.size() * 3 / 4) - padded_size;
    }
  
    std::string base64_encode(const unsigned char* bytes_to_encode, std::size_t len)
    {
      std::string ret(base64_encoded_size(len),0);
      int i = 0;
      int j = 0;
      unsigned char char_array_3[3];
      unsigned char char_array_4[4];
      auto it = ret.begin();
      while (len--)
      {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3)
        {
          char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
          char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
          char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
          char_array_4[3] = char_array_3[2] & 0x3f;

          for(i = 0; (i <4) ; i++)
            *it++ = base64_chars[char_array_4[i]];
          i = 0;
        }
      }

      if (i)
      {
        for(j = i; j < 3; j++)
          char_array_3[j] = '\0';

        char_array_4[0] = ( char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

        for (j = 0; (j < i + 1); j++)
          *it = base64_chars[char_array_4[j]];

        while((i++ < 3))
          *it = '=';
      }

      return ret;
    }
  
    std::string base64_encode(const buffer& buf)
    {
      return base64_encode(buf.data(), buf.size());
    }
      
    buffer base64_decode(const std::string& encoded_string)
    {
      size_t in_len = encoded_string.size();
      int i = 0;
      int j = 0;
      int in_ = 0;
      unsigned char char_array_4[4], char_array_3[3];
      buffer ret(base64_decoded_size(encoded_string));
      auto it = ret.begin();
      while (in_len && ( encoded_string[in_] != '=') && is_base64(encoded_string[in_]))
      {
        in_len--;
        char_array_4[i++] = encoded_string[in_]; in_++;
        if (i ==4)
         {
          for (i = 0; i <4; i++)
            char_array_4[i] = base64_chars.find(char_array_4[i]);

          char_array_3[0] = ( char_array_4[0] << 2       ) + ((char_array_4[1] & 0x30) >> 4);
          char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
          char_array_3[2] = ((char_array_4[2] & 0x3) << 6) +   char_array_4[3];

          for (i = 0; (i < 3); i++)
            *it++ = char_array_3[i];
          i = 0;
        }
      }

      if (i) {
        for (j = 0; j < i; j++)
          char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);

        for (j = 0; (j < i - 1); j++) *it++ = char_array_3[j];
      }

      return ret;
    }
  }
}

