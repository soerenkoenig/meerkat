
//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2017 Sören König. All rights reserved.
//

#pragma once

#include "owl/math/matrix.hpp"
#include "owl/math/primitive_traits.hpp"

namespace owl
{
  namespace math
  {
    namespace detail
    {

      template <typename Scalar, std::size_t Dimension>
      struct interval_helper
      {
        using scalar_type = Scalar;
        using bound_type = vector<Scalar, Dimension>;
        
        static constexpr bound_type max()
        {
          return bound_type::max();
        }
        
        static constexpr bound_type lowest()
        {
          return bound_type::lowest();
        }
        
        static bound_type prev(const bound_type& p)
        {
          bound_type p_prev;
          for(std::size_t i = 0; i < Dimension; ++i)
            p_prev(i) = interval_helper<Scalar,1>::prev(p(i));
          return p_prev;
        }
        
        static bound_type next(const bound_type& p)
        {
          bound_type p_next;
          for(std::size_t i = 0; i < Dimension; ++i)
            p_next(i) = interval_helper<Scalar,1>::next(p(i));
          return p_next;
        }
        
        static bound_type minimum(const bound_type& a, const bound_type& b)
        {
          return comp_min(a,b);
        }
        
        static bound_type maximum(const bound_type& a, const bound_type& b)
        {
         return comp_max(a,b);
        }
        
        template<bool LowerBoundOpen, bool UpperBoundOpen>
        static bool inside(const bound_type& lo, const bound_type& hi, const bound_type& p)
        {
          for(std::size_t i = 0; i < Dimension; ++i)
              if(!interval_helper<Scalar,1>::template inside<LowerBoundOpen, UpperBoundOpen>(lo(i), hi(i), p(i)))
                return false;
          return true;
        }
      
        template<bool LowerBoundOpen, bool UpperBoundOpen>
        static bool empty(const bound_type& lo, const bound_type& hi)
        {
           for(std::size_t i = 0; i < Dimension; ++i)
              if(interval_helper<Scalar,1>::template empty<LowerBoundOpen, UpperBoundOpen>(lo(i), hi(i)))
                return true;
          return false;
        }
      };
      
      template <typename Scalar>
      struct interval_helper<Scalar, 1>
      {
        using scalar_type = Scalar;
        using bound_type = Scalar;
        
        static constexpr bound_type max()
        {
          return std::numeric_limits<bound_type>::max();
        }
        
        static constexpr bound_type lowest()
        {
          return std::numeric_limits<bound_type>::lowest();
        }
        
        static bound_type next(const bound_type& p)
        {
          if constexpr(std::is_floating_point_v<bound_type>)
            return std::nextafter(p, max());
          if constexpr(std::is_integral_v<bound_type>)
            return p == max() ? p : ++p;
          return p;
        }
        
        static bound_type prev(const bound_type& p)
        {
          if constexpr(std::is_floating_point_v<bound_type>)
            return std::nextafter(p, lowest());
          if constexpr(std::is_integral_v<bound_type>)
            return p == lowest() ? p : --p;
          return p;
        }
        
        template<bool LowerBoundOpen, bool UpperBoundOpen>
        static bool inside(const bound_type& lo, const bound_type& hi, const bound_type& p)
        {
          if constexpr(LowerBoundOpen)
          {
           if(p <= lo)
            return false;
          }
          else
          {
            if(p < lo)
              return false;
          }
          if constexpr(UpperBoundOpen)
          {
            if(p >= hi)
              return false;
          }
          else
          {
            if(p > hi)
              return false;
          }
          return true;
        }
      
        
        static bound_type minimum(const bound_type& a, const bound_type& b)
        {
          return std::min(a, b);
        }
        
        static bound_type maximum(const bound_type& a, const bound_type& b)
        {
          return std::max(a, b);
        }
     
        template<bool LowerBoundOpen, bool UpperBoundOpen>
        static bool empty(const bound_type& lo, const bound_type& hi)
        {
          if constexpr(UpperBoundOpen || LowerBoundOpen)
           return lo >= hi;
          else
            return lo > hi;
        }
      };
    }

    template <typename Scalar, std::size_t Dimension = 1, bool LowerBoundOpen = false, bool UpperBoundOpen = true>
    class interval
    {
      using helper_type_ = typename detail::interval_helper<Scalar, Dimension>;

    public:
      using scalar_type = Scalar;
      using bound_type = typename helper_type_::bound_type;

      interval()
        : bounds{{helper_type_::max(), helper_type_::lowest()}}
      {
      }

      interval(const bound_type &b)
        : bounds{b, b}
      {
      }

      interval(const bound_type &lo, const bound_type &hi)
        : bounds{{lo, hi}}
      {
      }

      void clear()
      {
        *this = interval();
      }

      bool overlaps(const interval &other) const
      {
        return !intersect(other).empty();
      }

      bool inside(const bound_type &p) const
      {
        return helper_type_::template inside<LowerBoundOpen, UpperBoundOpen>(lower_bound, upper_bound, p);
      }

      bool inside(const interval &other) const
      {
        return inside(other.lower_bound) && inside(other.upper_bound);
      }

      interval intersect(const interval &other) const
      {
        return {helper_type_::maximum(lower_bound, other.lower_bound),
                helper_type_::minimum(upper_bound, other.upper_bound)};
      }

      auto center() const
      {
        return (lower_bound + upper_bound) / 2;
      }

      void insert(const interval &inter)
      {
          insert(lower_bound);
          insert(upper_bound);
      }

      //ensures p is inside the interval
      void insert(const bound_type &p)
      {
        if constexpr(LowerBoundOpen)
          lower_bound = helper_type_::minimum(lower_bound, helper_type_::prev(p));
        else
          lower_bound = helper_type_::minimum(lower_bound, p);

        if constexpr(UpperBoundOpen)
          upper_bound = helper_type_::maximum(upper_bound, helper_type_::next(p));
        else
          upper_bound = helper_type_::maximum(upper_bound, p);
      }

      bool empty() const
      {
        return helper_type_::template empty<LowerBoundOpen, UpperBoundOpen>(lower_bound, upper_bound);
      }

      auto extents() const
      {
        return upper_bound - lower_bound;
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      const auto &left() const
      {
        return lower_bound.x();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      auto &left()
      {
        return lower_bound.x();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      const auto &right() const
      {
        return upper_bound.x();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      auto &right()
      {
        return upper_bound.x();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      const auto &top() const
      {
        return lower_bound.y();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      auto &top()
      {
        return lower_bound.y();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      const auto &bottom() const
      {
        return upper_bound.y();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      auto &bottom()
      {
        return upper_bound.y();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      const auto &near() const
      {
        return lower_bound.z();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      auto &near()
      {
        return lower_bound.z();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      const auto &far() const
      {
        return upper_bound.z();
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      auto &far()
      {
        return upper_bound.z();
      }

      auto width() const
      {
        return upper_bound[0] - lower_bound[0];
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 2>>
      auto height() const
      {
        return upper_bound[1] - lower_bound[1];
      }

      template<bool Dummy = true, typename = std::enable_if_t<Dummy && Dimension >= 3>>
      auto depth() const
      {
        return upper_bound[2] - lower_bound[2];
      }

      bool operator==(const interval &other) const
      {
        return bounds == other.bounds;
      }

      bool operator!=(const interval &other) const
      {
        return bounds != other.bounds;
      }


      void enlarge(const Scalar& factor, const Scalar& offset)
      {

        auto c = center();
        decltype(c) one;
        if constexpr(Dimension == 1)
          one = 1;
        else
        {
          for(auto& c : one)
            c = Scalar(1);
        }
        lower_bound = (Scalar(1)-factor) * c + factor * lower_bound - offset*one;
        upper_bound = (Scalar(1)-factor) * c + factor * upper_bound + offset*one;
      }

      auto closest_point(const bound_type &p) const
      {
        bound_type q;
        if constexpr (Dimension == 1)
        {
          q = (std::min)((std::max)(p, lower_bound), upper_bound);
        }
        else
        {
          for (std::size_t i = 0; i < Dimension; ++i)
            q[i] = (std::min)((std::max)(p[i], lower_bound[i]), upper_bound[i]);
        }
        return q;
      }
      auto sqr_distance(const bound_type& q) const
      {
        return math::sqr_distance(q, closest_point(q));
      }
    
      union
      {
        std::array<bound_type, 2> bounds;
        struct
        {
          bound_type lower_bound;
          bound_type upper_bound;
        };
      };
    };

    template <typename Scalar, std::size_t Dimension, bool LowerBoundOpen, bool UpperBoundOpen>
    auto sqr_distance(const interval<Scalar,Dimension,LowerBoundOpen,UpperBoundOpen>& inter,
                      const typename interval<Scalar,Dimension,LowerBoundOpen,UpperBoundOpen>::bound_type& q)
    {
      return inter.sqr_distance(q);
    }

    template <typename Scalar, std::size_t Dimension, bool LowerBoundOpen, bool UpperBoundOpen>
    auto closest_point(const interval<Scalar,Dimension,LowerBoundOpen,UpperBoundOpen>& inter,
                      const typename interval<Scalar,Dimension,LowerBoundOpen,UpperBoundOpen>::bound_type& q)
    {
      return inter.closest_point(q);
    }
  
    template <typename ValueRange, bool LowerBoundOpen = false, bool UpperBoundOpen = true, typename = typename utils::is_iterable<ValueRange>::value>
    auto bounds(ValueRange&& values)
    {
      using Value = std::decay_t<decltype(*std::begin(values))>;
      using Scalar = typename scalar_t<Value>::type;
      interval<Scalar, dimension_t<Value>::value, LowerBoundOpen, UpperBoundOpen> b;
      for(const auto& v : values)
        b.insert(v);
  
      return b;
    }
  
    template <typename Scalar, bool LowerBoundOpen, bool UpperBoundOpen>
    std::ostream& operator<<(std::ostream& out, const interval<Scalar, 1, LowerBoundOpen, UpperBoundOpen>& inter)
    {
      return out << (LowerBoundOpen ? "( ":"[ ") <<inter.lower_bound << ", "<<inter.upper_bound<< (UpperBoundOpen ? ")":"]");
    }
  
    template <typename Scalar, std::size_t Dimension, bool LowerBoundOpen, bool UpperBoundOpen>
    std::ostream& operator<<(std::ostream& out, const interval<Scalar, Dimension, LowerBoundOpen, UpperBoundOpen>& inter)
    {
      return out << (LowerBoundOpen ? "( ":"[ ") << "(" << inter.lower_bound<<")" << ", ("<< inter.upper_bound<<") "<< (UpperBoundOpen ? ")":"]");
    }
    
    template <typename Scalar, bool LowerBoundOpen = false, bool UpperBoundOpen = true>
    using rectangle = interval<Scalar, 2, LowerBoundOpen, UpperBoundOpen>;
    
    template <typename Scalar, bool LowerBoundOpen = false, bool UpperBoundOpen = true>
    using box = interval<Scalar, 3, LowerBoundOpen, UpperBoundOpen>;
  }
}

