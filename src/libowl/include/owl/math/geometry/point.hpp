//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include "owl/math/matrix.hpp"

namespace owl
{
  namespace math
  {
    namespace geometry
    {

      template <typename Scalar, std::size_t Dimension>
      class point
      {
      public:
        using scalar = Scalar;
        using vector = math::vector<Scalar,Dimension>;
        using reference = typename vector::reference;
        using const_reference = typename vector::const_reference;
        using size_type = typename vector::size_type;
        using iterator = typename vector::iterator;
        using const_iterator = typename vector::const_iterator;

        const point& closest_point(const point& q) const
        {
          return  *this;
        }

        constexpr point() = default;
        point(const point& ) = default;
        point(point&& ) = default;

        point& operator=(const point& ) = default;
        point& operator=(point&& ) = default;

        template<typename... Args>
        constexpr point(Args&&... coeffs)
          : position_(coeffs...)
        {
        }

        inline iterator begin() { return position_.begin(); }
        inline const_iterator begin() const { return position_.begin(); }
        inline const_iterator cbegin() const { return position_.cbegin(); }

        inline iterator end() { return position_.end(); }
        inline const_iterator end() const { return position_.end(); }
        inline const_iterator cend() const { return position_.cend(); }

        reference operator[](size_type pos)
        {
          return position_[pos];
        }

        const_reference operator[](size_type pos) const
        {
          return position_[pos];
        }

        reference operator()(size_type pos)
        {
          return position_[pos];
        }

        const_reference operator()(size_type pos) const
        {
          return position_[pos];
        }

        operator const vector&() const
        {
          return position_;
        }

        operator vector&()
        {
          return position_;
        }

        const vector& as_vector() const
        {
          return position_;
        }

        vector& as_vector()
        {
          return position_;
        }

        inline vector operator-(const point& other) const
        {
          return position_ - other.position_;
        }

        inline point operator+=(const vector& other)
        {
          position_ += other;
        }

        inline point operator+(const vector& other) const
        {
          return point{position_ + other};
        }

        template <typename T>
        point operator*(T&& s) const
        {
          return point{position_*s};
        };

        template<typename T>
        auto operator/(T&& s) const
        {

          using result_type = point<std::common_type_t <Scalar,T>, Dimension>;
          return result_type{position_/s};
        };

        inline scalar& x()
        {
          return position_.x();
        }

        inline const scalar& x() const
        {
          return position_.x();
        }

        inline scalar& y()
        {
          return position_.y();
        }

        inline const scalar& y() const
        {
          return position_.y();
        }

        inline scalar& z()
        {
          return position_.z();
        }

        inline const scalar& z() const
        {
          return position_.z();
        }

        inline bool operator==(const point& other) const
        {
          return position_ == other.position_;
        }

        inline bool operator!=(const point& other) const
        {
          return position_ != other.position_;
        }

        inline bool operator<(const point& other) const
        {
          return position_ < other.position_;
        }

        inline bool operator<=(const point& other) const
        {
          return position_ <= other.position_;
        }

        inline bool operator>(const point& other) const
        {
          return position_ > other.position_;
        }

        inline bool operator>=(const point& other) const
        {
          return position_ >= other.position_;
        }

        point lerp(const point& other, scalar t) const
        {
          return point{(t-1)*position_ + t*other.position_ };
        }

        static constexpr point origin()
        {
          return point{vector::zero()};
        }

        static constexpr point one()
        {
          return point{vector::one()};
        }

        static constexpr point max()
        {
          return point(vector::max());
        }

        static constexpr point lowest()
        {
          return point(vector::lowest());
        }

      private:
        vector position_;
      };

      using point3f = point<float,3>;
      using point2f = point<float,2>;

      using point3d = point<double,3>;
      using point2d = point<double,2>;

      template<typename Scalar, std::size_t Dimension>
      std::ostream& operator<<(std::ostream& out, const point<Scalar,Dimension>& p)
      {
        return out << transpose(typename point<Scalar,Dimension>::vector(p));
      }


      template <typename S, std::size_t Dimension>
      auto operator*(const S& s, const point<S,Dimension>& p)
      {
        return p*s;
      };

      template<typename S, std::size_t N, typename S2>
      inline auto comp_max(const point<S,N>& lhs, const point<S2, N>& rhs)
      {
        return math::comp_max<S,N,1,S2>(lhs,rhs);
      }

      template<typename S, std::size_t N, typename S2>
      inline auto comp_min(const point<S, N>& lhs, const point<S2, N>& rhs)
      {
        return math::comp_min<S,N,1,S2>(lhs,rhs);
      }


     /* template <typename Scalar, std::size_t Dimension>
      Scalar sqr_distance(const point<Scalar,Dimension>& a, const point<Scalar,Dimension>& b)
      {
        auto v = a-b;
        return v.sqr_length();
      }

      template <typename Scalar, std::size_t Dimension>
      Scalar distance(const point<Scalar,Dimension>& a, const point<Scalar,Dimension>& b)
      {
        return std::sqrt(sqr_distance(a,b));
      }*/
    }
  }
}



