#pragma once

#include <cmath>
#include "owl/math/matrix.hpp"

namespace owl
{
  namespace math
  {
    template<typename Scalar, std::size_t Derivatives = 1>
    class dual_number
    {
    public:
      using scalar = Scalar;
      using dual_t = std::conditional_t<Derivatives == 1, scalar, vector<Scalar, Derivatives>>;

      dual_number(const scalar &real = {}, const dual_t &dual = {})
        : m_real{real}, m_dual{dual}
      {}

      dual_number &operator+=(const dual_number &rhs)
      {
        m_real += rhs.m_real;
        m_dual += rhs.m_dual;
        return *this;
      }

      dual_number &operator+=(const scalar &rhs)
      {
        m_real += rhs;
        return *this;
      }

      dual_number operator+(const dual_number &rhs) const
      {
        return {m_real + rhs.m_real, m_dual + rhs.m_dual};
      }

      dual_number operator+(const scalar &rhs) const
      {
        return {m_real + rhs, m_dual};
      }

      dual_number &operator-=(const dual_number &rhs)
      {
        m_real -= rhs.m_real;
        m_dual -= rhs.m_dual;
        return *this;
      }

      dual_number &operator-=(const scalar &rhs)
      {
        m_real -= rhs.m_real;
        return *this;
      }

      dual_number operator-(const dual_number &rhs) const
      {
        return {m_real - rhs.m_real, m_dual - rhs.m_dual};
      }

      dual_number operator-(const scalar &rhs) const
      {
        return {m_real - rhs, m_dual};
      }

      dual_number operator*(const dual_number &rhs) const
      {
        return {m_real * rhs.m_real, rhs.m_dual * m_real + m_dual * rhs.m_real};
      }

      dual_number operator*(const scalar &rhs) const
      {

        return {m_real * rhs, m_dual * rhs};
      }

      dual_number &operator*=(const scalar &rhs)
      {
        m_real *= rhs;
        m_dual *= rhs;
        return *this;
      }

      dual_number &operator*=(const dual_number &rhs)
      {
        m_dual = rhs.m_dual * m_real + m_dual * rhs.m_real;
        m_real *= rhs.m_real;
        return *this;
      }

      dual_number operator-() const
      {
        return {-m_real, -m_dual};
      }

      dual_number operator/(const dual_number &rhs) const
      {
        return {m_real / rhs.m_real, (m_dual * rhs.m_real - m_real * rhs.m_dual) / (rhs.m_real * rhs.m_real)};
      }

      dual_number operator/(const scalar &rhs) const
      {
        return {m_real / rhs, m_dual / rhs};
      }

      dual_number &operator/=(const dual_number &rhs)
      {
        m_dual = (m_dual * rhs.m_real - m_real * rhs.m_dual) / (rhs.m_real * rhs.m_real);
        m_real /= rhs.m_real;
        return *this;
      }

      dual_number &operator/=(const scalar &rhs)
      {
        m_dual /= rhs;
        m_real /= rhs;
        return *this;
      }

      const scalar &real() const
      {
        return m_real;
      }

      const dual_t &dual() const
      {
        return m_dual;
      }


    private:
      scalar m_real;
      dual_t m_dual;
    };

    template<typename Scalar, std::size_t Derivatives>
    bool operator<(const dual_number<Scalar, Derivatives> &lhs, const Scalar &rhs)
    {
      return lhs.real() < rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator<(const dual_number<Scalar, Derivatives> &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs.real() < rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator<(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs < rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator<=(const dual_number<Scalar> &lhs, const Scalar &rhs)
    {
      return lhs.real() <= rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator<=(const dual_number<Scalar, Derivatives> &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs.real() <= rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator<=(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs < rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator>(const dual_number<Scalar, Derivatives> &lhs, const Scalar &rhs)
    {
      return lhs.real() > rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator>(const dual_number<Scalar, Derivatives> &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs.real() > rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator>(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs > rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator>=(const dual_number<Scalar, Derivatives> &lhs, const Scalar &rhs)
    {
      return lhs.real() >= rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator>=(const dual_number<Scalar, Derivatives> &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs.real() >= rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator>=(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs >= rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator==(const dual_number<Scalar, Derivatives> &lhs, const Scalar &rhs)
    {
      return lhs.real() == rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator==(const dual_number<Scalar, Derivatives> &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs.real() == rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator==(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs == rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator!=(const dual_number<Scalar, Derivatives> &lhs, const Scalar &rhs)
    {
      return lhs.real() != rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator!=(const dual_number<Scalar, Derivatives> &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs.real() != rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    bool operator!=(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return lhs != rhs.real();
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> sin(const dual_number<Scalar, Derivatives> &x)
    {
      return {std::sin(x.real()), std::cos(x.real()) * x.dual()};
    };

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> cos(const dual_number<Scalar, Derivatives> &x)
    {
      return {std::cos(x.real()), -std::sin(x.real()) * x.dual()};
    };

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> sqrt(const dual_number<Scalar, Derivatives> &x)
    {
      Scalar sqrt_real = std::sqrt(x.real());
      return {sqrt_real, x.dual() / (Scalar(2) * sqrt_real)};
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> pow(const dual_number<Scalar, Derivatives> &x, const Scalar &y)
    {
      return {std::pow(x.real(), y), x.dual() * y * std::pow(x.real(), y - Scalar(1))};
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> tan(const dual_number<Scalar, Derivatives> &x)
    {
      Scalar cos_real = std::cos(x.real());
      return {std::tan(x.real()), x.dual() / (cos_real * cos_real)};
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> atan(const dual_number<Scalar, Derivatives> &x)
    {
      Scalar cos_real = std::cos(x.real());
      return {std::atan(x.real()), x.dual() / (Scalar(1) + x.real() * x.real())};
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> max(const dual_number<Scalar, Derivatives> &lhs,
      const dual_number<Scalar, Derivatives> &rhs)
    {
      return (lhs > rhs) ? lhs : rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> max(const dual_number<Scalar, Derivatives> &lhs, const Scalar &rhs)
    {
      return (lhs > rhs) ? lhs : rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> max(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return (lhs > rhs) ? lhs : rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> min(const dual_number<Scalar, Derivatives> &lhs,
      const dual_number<Scalar, Derivatives> &rhs)
    {
      return (lhs < rhs) ? lhs : rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> min(const dual_number<Scalar, Derivatives> &lhs, const Scalar &rhs)
    {
      return (lhs < rhs) ? lhs : rhs;
    }

    template<typename Scalar, std::size_t Derivatives>
    dual_number<Scalar, Derivatives> min(const Scalar &lhs, const dual_number<Scalar, Derivatives> &rhs)
    {
      return (lhs < rhs) ? lhs : rhs;
    }


    template<typename Char, typename Scalar, std::size_t Derivatives>
    std::basic_ostream<Char> &operator<<(std::basic_ostream<Char> &out, const dual_number<Scalar, Derivatives> &d)
    {
      out.put(out.widen('('));
      out << d.real();
      out.put(out.widen(','));
      out << d.dual();
      out.put(out.widen(')'));
      return out;
    }
  }
}

namespace std
{
  template<typename Scalar, std::size_t Derivatives>
  struct hash<owl::math::dual_number < Scalar, Derivatives>>
{
  std::size_t operator()(const owl::math::dual_number <Scalar, Derivatives> &d) const
  {
    std::size_t h = owl::utils::hash_value(d.dual());
    owl::utils::hash_combine(h, d.real());
    return h;
  }
};









