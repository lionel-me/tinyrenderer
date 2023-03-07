/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <cmath>
#include <ostream>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class t>
struct Vec2 {
  union {
    struct {
      t u, v;
    };
    struct {
      t x, y;
    };
    t raw[2];
  };
  Vec2() : u(0), v(0) {}
  Vec2(t _u, t _v) : u(_u), v(_v) {}
  inline Vec2<t> operator+(const Vec2<t>& V) const {
    return Vec2<t>(u + V.u, v + V.v);
  }
  inline Vec2<t> operator-(const Vec2<t>& V) const {
    return Vec2<t>(u - V.u, v - V.v);
  }
  inline t& operator[](const int i) { return raw[i]; }
  inline t operator[](const int i) const { return raw[i]; }
  inline Vec2<t> operator*(double f) const { return Vec2<t>(u * f, v * f); }
  template <class>
  friend std::ostream& operator<<(std::ostream& s, Vec2<t>& v);
};

template <class t>
struct Vec3 {
  union {
    struct {
      t x, y, z;
    };
    struct {
      t ivert, iuv, inorm;
    };
    t raw[3];
  };
  Vec3() : x(0), y(0), z(0) {}
  Vec3(t _x, t _y, t _z) : x(_x), y(_y), z(_z) {}
  inline Vec3<t> operator^(const Vec3<t>& v) const {
    return Vec3<t>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }
  inline Vec3<t> operator+(const Vec3<t>& v) const {
    return Vec3<t>(x + v.x, y + v.y, z + v.z);
  }
  inline Vec3<t> operator-(const Vec3<t>& v) const {
    return Vec3<t>(x - v.x, y - v.y, z - v.z);
  }
  inline Vec3<t> operator*(double f) const {
    return Vec3<t>(x * f, y * f, z * f);
  }
  inline t operator*(const Vec3<t>& v) const {
    return x * v.x + y * v.y + z * v.z;
  }
  inline Vec3<t> operator/(double f) const {
    return Vec3<t>(x / f, y / f, z / f);
  }
  inline t& operator[](const int i) { return raw[i]; }
  inline t operator[](const int i) const { return raw[i]; }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3<t>& normalize(t l = 1) {
    *this = (*this) * (l / norm());
    return *this;
  }
};

typedef Vec2<double> Vec2f;
typedef Vec2<int> Vec2i;
typedef Vec3<double> Vec3f;
typedef Vec3<int> Vec3i;

struct TriangleS {
  Vec2i pts[3];
  TriangleS() = default;
  TriangleS(const Vec2i& p0, const Vec2i& p1, const Vec2i& p2)
      : pts{p0, p1, p2} {}
  inline Vec2i operator[](const int i) const { return pts[i]; }
  inline Vec2i& operator[](const int i) { return pts[i]; }
  Vec3f barycentric(const Vec2i& p) const;
};

struct TriangleW {
  Vec3f pts[3];
  TriangleW() = default;
  TriangleW(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2)
      : pts{p0, p1, p2} {}
  inline Vec3f operator[](const int i) const { return pts[i]; }
  inline Vec3f& operator[](const int i) { return pts[i]; }
  Vec3f normal() { return ((pts[2] - pts[0]) ^ (pts[1] - pts[0])).normalize(); }
  Vec3f barycentric(const Vec3f& p) const;
};

#endif  // GEOMETRY_H_
