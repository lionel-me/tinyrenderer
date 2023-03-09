/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <cassert>
#include <cmath>
#include <ostream>
#include <vector>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <int n>
struct vec {
  double data[n] = {0};
  double& operator[](const int i) {
    assert(i >= 0 && i < n);
    return data[i];
  }
  double operator[](const int i) const {
    assert(i >= 0 && i < n);
    return data[i];
  }
  double norm2() const { return *this * *this; }
  double norm() const { return std::sqrt(norm2()); }
};

template <int n>
double operator*(const vec<n>& lhs, const vec<n>& rhs) {
  double ret = 0;
  for (int i = n; i--; ret += lhs[i] * rhs[i]) {
  }
  return ret;
}

template <int n>
vec<n> operator+(const vec<n>& lhs, const vec<n>& rhs) {
  vec<n> ret = lhs;
  for (int i = n; i--; ret[i] += rhs[i]) {
  }
  return ret;
}

template <int n>
vec<n> operator-(const vec<n>& lhs, const vec<n>& rhs) {
  vec<n> ret = lhs;
  for (int i = n; i--; ret[i] -= rhs[i]) {
  }
  return ret;
}

template <int n>
vec<n> operator*(const double& rhs, const vec<n>& lhs) {
  vec<n> ret = lhs;
  for (int i = n; i--; ret[i] *= rhs) {
  }
  return ret;
}

template <int n>
vec<n> operator*(const vec<n>& lhs, const double& rhs) {
  vec<n> ret = lhs;
  for (int i = n; i--; ret[i] *= rhs) {
  }
  return ret;
}

template <int n>
vec<n> operator/(const vec<n>& lhs, const double& rhs) {
  vec<n> ret = lhs;
  for (int i = n; i--; ret[i] /= rhs) {
  }
  return ret;
}

template <int n>
vec<n> pointify(const vec<n>& in) {
  vec<n> ret;
  for (int i = 0; i < n - 1; i++) {
    ret[i] = in[i] / in[n - 1];
  }
  ret[n - 1] = 1;
  return ret;
}

template <int n1, int n2>
vec<n1> embed(const vec<n2>& v, double fill = 1) {
  vec<n1> ret;
  for (int i = n1; i--; ret[i] = (i < n2 ? v[i] : fill)) {
  }
  return ret;
}

template <int n1, int n2>
vec<n1> proj(const vec<n2>& v) {
  vec<n1> ret;
  for (int i = n1; i--; ret[i] = v[i]) {
  }
  return ret;
}

template <int n>
std::ostream& operator<<(std::ostream& out, const vec<n>& v) {
  for (int i = 0; i < n; i++) out << v[i] << " ";
  return out;
}

typedef vec<4> vec4;
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
  explicit Vec3(const vec4& v) : x(v[0]), y(v[1]), z(v[2]) {}
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
  vec4 to_homogeneous() const {
    vec4 result;
    result[0] = x;
    result[1] = y;
    result[2] = z;
    result[3] = 1;
    return result;
  }
};

typedef Vec2<double> Vec2f;
typedef Vec2<int> Vec2i;
typedef Vec3<double> Vec3f;
typedef Vec3<int> Vec3i;

template <class t, int r, int c>
struct Matrix {
  std::vector<std::vector<t>> rows;
  int row_, col_;
  Matrix() {
    row_ = r;
    col_ = c;
    for (int i = 0; i < r; i++) rows.push_back(std::vector<t>(c, 0));
  }
  explicit Matrix(t v) {
    row_ = r;
    col_ = c;
    for (int i = 0; i < r; i++) rows.push_back(std::vector<t>(c, v));
  }
  int nrows() const { return row_; }
  int ncols() const { return col_; }
  inline std::vector<t>& operator[](const int& i) { return rows[i]; }
  inline std::vector<t> operator[](const int& i) const { return rows[i]; }
  vec<r> operator*(const vec<r>& a) const {
    vec<r> ret;
    for (int i = 0; i < row_; i++) {
      for (int j = 0; j < col_; j++) {
        ret[i] += rows[i][j] * a[j];
      }
    }
    return ret;
  }
  vec<r> multiply(const vec<c>& a) const {
    vec<r> ret;
    for (int i = 0; i < row_; i++) {
      for (int j = 0; j < col_; j++) {
        ret[i] += rows[i][j] * a[j];
      }
    }
    return ret;
  }
};

template <class t, int lr, int lc, int rc>
Matrix<t, lr, rc> operator*(const Matrix<t, lr, lc>& lhs,
                            const Matrix<t, lc, rc>& rhs) {
  Matrix<t, lr, rc> result;
  for (int i = 0; i < lr; i++) {
    for (int j = 0; j < rc; j++) {
      for (int k = 0; k < lc; k++) {
        result[i][j] += lhs[i][k] * rhs[k][j];
      }
    }
  }
  return result;
}

typedef Matrix<double, 4, 4> MatrixW;

template <class t, int l>
Matrix<t, l, l> identity() {
  Matrix<t, l, l> ret;
  for (int i = 0; i < l; i++) ret[i][i] = 1;
  return ret;
}

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
