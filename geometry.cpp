/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#include "geometry.h"

#include <array>

Vec3f TriangleS::barycentric(const Vec2i& p) const {
  Vec3f x(pts[1][0] - pts[0][0], pts[2][0] - pts[0][0], pts[0][0] - p[0]);
  Vec3f y(pts[1][1] - pts[0][1], pts[2][1] - pts[0][1], pts[0][1] - p[1]);
  Vec3f u = x ^ y;
  if (std::abs(u[2]) < 1) {
    return Vec3f(-1, 1, 1);
  }
  return Vec3f(1. - (u[0] + u[1]) / u[2], u[0] / u[2], u[1] / u[2]);
}

// only two coordinates of p needed to find u, v
Vec3f TriangleW::barycentric(const Vec3f& p) const {
  Vec3f x(pts[1][0] - pts[0][0], pts[2][0] - pts[0][0], pts[0][0] - p[0]);
  Vec3f y(pts[1][1] - pts[0][1], pts[2][1] - pts[0][1], pts[0][1] - p[1]);
  Vec3f u = x ^ y;
  return Vec3f(1. - (u[0] + u[1]) / u[2], u[0] / u[2], u[1] / u[2]);
}
