/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */

#include "gl.h"

#include "global.h"

namespace gl {

void GL::lookat(const Vec3f &eye, const Vec3f &center, const Vec3f &up) {
  Vec3f z = (eye - center).normalize();
  Vec3f x = (up ^ z).normalize();
  Vec3f y = (z ^ x).normalize();
  MatrixW minv = identity<double, 4>();
  MatrixW tr = identity<double, 4>();
  for (int i = 0; i < 3; i++) {
    minv[0][i] = x[i];
    minv[1][i] = y[i];
    minv[2][i] = z[i];
    tr[i][3] = -center[i];
  }
  model_view_ = minv * tr;
}

void GL::projection(const Vec3f &eye, const Vec3f &center) {
  double dis = (eye - center).norm();
  projection_ = identity<double, 4>();
  projection_[3][2] = -1. / dis;
}

void GL::viewport(const int &x, const int &y, const int &w, const int &h) {
  view_port_[0][3] = x + w / 2.f;
  view_port_[1][3] = y + h / 2.f;
  view_port_[2][3] = kDepth / 2.f;
  view_port_[0][0] = w / 2.f;
  view_port_[1][1] = h / 2.f;
  view_port_[2][2] = kDepth / 2.f;
}
vec4 GL::transform(const vec4 &p) {
  return view_port_ * pointify(projection_ * model_view_ * p);
}

Vec3f central_projection(const Vec3f &camera, const Vec3f &in) {
  MatrixW proj = identity<double, 4>();
  vec4 in_h = in.to_homogeneous();
  proj[3][2] = -1. / camera[2];  // camera on the z axis
  return Vec3f(pointify(proj.multiply(in_h)));
}

// the triangles are given in world coordinates between -1 and 1
void GL::draw_triangle(const TriangleW &triangle, const int &iface,
                       TGAImage *image, int *z_buffer, const Vec3f &light_dir) {
  TriangleS triangle_s;  // triangle on screen coordinates
  Vec3i z;               // zs of three vertices
  Vec2i uvs[3];          // texture coordinates of three vertices
  Vec3f norms[3];        // normal vectors of three vertices
  for (int i = 0; i < 3; i++) {
    Vec3f p = Vec3f(transform(triangle[i].to_homogeneous()));
    triangle_s[i] = Vec2i(p.x, p.y);
    z[i] = static_cast<int>(p.z);
  }
  for (int j = 0; j < 3; j++) {
    uvs[j] = global::model->uv(iface, j);
  }
  for (int j = 0; j < 3; j++) {
    norms[j] = global::model->normal(iface, j);
  }
  // bounding box
  int xmin = image->width() - 1, xmax = 0, ymin = image->height() - 1, ymax = 0;
  for (int i = 0; i < 3; i++) {
    Vec2i p = triangle_s[i];
    xmin = std::max(std::min(xmin, p[0]), 0);
    xmax = std::min(std::max(xmax, p[0]), image->width() - 1);
    ymin = std::max(std::min(ymin, p[1]), 0);
    ymax = std::min(std::max(ymax, p[1]), image->height() - 1);
  }
// draw the image
#pragma omp parallel for
  for (int x = xmin; x <= xmax; x++) {
    for (int y = ymin; y <= ymax; y++) {
      Vec3f uv = triangle_s.barycentric(Vec2i(x, y));
      if (uv.x < 0 || uv.y < 0 || uv.z < 0) continue;
      int pz = 0;
      for (int i = 0; i < 3; i++) pz += uv[i] * z[i];  // average depth
      int index = x + y * image->width();
      if (z_buffer[index] >= pz) continue;
      z_buffer[index] = pz;
      Vec2i tc = uvs[0] * uv.x + uvs[1] * uv.y +
                 uvs[2] * uv.z;  // average texture coordinates
      TGAColor c = global::model->diffuse().get(tc.x, tc.y);  // texture color
      Vec3f n = norms[0] * uv.x + norms[1] * uv.y +
                norms[2] * uv.z;  // average normal vector
      double intensity =
          -(light_dir * n);  // intensity (notice that norms in
                             // obj file are from inside to outside)
      if (intensity < .0) continue;
      image->set(
          x, y,
          TGAColor(c[0] * intensity, c[1] * intensity, c[2] * intensity, 255));
    }
  }
}
}  // namespace gl
