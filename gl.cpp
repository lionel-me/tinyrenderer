/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */

#include "gl.h"

#include <vector>

namespace gl {

double max_elevation_angle(double *zbuffer, Vec2f p, Vec2f dir,
                           const TGAImage &image) {
  double maxangle = 0;
  for (double t = 0.; t < 1000.; t += 1.) {
    Vec2f cur = p + dir * t;
    if (cur.x >= image.width() || cur.y >= image.height() || cur.x < 0 ||
        cur.y < 0)
      return maxangle;

    double distance = (p - cur).norm();
    if (distance < 1.f) continue;
    double elevation =
        zbuffer[static_cast<int>(cur.x) +
                static_cast<int>(cur.y) * image.height()] -
        zbuffer[static_cast<int>(p.x) + static_cast<int>(p.y) * image.height()];
    maxangle =
        std::max(maxangle, static_cast<double>(atanf(elevation / distance)));
  }
  return maxangle;
}

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
  view_port_[3][3] = 1;
}
vec4 GL::transform(const vec4 &p) {
  return view_port_ * pointify(projection_ * model_view_ * p);
}

// the triangles are given in world coordinates between -1 and 1
void GL::draw_triangle(const TriangleW &triangle, const int &iface,
                       TGAImage *image, double *z_buffer,
                       const Vec3f &light_dir, Model *model,
                       double *shadow_buffer) {
  TriangleS triangle_s;  // triangle on screen coordinates
  Vec3f z;               // zs of three vertices
  Vec2f uvs[3];          // texture coordinates of three vertices
  Vec3f p_screen[3];     // p on screen coordinates
  for (int i = 0; i < 3; i++) {
    Vec3f p = Vec3f(transform(triangle[i].to_homogeneous(1.)));
    p_screen[i] = p;
    triangle_s[i] = Vec2i(p.x, p.y);
    z[i] = p.z;
  }
  for (int j = 0; j < 3; j++) {
    uvs[j] = model->uvf(iface, j);
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
      double pz = 0;
      for (int i = 0; i < 3; i++) pz += uv[i] * z[i];  // average depth
      int index = x + y * image->width();
      if (z_buffer[index] >= pz) continue;
      z_buffer[index] = pz;
      Vec2f tc = uvs[0] * uv.x + uvs[1] * uv.y +
                 uvs[2] * uv.z;         // average texture coordinates
      TGAColor c = model->diffuse(tc);  // texture color
      Vec3f n(mit_ * (model->normal(tc).to_homogeneous(0)));
      n = n.normalize();  // transform normal vector
      Vec3f l(m_ * (light_dir.to_homogeneous(0)));
      l = l.normalize();  // transform light direction
      Vec3f r = (n * (n * l * 2.) - l).normalize();  // reflected direction
      double spec = pow(std::max(.0, r.z), 1 + model->specular(tc)[0]);
      double diff =
          std::max(l * n, .0);  // intensity (notice that norms in
                                // obj file are from inside to outside)
      // shadow
      Vec3f pw = p_screen[0] * uv.x + p_screen[1] * uv.y + p_screen[2] * uv.z;
      Vec3f pshadow(pointify(m_shadow_ * (pw.to_homogeneous(1))));
      int idx = static_cast<int>(pshadow[0]) +
                static_cast<int>(pshadow[1]) * image->width();
      double shadow = .3 + .7 * (shadow_buffer[idx] < pshadow[2] + 5);
      //
      // ambient occlusion
      double total = 0;
      for (double a = 0; a < M_PI * 2 - 1e-4; a += M_PI / 4) {
        total += M_PI / 2 - max_elevation_angle(shadow_buffer, Vec2f(x, y),
                                                Vec2f(cos(a), sin(a)), *image);
      }
      total /= (M_PI / 2) * 8;
      total = pow(total, 100.f);
      //
      for (int i = 0; i < 3; i++)
        c[i] = std::min(total + c[i] * shadow * (1 * diff + 1.2 * spec), 255.);
      image->set(x, y, c);
    }
  }
}

void GL::render(Model *model, TGAImage *image, double *z_buffer,
                const Vec3f &light_dir, const MatrixW &sb_matrix,
                double *shadow_buffer) {
  invert_ = (view_port_ * projection_ * model_view_).invert();
  m_shadow_ = sb_matrix * invert_;
  m_ = model_view_;
  mit_ = m_.invert_transpose();
  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face = model->face(i);
    TriangleW triangle_w(model->vert(face[0]), model->vert(face[1]),
                         model->vert(face[2]));
    draw_triangle(triangle_w, i, image, z_buffer, light_dir, model,
                  shadow_buffer);
  }
}

void GL::shadow_draw(const TriangleW &triangle, TGAImage *image,
                     double *z_buffer) {
  TriangleS triangle_s;  // triangle on screen coordinates
  Vec3f z;               // zs of three vertices
  for (int i = 0; i < 3; i++) {
    Vec3f p = Vec3f(transform(triangle[i].to_homogeneous(1.)));
    triangle_s[i] = Vec2i(p.x, p.y);
    z[i] = p.z;
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
      double pz = 0;
      for (int i = 0; i < 3; i++) pz += uv[i] * z[i];  // average depth
      int index = x + y * image->width();
      if (z_buffer[index] >= pz) continue;
      z_buffer[index] = pz;
    }
  }
}

void GL::shadow_render(Model *model, TGAImage *image, double *z_buffer) {
  // m_ = projection_ * model_view_;
  m_ = model_view_;
  mit_ = m_.invert_transpose();
  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face = model->face(i);
    TriangleW triangle_w(model->vert(face[0]), model->vert(face[1]),
                         model->vert(face[2]));
    shadow_draw(triangle_w, image, z_buffer);
  }
}

MatrixW GL::transform_matrix() const {
  MatrixW temp = projection_ * model_view_;
  return view_port_ * temp;
}

}  // namespace gl
