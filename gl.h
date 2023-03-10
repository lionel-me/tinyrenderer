/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#ifndef GL_H_
#define GL_H_

#include <algorithm>

#include "geometry.h"
#include "model.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);  // 白色
const TGAColor red = TGAColor(255, 0, 0, 255);        // 红色
const int kDepth = 255;  // 三角形投影到像素坐标系时的默认深度

namespace gl {

class GL {
  MatrixW model_view_;
  MatrixW projection_;
  MatrixW view_port_;
  MatrixW m_;
  MatrixW mit_;
  MatrixW m_shadow_;
  MatrixW invert_;

 public:
  void lookat(const Vec3f &eye, const Vec3f &center, const Vec3f &up);
  void projection(const Vec3f &eye, const Vec3f &center);
  void viewport(const int &x, const int &y, const int &width,
                const int &height);
  vec4 transform(const vec4 &p);
  void draw_triangle(const TriangleW &triangle, const int &iface,
                     TGAImage *image, double *z_buffer, const Vec3f &light_dir,
                     Model *model, double *shadow_buffer);
  void shadow_draw(const TriangleW &triangle, TGAImage *image,
                   double *z_buffer);
  void render(Model *model, TGAImage *image, double *z_buffer,
              const Vec3f &light_dir, const MatrixW &sb_matrix,
              double *shadow_buffer);
  void shadow_render(Model *model, TGAImage *image, double *z_buffer);
  MatrixW transform_matrix() const;
};

}  // namespace gl

#endif  // GL_H_
