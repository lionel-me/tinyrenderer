/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#ifndef OUR_GL_H_
#define OUR_GL_H_

#include <vector>

#include "geometry.h"
#include "tgaimage.h"

void viewport(const int x, const int y, const int w, const int h);
void projection(const double coeff = 0);  // coeff = -1/c
void lookat(const vec3 &eye, const vec3 &center, const vec3 &up);

struct IShader {
  static TGAColor sample2D(const TGAImage &img, const vec2 &uvf) {
    return img.get(uvf[0] * img.width(), uvf[1] * img.height());
  }
  virtual bool fragment(const vec3 &bar, const TGAColor &color) = 0;
};

void triangle(const vec4 clip_verts[3], const IShader &shader,
              const TGAImage &image, const std::vector<double> &zbuffer);

#endif  // OUR_GL_H_
