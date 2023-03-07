/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#include <limits>

#include "geometry.h"
#include "global.h"
#include "model.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);  // 白色
const TGAColor red = TGAColor(255, 0, 0, 255);        // 红色
const int kDepth = 1023;  // 三角形投影到像素坐标系时的默认深度

// the triangles are given in world coordinates between -1 and 1
void draw_triangle(const TriangleW &triangle, const Vec2i *uvs, TGAImage *image,
                   int *z_buffer, const float &intensity) {
  TriangleS triangle_s;
  Vec3i z;
  for (int i = 0; i < 3; i++) {
    triangle_s[i] = Vec2i((triangle[i].x + 1.) / 2 * image->width(),
                          (triangle[i].y + 1.) / 2 * image->height());
    z[i] = static_cast<int>((triangle[i].z + 1.) / 2 * kDepth);
  }
  int xmin = image->width() - 1, xmax = 0, ymin = image->height() - 1, ymax = 0;
  for (int i = 0; i < 3; i++) {
    Vec2i p = triangle_s[i];
    xmin = std::max(std::min(xmin, p[0]), 0);
    xmax = std::min(std::max(xmax, p[0]), image->width() - 1);
    ymin = std::max(std::min(ymin, p[1]), 0);
    ymax = std::min(std::max(ymax, p[1]), image->height() - 1);
  }
  for (int x = xmin; x <= xmax; x++) {
    for (int y = ymin; y <= ymax; y++) {
      Vec3f uv = triangle_s.barycentric(Vec2i(x, y));
      if (uv.x < 0 || uv.y < 0 || uv.z < 0) continue;
      int pz = 0;
      for (int i = 0; i < 3; i++) pz += uv[i] * z[i];
      int index = x + y * image->width();
      if (z_buffer[index] < pz) {
        z_buffer[index] = pz;
        Vec2i tc = uvs[0] * uv.x + uvs[1] * uv.y + uvs[2] * uv.z;
        TGAColor c = global::model->diffuse().get(tc.x, tc.y);
        image->set(x, y,
                   TGAColor(c[0] * intensity, c[1] * intensity,
                            c[2] * intensity, 255));
      }
    }
  }
}

int main(int argc, char **argv) {
  global::model =
      new Model("/root/tinyrenderer/obj/diablo3_pose/diablo3_pose.obj");
  int width = 2560;
  int height = 2560;
  Vec3f light_dir(0.0f, 0.0f, -1.0f);
  int *z_buffer = new int[width * height]{std::numeric_limits<int>::min()};
  TGAImage image(width, height, TGAImage::RGB);
  for (int i = 0; i < global::model->nfaces(); i++) {
    std::vector<int> face = global::model->face(i);
    TriangleW triangle_w(global::model->vert(face[0]),
                         global::model->vert(face[1]),
                         global::model->vert(face[2]));
    Vec3f n = triangle_w.normal();
    float intensity = n * light_dir;
    Vec2i uvs[3];
    for (int j = 0; j < 3; j++) {
      uvs[j] = global::model->uv(i, j);
    }
    if (intensity > 0.0f) {
      draw_triangle(triangle_w, uvs, &image, z_buffer, intensity);
    }
  }
  // image.flip_vertically();  // i want to have the origin at the left
  // bottom corner of the image
  image.write_tga_file("output.tga");
  delete[] z_buffer;
  delete global::model;
  return 0;
}
