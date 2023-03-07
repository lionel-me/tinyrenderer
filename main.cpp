/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#include <limits>

#include "geometry.h"
#include "model.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);  // 白色
const TGAColor red = TGAColor(255, 0, 0, 255);        // 红色

int main(int argc, char** argv) {
  Model model(
      "/root/entertainment/tinyrenderer/obj/african_head/african_head.obj");
  int width = 1000;
  int height = 1000;
  Vec3f light_dir(0.0f, 0.0f, -1.0f);
  float* z_buffer =
      new float[width * height]{std::numeric_limits<float>::min()};
  TGAImage image(width, height, TGAImage::RGB);
  for (int i = 0; i < model.nfaces(); i++) {
    std::vector<int> face = model.face(i);
    TriangleW triangle_w(model.vert(face[0]), model.vert(face[1]),
                         model.vert(face[2]));
    Vec3f n = triangle_w.normal();
    float intensity = n * light_dir;
    if (intensity > 0.0f) {
      image.draw_triangle(
          triangle_w,
          TGAColor(intensity * 255, intensity * 255, intensity * 255, 255),
          z_buffer);
      // break;
    }
  }
  // image.flip_vertically();  // i want to have the origin at the left
  // bottom corner of the image
  image.write_tga_file("output.tga");
  delete[] z_buffer;
  return 0;
}
