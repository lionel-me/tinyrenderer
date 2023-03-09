/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#include <limits>

#include "geometry.h"
#include "gl.h"
#include "global.h"
#include "model.h"
#include "tgaimage.h"

int main(int argc, char **argv) {
  if (2 == argc) {
    global::model = new Model(argv[1]);
  } else {
    global::model =
        new Model("/root/tinyrenderer/obj/african_head/african_head.obj");
  }
  int width = 800;
  int height = 800;
  Vec3f light_dir(0, 0, -1), eye(2, 0, 0), center(0, 0, 0), up(0, 1, 0);
  gl::GL our_gl;
  our_gl.lookat(eye, center, up);
  our_gl.projection(eye, center);
  our_gl.viewport(100, 100, 600, 600);
  int *z_buffer = new int[width * height]{std::numeric_limits<int>::min()};
  TGAImage image(width, height, TGAImage::RGB);
  for (int i = 0; i < global::model->nfaces(); i++) {
    std::vector<int> face = global::model->face(i);
    TriangleW triangle_w(global::model->vert(face[0]),
                         global::model->vert(face[1]),
                         global::model->vert(face[2]));
    our_gl.draw_triangle(triangle_w, i, &image, z_buffer, light_dir);
  }
  image.write_tga_file("output.tga");
  delete[] z_buffer;
  delete global::model;
  return 0;
}
