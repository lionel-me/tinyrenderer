/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#include <iostream>
#include <limits>

#include "geometry.h"
#include "gl.h"
#include "model.h"
#include "tgaimage.h"

int main(int argc, char **argv) {
  // Matrix<double, 2, 2> temp;
  // temp[0][0] = 1;
  // temp[1][1] = 2;
  // std::cout << temp;
  // Matrix<double, 2, 2> reverse = temp.invert();
  // std::cout << reverse;
  Model *model;
  Model *model_eye_inner;
  Model *model_eye_outer;
  if (2 == argc) {
    model = new Model(argv[1]);
  } else {
    model = new Model("/root/tinyrenderer/obj/african_head/african_head.obj");
    model_eye_inner = new Model(
        "/root/tinyrenderer/obj/african_head/african_head_eye_inner.obj");
    model_eye_outer = new Model(
        "/root/tinyrenderer/obj/african_head/african_head_eye_outer.obj");
  }
  // Model *floor = new Model("/root/tinyrenderer/obj/floor.obj");
  int width = 800;
  int height = 800;
  Vec3f eye(0, 0, 5), center(0, 0, 0), up(0, 1, 0);
  // Vec3f light_dir = (eye - center).normalize();
  Vec3f light_dir = Vec3f(-5, 2, 5);
  double *z_buffer =
      new double[width * height]{std::numeric_limits<double>::min()};
  double *s_buffer =
      new double[width * height]{std::numeric_limits<double>::min()};
  TGAImage image(width, height, TGAImage::RGB);
  // shadow buffer
  gl::GL shadow_gl;
  shadow_gl.lookat(light_dir, center, up);
  shadow_gl.projection(light_dir, center);
  shadow_gl.viewport(100, 100, 600, 600);
  shadow_gl.shadow_render(model, &image, s_buffer);
  MatrixW m = shadow_gl.transform_matrix();
  //
  gl::GL our_gl;
  our_gl.lookat(eye, center, up);
  our_gl.projection(eye, center);
  our_gl.viewport(100, 100, 600, 600);
  our_gl.render(model, &image, z_buffer, light_dir, m, s_buffer);
  if (2 != argc) {
    our_gl.render(model_eye_inner, &image, z_buffer, light_dir, m, s_buffer);
    // our_gl.render(model_eye_outer, &image, z_buffer, light_dir);
  }
  // our_gl.render(floor, &image, z_buffer, light_dir);
  image.write_tga_file("output.tga");
  delete[] z_buffer;
  delete[] s_buffer;
  delete model;
  if (2 != argc) {
    delete model_eye_inner;
    delete model_eye_outer;
  }
  // delete floor;
  return 0;
}
