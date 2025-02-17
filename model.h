/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#ifndef MODEL_H_
#define MODEL_H_

#include <string>
#include <vector>

#include "geometry.h"
#include "tgaimage.h"

class Model {
  std::vector<Vec3f> verts{};      // array of vertices
  std::vector<Vec2f> tex_coord{};  // per-vertex array of tex coords
  std::vector<Vec3f> norms{};      // per-verteVec3fx array of normal vectors
  std::vector<int> facet_vrt{};
  std::vector<int> facet_tex{};  // per-triangle indices in the above arrays
  std::vector<int> facet_nrm{};
  TGAImage diffusemap{};   // diffuse color texture
  TGAImage normalmap{};    // normal map texture
  TGAImage specularmap{};  // specular map texture
  void load_texture(const std::string filename, const std::string suffix,
                    TGAImage *img);

 public:
  explicit Model(const std::string filename);
  int nverts() const;
  int nfaces() const;
  Vec3f normal(const int iface,
               const int nthvert) const;  // per triangle corner normal vertex

  Vec3f vert(const int i) const;
  Vec3f vert(const int iface, const int nthvert) const;
  std::vector<int> face(const int iface) const;
  Vec2f uvf(const int iface, const int nthvert) const;
  const TGAImage &diffuse() const { return diffusemap; }
  TGAColor diffuse(const Vec2f &uv) const;
  const TGAImage &normal() const { return normalmap; }
  Vec3f normal(const Vec2f &uv) const;
  const TGAImage &specular() const { return specularmap; }
  TGAColor specular(const Vec2f &uv) const;
};

#endif  // MODEL_H_
