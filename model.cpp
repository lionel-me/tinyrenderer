/**
 *
 * @copyright Copyright (c) 2023 lionel-me
 *
 */
#include "model.h"

#include <iostream>
#include <sstream>

Model::Model(const std::string filename) {
  std::ifstream in;
  in.open(filename, std::ifstream::in);
  if (in.fail()) return;
  std::string line;
  while (!in.eof()) {
    std::getline(in, line);
    std::istringstream iss(line.c_str());
    char trash;
    if (!line.compare(0, 2, "v ")) {
      iss >> trash;
      Vec3f v;
      for (int i = 0; i < 3; i++) iss >> v[i];
      verts.push_back(v);
    } else if (!line.compare(0, 3, "vn ")) {
      iss >> trash >> trash;
      Vec3f n;
      for (int i = 0; i < 3; i++) iss >> n[i];
      norms.push_back(n.normalize());
    } else if (!line.compare(0, 3, "vt ")) {
      iss >> trash >> trash;
      Vec2f uv;
      for (int i = 0; i < 2; i++) iss >> uv[i];
      tex_coord.push_back({uv.x, 1 - uv.y});
    } else if (!line.compare(0, 2, "f ")) {
      int f, t, n;
      iss >> trash;
      int cnt = 0;
      while (iss >> f >> trash >> t >> trash >> n) {
        facet_vrt.push_back(--f);
        facet_tex.push_back(--t);
        facet_nrm.push_back(--n);
        cnt++;
      }
      if (3 != cnt) {
        std::cerr << "Error: the obj file is supposed to be triangulated"
                  << std::endl;
        return;
      }
    }
  }
  std::cerr << "# v# " << nverts() << " f# " << nfaces() << " vt# "
            << tex_coord.size() << " vn# " << norms.size() << std::endl;
  load_texture(filename, "_diffuse.tga", &diffusemap);
  load_texture(filename, "_nm.tga", &normalmap);
  load_texture(filename, "_spec.tga", &specularmap);
}

int Model::nverts() const { return verts.size(); }

int Model::nfaces() const { return facet_vrt.size() / 3; }

Vec3f Model::vert(const int i) const { return verts[i]; }

Vec3f Model::vert(const int iface, const int nthvert) const {
  return verts[facet_vrt[iface * 3 + nthvert]];
}

std::vector<int> Model::face(const int iface) const {
  return std::vector<int>(facet_vrt.begin() + iface * 3,
                          facet_vrt.begin() + (iface + 1) * 3);
}

void Model::load_texture(const std::string filename, const std::string suffix,
                         TGAImage *img) {
  size_t dot = filename.find_last_of(".");
  if (dot == std::string::npos) return;
  std::string texfile = filename.substr(0, dot) + suffix;
  std::cerr << "texture file " << texfile << " loading "
            << (img->read_tga_file(texfile.c_str()) ? "ok" : "failed")
            << std::endl;
}

Vec3f Model::normal(const int iface, const int nthvert) const {
  return norms[facet_nrm[iface * 3 + nthvert]];
}

// texture
Vec2f Model::uvf(const int iface, const int nthvert) const {
  Vec2f coord = tex_coord[facet_tex[iface * 3 + nthvert]];
  return coord;
}

Vec3f Model::normal(const Vec2f &uvf) const {
  TGAColor c =
      normalmap.get(uvf[0] * normalmap.width(), uvf[1] * normalmap.height());
  return Vec3f{static_cast<double>(c[2]), static_cast<double>(c[1]),  // NOLINT
               static_cast<double>(c[0])} *
             2. / 255. -
         Vec3f{1, 1, 1};
}

TGAColor Model::diffuse(const Vec2f &uvf) const {
  return diffusemap.get(static_cast<int>(uvf.x * diffusemap.width()),
                        static_cast<int>(uvf.y * diffusemap.height()));
}

TGAColor Model::specular(const Vec2f &uvf) const {
  return specularmap.get(static_cast<int>(uvf.x * specularmap.width()),
                         static_cast<int>(uvf.y * specularmap.height()));
}
