#include "psom/mesh.h"

#include <cstring>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace psom {

void Mesh::compact() {
    std::vector<int> remap(v.size(), -1);
    int n = 0;
    for (auto& t : f)
        for (int k = 0; k < 3; ++k)
            if (remap[t[k]] < 0) remap[t[k]] = n++;
    auto pack = [&](auto& vec) {
        if (vec.size() != remap.size()) return;
        std::remove_reference_t<decltype(vec)> out(n);
        for (size_t i = 0; i < remap.size(); ++i)
            if (remap[i] >= 0) out[remap[i]] = vec[i];
        vec.swap(out);
    };
    pack(weight);
    pack(diversity);
    pack(sigma);
    pack(v);
    for (auto& t : f)
        for (int k = 0; k < 3; ++k) t[k] = remap[t[k]];
}

double signed_volume(const Mesh& m) {
    double vol = 0;
    for (auto& t : m.f) {
        const cv::Vec3d a = m.v[t[0]], b = m.v[t[1]], c = m.v[t[2]];
        vol += a.dot(b.cross(c));
    }
    return vol / 6.0;
}

// Binary little-endian PLY: x y z weight diversity sigma (float), faces as uchar + int list.
bool write_ply(const fs::path& p, const Mesh& m) {
    const fs::path tmp = p.string() + ".tmp";
    {
        std::ofstream o(tmp, std::ios::binary);
        if (!o) return false;
        o << "ply\nformat binary_little_endian 1.0\ncomment psom mesh, world metres\n"
          << "element vertex " << m.v.size() << "\n"
          << "property float x\nproperty float y\nproperty float z\n"
          << "property float weight\nproperty float diversity\nproperty float sigma\n"
          << "element face " << m.f.size() << "\nproperty list uchar int vertex_indices\nend_header\n";
        const bool ch = m.has_channels();
        for (size_t i = 0; i < m.v.size(); ++i) {
            const float rec[6] = {m.v[i][0], m.v[i][1], m.v[i][2], ch ? m.weight[i] : 0.f,
                                  ch ? m.diversity[i] : 0.f, ch ? m.sigma[i] : 0.f};
            o.write(reinterpret_cast<const char*>(rec), sizeof rec);
        }
        for (auto& t : m.f) {
            const uint8_t n = 3;
            o.write(reinterpret_cast<const char*>(&n), 1);
            o.write(reinterpret_cast<const char*>(&t[0]), 3 * sizeof(int));
        }
        if (!o) return false;
    }
    std::error_code ec;
    fs::rename(tmp, p, ec);
    return !ec;
}

// Reads only the layout write_ply produces.
bool read_ply(const fs::path& p, Mesh& m, std::string& err) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return err = "cannot open " + p.string(), false;
    std::string line;
    size_t nv = 0, nf = 0;
    int vprops = 0;
    bool binary = false;
    while (std::getline(in, line)) {
        if (line.rfind("format binary_little_endian", 0) == 0) binary = true;
        if (line.rfind("element vertex ", 0) == 0) nv = std::stoull(line.substr(15));
        if (line.rfind("element face ", 0) == 0) nf = std::stoull(line.substr(13));
        if (line.rfind("property float ", 0) == 0) ++vprops;
        if (line == "end_header") break;
    }
    if (!binary || vprops != 6) return err = "unsupported PLY layout", false;
    if (nv > 50'000'000 || nf > 100'000'000) return err = "PLY too large", false;
    m = Mesh{};
    m.v.resize(nv);
    m.weight.resize(nv);
    m.diversity.resize(nv);
    m.sigma.resize(nv);
    for (size_t i = 0; i < nv; ++i) {
        float rec[6];
        if (!in.read(reinterpret_cast<char*>(rec), sizeof rec)) return err = "PLY truncated", false;
        m.v[i] = {rec[0], rec[1], rec[2]};
        m.weight[i] = rec[3];
        m.diversity[i] = rec[4];
        m.sigma[i] = rec[5];
    }
    m.f.resize(nf);
    for (size_t i = 0; i < nf; ++i) {
        uint8_t n = 0;
        int idx[3];
        if (!in.read(reinterpret_cast<char*>(&n), 1) || n != 3 || !in.read(reinterpret_cast<char*>(idx), sizeof idx))
            return err = "PLY faces truncated or not triangles", false;
        for (int k = 0; k < 3; ++k)
            if (idx[k] < 0 || size_t(idx[k]) >= nv) return err = "PLY face index out of range", false;
        m.f[i] = {idx[0], idx[1], idx[2]};
    }
    return true;
}

}  // namespace psom
