#pragma once
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "psom/mesh.h"
#include "psom/synth.h"

namespace fs = std::filesystem;

inline fs::path test_root() {
    static const fs::path root = [] {
        fs::path p = fs::temp_directory_path() / ("psom_tests_" + std::to_string(getpid()));
        fs::remove_all(p);
        fs::create_directories(p);
        return p;
    }();
    return root;
}

// A fresh copy of a synthetic project; the rendering is cached per spec name.
inline fs::path synth_copy(const std::string& name, const psom::SynthSpec& spec, const std::string& copy) {
    const fs::path src = test_root() / ("src_" + name);
    if (!fs::exists(src / "project.json")) psom::synth_project(src, spec);
    const fs::path dst = test_root() / copy;
    fs::remove_all(dst);
    fs::copy(src, dst, fs::copy_options::recursive);
    return dst;
}

inline std::vector<std::string> read_lines(const fs::path& p) {
    std::ifstream in(p);
    std::vector<std::string> out;
    for (std::string l; std::getline(in, l);) out.push_back(l);
    return out;
}

inline void write_lines(const fs::path& p, const std::vector<std::string>& lines) {
    std::ofstream o(p);
    for (auto& l : lines) o << l << "\n";
}

inline std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Every file under a directory with its contents, to prove nothing changed.
inline std::string tree_digest(const fs::path& dir) {
    std::vector<std::string> parts;
    for (auto& e : fs::recursive_directory_iterator(dir))
        if (e.is_regular_file())
            parts.push_back(e.path().string() + ":" + std::to_string(std::hash<std::string>{}(slurp(e.path()))));
    std::sort(parts.begin(), parts.end());
    std::string all;
    for (auto& p : parts) all += p + "\n";
    return all;
}

// Axis-aligned cube [x0,x0+s]^3 with outward faces; `skip` names faces to leave out ("bottom", "top").
inline psom::Mesh cube(double s, cv::Vec3d o = {0, 0, 0}, std::vector<std::string> skip = {}) {
    psom::Mesh m;
    for (int i = 0; i < 8; ++i)
        m.v.emplace_back(float(o[0] + s * (i & 1)), float(o[1] + s * ((i >> 1) & 1)), float(o[2] + s * ((i >> 2) & 1)));
    // y is up; each face as two triangles, counter-clockwise from outside
    const std::vector<std::pair<std::string, std::vector<cv::Vec3i>>> faces = {
        {"bottom", {{0, 1, 5}, {0, 5, 4}}}, {"top", {{2, 6, 7}, {2, 7, 3}}},
        {"x0", {{0, 4, 6}, {0, 6, 2}}},     {"x1", {{1, 3, 7}, {1, 7, 5}}},
        {"z0", {{0, 2, 3}, {0, 3, 1}}},     {"z1", {{4, 5, 7}, {4, 7, 6}}},
    };
    for (auto& [name, tris] : faces)
        if (std::find(skip.begin(), skip.end(), name) == skip.end())
            for (auto& t : tris) m.f.push_back(t);
    return m;
}
