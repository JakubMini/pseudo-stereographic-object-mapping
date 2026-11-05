#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace psom {

// World metres. Per-vertex confidence channels are kept apart on purpose: none of them alone is accuracy.
struct Mesh {
    std::vector<cv::Vec3f> v;
    std::vector<cv::Vec3i> f;
    std::vector<float> weight;     // observation weight
    std::vector<float> diversity;  // 0 = always seen from one direction, ->1 = seen from all around
    std::vector<float> sigma;      // fused depth sigma, metres (assumes independent noise per view)

    bool has_channels() const { return weight.size() == v.size(); }
    void compact();  // drop unreferenced vertices
};

bool write_ply(const std::filesystem::path& p, const Mesh& m);
bool read_ply(const std::filesystem::path& p, Mesh& m, std::string& err);

double signed_volume(const Mesh& m);

}  // namespace psom
