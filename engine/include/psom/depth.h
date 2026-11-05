#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "psom/session.h"

namespace psom {

// Metric depth along the camera z axis, in metres, with a 1-sigma uncertainty per pixel.
// 0 means rejected or unknown.
struct DepthMap {
    enum class Source { None, Raw, Mvs };
    Intrinsics K;
    std::vector<float> d, sigma;
    Source source = Source::None;

    void resize(const Intrinsics& k) {
        K = k;
        d.assign(size_t(k.w) * k.h, 0.f);
        sigma.assign(size_t(k.w) * k.h, 0.f);
    }
    bool empty() const { return d.empty(); }
    size_t valid() const;
};

// ARCore raw depth for one frame. False (with a reason) when the frame's depth can't be trusted.
bool load_raw_depth(const Session& s, const Frame& f, DepthMap& out, std::string& why);

bool save_depth(const std::filesystem::path& p, const DepthMap& m);
bool load_depth(const std::filesystem::path& p, DepthMap& m);

}  // namespace psom
