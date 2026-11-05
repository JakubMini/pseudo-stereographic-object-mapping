#pragma once
#include <cstdint>
#include <filesystem>
#include <string>

#include <opencv2/core.hpp>

#include "psom/result.h"

namespace psom {

// A rendered scan of a box or cylinder on a textured table, written in the real session format.
struct SynthSpec {
    std::string shape = "box";          // box: dims L,W,H; cylinder: D,D,H
    cv::Vec3d dims_mm{100, 60, 40};
    bool depth = true;                  // write raw depth like an ARCore phone that supports it
    bool low_texture = false;           // plain object surface
    bool tracking_loss = false;         // a gap in the middle of the orbit
    bool interrupted = false;           // no capture_end, last line cut short
    int frames = 48;
    int width = 640, height = 480;
    double radius = 0.35;               // metres from the object
    double pose_noise_mm = 0;
    uint32_t seed = 1;
};

Result synth_project(const std::filesystem::path& project, const SynthSpec& spec);

}  // namespace psom
