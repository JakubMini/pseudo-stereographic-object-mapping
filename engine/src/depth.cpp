#include "psom/depth.h"

#include <cstring>
#include <fstream>

#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

namespace psom {

size_t DepthMap::valid() const {
    size_t n = 0;
    for (float x : d) n += x > 0;
    return n;
}

bool load_raw_depth(const Session& s, const Frame& f, DepthMap& out, std::string& why) {
    if (!f.has_depth) return why = "no depth for this frame", false;
    cv::Mat d = cv::imread(s.depth_path(f).string(), cv::IMREAD_UNCHANGED);
    cv::Mat c = cv::imread(s.conf_path(f).string(), cv::IMREAD_UNCHANGED);
    if (d.empty() || d.type() != CV_16U) return why = "depth is not a 16-bit PNG", false;
    if (c.empty() || c.type() != CV_8U) return why = "confidence is not an 8-bit PNG", false;
    if (d.cols != s.depth_w || d.rows != s.depth_h || c.size() != d.size())
        return why = "depth size differs from session.json", false;
    out.resize(f.K.scaled(d.cols, d.rows));
    out.source = DepthMap::Source::Raw;
    for (int y = 0; y < d.rows; ++y)
        for (int x = 0; x < d.cols; ++x) {
            const double z = d.at<uint16_t>(y, x) * 1e-3;
            const int conf = c.at<uint8_t>(y, x);
            if (z < 0.05 || z > 3.0 || conf < 64) continue;
            const size_t k = size_t(y) * d.cols + x;
            out.d[k] = float(z);
            // shortcut: ARCore gives confidence, not sigma; this z^2 model is a guess, replace it with
            // one fitted to the real-device sessions once they exist.
            out.sigma[k] = float(0.001 + 0.004 * z * z * (255.0 / conf));
        }
    return true;
}

namespace {
constexpr uint32_t kMagic = 0x50445350;  // "PSDP"
}

bool save_depth(const fs::path& p, const DepthMap& m) {
    const fs::path tmp = p.string() + ".tmp";
    {
        std::ofstream o(tmp, std::ios::binary);
        if (!o) return false;
        const uint32_t hdr[4] = {kMagic, 1, uint32_t(m.K.w), uint32_t(m.K.h)};
        const float k[4] = {float(m.K.fx), float(m.K.fy), float(m.K.cx), float(m.K.cy)};
        const uint32_t src = uint32_t(m.source);
        o.write(reinterpret_cast<const char*>(hdr), sizeof hdr);
        o.write(reinterpret_cast<const char*>(k), sizeof k);
        o.write(reinterpret_cast<const char*>(&src), sizeof src);
        o.write(reinterpret_cast<const char*>(m.d.data()), m.d.size() * sizeof(float));
        o.write(reinterpret_cast<const char*>(m.sigma.data()), m.sigma.size() * sizeof(float));
        if (!o) return false;
    }
    std::error_code ec;
    fs::rename(tmp, p, ec);
    return !ec;
}

bool load_depth(const fs::path& p, DepthMap& m) {
    std::ifstream in(p, std::ios::binary);
    uint32_t hdr[4];
    float k[4];
    uint32_t src = 0;
    if (!in.read(reinterpret_cast<char*>(hdr), sizeof hdr) || hdr[0] != kMagic || hdr[1] != 1) return false;
    if (hdr[2] == 0 || hdr[3] == 0 || hdr[2] > 8192 || hdr[3] > 8192) return false;
    if (!in.read(reinterpret_cast<char*>(k), sizeof k) || !in.read(reinterpret_cast<char*>(&src), sizeof src)) return false;
    m.resize({k[0], k[1], k[2], k[3], int(hdr[2]), int(hdr[3])});
    m.source = DepthMap::Source(src);
    return bool(in.read(reinterpret_cast<char*>(m.d.data()), m.d.size() * sizeof(float))) &&
           bool(in.read(reinterpret_cast<char*>(m.sigma.data()), m.sigma.size() * sizeof(float)));
}

}  // namespace psom
