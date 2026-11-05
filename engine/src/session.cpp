#include "psom/session.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace psom {

cv::Matx44d gl_to_cv(const cv::Matx44d& T) {
    const cv::Matx44d D(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);
    return T * D;
}

Intrinsics Intrinsics::scaled(int w2, int h2) const {
    const double sx = double(w2) / w, sy = double(h2) / h;
    // pixel centres sit at integer coordinates
    return {fx * sx, fy * sy, (cx + 0.5) * sx - 0.5, (cy + 0.5) * sy - 0.5, w2, h2};
}

std::string frame_stem(int i) {
    char b[16];
    std::snprintf(b, sizeof b, "%06d", i);
    return b;
}

fs::path Session::image_path(const Frame& f) const { return dir / "frames" / (frame_stem(f.i) + ".jpg"); }
fs::path Session::depth_path(const Frame& f) const { return dir / "depth" / (frame_stem(f.i) + ".png"); }
fs::path Session::conf_path(const Frame& f) const { return dir / "conf" / (frame_stem(f.i) + ".png"); }

namespace {

bool read_lines(const fs::path& p, std::vector<std::string>& lines, bool& ends_with_newline) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return false;
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string all = ss.str();
    ends_with_newline = !all.empty() && all.back() == '\n';
    std::string line;
    std::istringstream is(all);
    while (std::getline(is, line)) lines.push_back(line);
    return true;
}

bool valid_pose(const cv::Matx44d& T, std::string& why) {
    for (double x : T.val)
        if (!std::isfinite(x)) return why = "pose is not finite", false;
    if (std::abs(T(3, 0)) > 1e-6 || std::abs(T(3, 1)) > 1e-6 || std::abs(T(3, 2)) > 1e-6 ||
        std::abs(T(3, 3) - 1) > 1e-6)
        return why = "pose last row is not 0 0 0 1", false;
    const cv::Matx33d R = T.get_minor<3, 3>(0, 0);
    const cv::Matx33d E = R.t() * R - cv::Matx33d::eye();
    for (double x : E.val)
        if (std::abs(x) > 1e-3) return why = "pose rotation is not orthonormal", false;
    if (cv::determinant(R) < 0) return why = "pose rotation is a reflection", false;
    return true;
}

// Reads frame line `j` into `f`; false with a reason if the frame can't be trusted.
bool parse_frame(const json& j, const Session& s, Frame& f, bool& wants_depth, std::string& why) {
    if (!j.is_object()) return why = "not an object", false;
    if (!j.contains("i") || !j["i"].is_number_integer() || j["i"].get<int>() < 0) return why = "bad index", false;
    f.i = j["i"].get<int>();
    if (!j.contains("t_ns") || !j["t_ns"].is_number_integer()) return why = "missing t_ns", false;
    f.t_ns = j["t_ns"].get<int64_t>();
    if (!j.contains("pose") || !j["pose"].is_array() || j["pose"].size() != 16) return why = "pose needs 16 numbers", false;
    cv::Matx44d T;
    for (int k = 0; k < 16; ++k) {
        if (!j["pose"][k].is_number()) return why = "pose has a non-number", false;
        T.val[k] = j["pose"][k].get<double>();  // row-major
    }
    if (!valid_pose(T, why)) return false;
    f.T_wc = gl_to_cv(T);
    f.T_wc_depth = f.T_wc;
    if (j.contains("depth_pose")) {
        if (!j["depth_pose"].is_array() || j["depth_pose"].size() != 16) return why = "depth_pose needs 16 numbers", false;
        cv::Matx44d Td;
        for (int k = 0; k < 16; ++k) {
            if (!j["depth_pose"][k].is_number()) return why = "depth_pose has a non-number", false;
            Td.val[k] = j["depth_pose"][k].get<double>();
        }
        if (!valid_pose(Td, why)) return why = "depth_pose: " + why, false;
        f.T_wc_depth = gl_to_cv(Td);
        f.depth_has_pose = true;
    }
    if (!j.contains("K") || !j["K"].is_array() || j["K"].size() != 4) return why = "K needs 4 numbers", false;
    for (int k = 0; k < 4; ++k)
        if (!j["K"][k].is_number()) return why = "K has a non-number", false;
    f.K = {j["K"][0].get<double>(), j["K"][1].get<double>(), j["K"][2].get<double>(), j["K"][3].get<double>(),
           s.width, s.height};
    if (!(f.K.fx > 0) || !(f.K.fy > 0) || !std::isfinite(f.K.fx) || !std::isfinite(f.K.fy))
        return why = "focal length is not positive", false;
    if (!(f.K.cx >= 0 && f.K.cx < s.width && f.K.cy >= 0 && f.K.cy < s.height))
        return why = "principal point outside the image", false;
    if (j.value("tracking", std::string()) != "tracking") return why = "not tracking", false;
    f.sharpness = j.value("sharpness", 0.0);
    wants_depth = j.value("depth", false);
    f.depth_t_ns = j.contains("depth_t_ns") && j["depth_t_ns"].is_number_integer() ? j["depth_t_ns"].get<int64_t>() : 0;
    if (!fs::exists(s.image_path(f))) return why = "image file missing", false;
    return true;
}

}  // namespace

Result load_session(const fs::path& dir, Session& s) {
    Result r;
    r.stage = "load";
    s = Session{};
    s.dir = dir;

    std::ifstream sj(dir / "session.json");
    if (!sj) return Result::fail("load", "session.json not found in " + dir.string());
    json meta = json::parse(sj, nullptr, false);
    if (meta.is_discarded() || !meta.is_object()) return Result::fail("load", "session.json is not valid JSON");

    if (meta.value("format", std::string()) != "psom-session/1")
        return Result::fail("load", "unknown session format '" + meta.value("format", std::string()) + "'");
    const json conv = meta.value("conventions", json::object());
    // An unknown convention is refused rather than guessed.
    if (conv.value("pose", std::string()) != "arcore_gl_world_from_camera")
        return Result::fail("load", "unsupported pose convention");
    if (conv.value("depth_units", std::string()) != "mm") return Result::fail("load", "unsupported depth units");
    if (conv.value("image_orientation", std::string()) != "sensor")
        return Result::fail("load", "unsupported image orientation");

    const json img = meta.value("image", json::object());
    s.width = img.value("width", 0);
    s.height = img.value("height", 0);
    if (s.width <= 0 || s.height <= 0) return Result::fail("load", "image size missing");

    const json depth = meta.value("depth", json::object());
    if (depth.value("supported", false)) {
        s.depth_w = depth.value("width", 0);
        s.depth_h = depth.value("height", 0);
        s.max_skew_ns = depth.value("max_skew_ns", int64_t(16'666'666));
        const double a_img = double(s.width) / s.height;
        const double a_depth = s.depth_h > 0 ? double(s.depth_w) / s.depth_h : 0;
        if (s.depth_w <= 0 || s.depth_h <= 0)
            r.warn("raw depth declared but its size is missing; depth ignored");
        else if (std::abs(a_depth - a_img) / a_img > 0.01)
            r.warn("depth aspect ratio does not match the image; depth ignored (alignment check)");
        else
            s.depth_usable = true;
    }
    if (meta.contains("target") && meta["target"].is_array() && meta["target"].size() == 3) {
        cv::Vec3d t(meta["target"][0].get<double>(), meta["target"][1].get<double>(), meta["target"][2].get<double>());
        if (std::isfinite(t[0]) && std::isfinite(t[1]) && std::isfinite(t[2])) s.target = t;
    }

    std::vector<std::string> lines;
    bool newline = false;
    if (!read_lines(dir / "frames.jsonl", lines, newline)) return Result::fail("load", "frames.jsonl not found");
    std::map<std::string, int> drops;
    int64_t last_t = INT64_MIN;
    for (size_t li = 0; li < lines.size(); ++li) {
        if (lines[li].empty()) continue;
        const bool last = li + 1 == lines.size();
        json j = json::parse(lines[li], nullptr, false);
        if (j.is_discarded()) {
            if (last && !newline)
                r.warn("dropped a truncated last frame line (capture was cut short)");
            else
                ++drops["not valid JSON"];
            ++s.frames_dropped;
            continue;
        }
        Frame f;
        bool wants_depth = false;
        std::string why;
        if (!parse_frame(j, s, f, wants_depth, why)) {
            ++drops[why];
            ++s.frames_dropped;
            continue;
        }
        if (f.t_ns <= last_t) {
            ++drops["timestamp not increasing"];
            ++s.frames_dropped;
            continue;
        }
        last_t = f.t_ns;
        if (wants_depth && s.depth_usable) {
            // With its own pose, late depth is fine (within half a second); without one it must
            // belong to this very frame.
            const int64_t limit = f.depth_has_pose ? 500'000'000 : s.max_skew_ns;
            if (f.depth_t_ns == 0 || std::llabs(f.depth_t_ns - f.t_ns) > limit)
                ++s.depth_dropped_skew;
            else
                f.has_depth = fs::exists(s.depth_path(f)) && fs::exists(s.conf_path(f));
        }
        s.frames.push_back(f);
    }
    for (auto& [why, n] : drops) r.warn("dropped " + std::to_string(n) + " frame(s): " + why);
    if (s.depth_dropped_skew)
        r.warn("depth of " + std::to_string(s.depth_dropped_skew) +
               " frame(s) ignored: depth timestamp too far from the camera timestamp");

    // events.jsonl is advisory: unreadable lines are skipped.
    std::vector<std::string> ev;
    bool evnl = false;
    int64_t lost_at = -1;
    std::string lost_reason;
    if (read_lines(dir / "events.jsonl", ev, evnl)) {
        for (auto& l : ev) {
            json e = json::parse(l, nullptr, false);
            if (e.is_discarded() || !e.is_object()) continue;
            const std::string kind = e.value("event", std::string());
            const int64_t t = e.value("t_ns", int64_t(0));
            if (kind == "tracking_lost" && lost_at < 0) {
                lost_at = t;
                lost_reason = e.value("reason", std::string("unknown"));
            } else if (kind == "tracking_resumed" && lost_at >= 0) {
                s.tracking_loss.push_back({lost_at, t, lost_reason});
                lost_at = -1;
            } else if (kind == "capture_end") {
                s.capture_ended = true;
                if (e.contains("rejected") && e["rejected"].is_object())
                    for (auto& [k, v] : e["rejected"].items())
                        if (v.is_number_integer()) s.rejected[k] = v.get<int>();
            }
        }
    }
    if (lost_at >= 0) s.tracking_loss.push_back({lost_at, s.frames.empty() ? lost_at : s.frames.back().t_ns, lost_reason});
    if (!s.capture_ended) r.warn("capture was interrupted (no capture_end); using the frames that were saved");

    std::ifstream pc(dir / "points.csv");
    std::string row;
    while (std::getline(pc, row)) {
        long long t, id;
        double x, y, z, c;
        if (std::sscanf(row.c_str(), "%lld,%lld,%lf,%lf,%lf,%lf", &t, &id, &x, &y, &z, &c) != 6) continue;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) s.points.push_back({{x, y, z}, c});
    }

    if (s.frames.size() < 3)
        return Result::fail("load", "Only " + std::to_string(s.frames.size()) + " usable frame" + (s.frames.size() == 1 ? "" : "s") +
                                        ". At least 3 are needed.");
    r.ok = true;
    return r;
}

}  // namespace psom
