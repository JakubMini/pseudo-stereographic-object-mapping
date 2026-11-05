#pragma once
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "psom/result.h"

namespace psom {

struct Intrinsics {
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int w = 0, h = 0;
    Intrinsics scaled(int w2, int h2) const;
    cv::Vec2d project(const cv::Vec3d& Xc) const { return {fx * Xc[0] / Xc[2] + cx, fy * Xc[1] / Xc[2] + cy}; }
    cv::Vec3d ray(double u, double v) const { return {(u - cx) / fx, (v - cy) / fy, 1.0}; }
};

// A keyframe. T_wc is world-from-camera in the vision convention (+x right, +y down, +z forward).
struct Frame {
    int i = 0;
    int64_t t_ns = 0;
    cv::Matx44d T_wc = cv::Matx44d::eye();
    Intrinsics K;
    double sharpness = 0;
    bool has_depth = false;
    int64_t depth_t_ns = 0;
    // Raw depth arrives a frame or two late; this is the pose of the camera frame it belongs to.
    cv::Matx44d T_wc_depth = cv::Matx44d::eye();
    bool depth_has_pose = false;
    // Up, from the phone's gravity sensor, in device axes (photo sessions).
    cv::Vec3d gravity{0, 0, 0};
    bool has_gravity = false;

    cv::Matx33d R() const { return T_wc.get_minor<3, 3>(0, 0); }
    cv::Vec3d center() const { return {T_wc(0, 3), T_wc(1, 3), T_wc(2, 3)}; }
    cv::Vec3d forward() const { return {T_wc(0, 2), T_wc(1, 2), T_wc(2, 2)}; }
    cv::Vec3d to_cam(const cv::Vec3d& Xw) const { return R().t() * (Xw - center()); }
    cv::Vec3d to_world(const cv::Vec3d& Xc) const { return R() * Xc + center(); }
};

struct FeaturePoint {
    cv::Vec3d X;
    double confidence = 0;
};

struct Interval {
    int64_t from_ns = 0, to_ns = 0;
    std::string reason;
};

struct Session {
    std::filesystem::path dir;
    int width = 0, height = 0;
    bool depth_usable = false;  // phone supported raw depth and its geometry checks passed
    int depth_w = 0, depth_h = 0;
    int64_t max_skew_ns = 0;
    std::optional<cv::Vec3d> target;  // hit-test hint, may be absent
    std::vector<Frame> frames;
    std::vector<FeaturePoint> points;
    std::vector<Interval> tracking_loss;
    bool capture_ended = false;
    std::map<std::string, int> rejected;  // keyframes the app rejected, by reason
    int depth_dropped_skew = 0;
    int frames_dropped = 0;
    int tracking_jumps = 0;  // times the pose jumped faster than a hand moves
    // Photos only: poses come from the images (sfm.h), not the phone. Until then frames hold identity.
    bool photos = false;
    bool scale_assumed = false;  // true once poses came from photos: sizes are a guess until calibrated
    int sensor_orientation = 90;
    bool focal_calibrated = false;  // camera.intrinsics == "calibrated": the focal length is trusted as is

    std::filesystem::path image_path(const Frame& f) const;
    std::filesystem::path depth_path(const Frame& f) const;
    std::filesystem::path conf_path(const Frame& f) const;
};

// Loads and validates <dir>/session.json, frames.jsonl, events.jsonl and points.csv. Invalid
// frames are dropped with a warning naming the reason; the session fails below 3 valid frames.
Result load_session(const std::filesystem::path& dir, Session& out);

// ARCore's camera convention (+y up, looking down -z) to the vision one. Its own inverse.
cv::Matx44d gl_to_cv(const cv::Matx44d& T);

std::string frame_stem(int i);

}  // namespace psom
