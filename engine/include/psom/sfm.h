#pragma once
#include <vector>

#include <opencv2/core.hpp>

#include "psom/result.h"
#include "psom/session.h"

namespace psom {

// Structure from motion: where each photo was taken, worked out from the photos alone.
struct SfmParams {
    int max_side = 1024;       // features are found on images scaled so the long side is at most this
    int max_features = 3000;
    double contrast = 0.02;    // SIFT contrast threshold; OpenCV's 0.04 starves low-contrast scenes (COLMAP's equivalent is ~0.02)
    bool equalize = true;      // local contrast equalisation first: a dim room still has texture
    int window = 10;           // each photo is matched with this many neighbours on either side (wrapping)
    double ratio = 0.8;        // Lowe's ratio test
    int min_inliers = 20;      // matches that survive the epipolar check for a pair to count (COLMAP: 15)
    double epipolar_px = 1.5;
    double pnp_px = 4.0;
    double max_reproj_px = 4.0;
    double min_angle_deg = 1.5;  // triangulation angle below which a point says nothing about depth
    double ba_growth = 1.15;     // everything is re-adjusted each time the model grows by this factor
    bool refine_focal = true;    // off when the focal length is a factory calibration rather than a guess
    bool verbose = false;        // JSON lines on stderr: features, pairs, placements
};

struct SfmResult {
    std::vector<int> frames;           // indices into Session::frames that were placed
    std::vector<cv::Matx44d> T_wc;     // world-from-camera, vision convention, one per placed frame
    Intrinsics K;                      // refined focal length, shared by all frames
    std::vector<cv::Vec3d> points;     // triangulated scene points
    double rms_px = 0;
    double f_guess = 0;
};

// Poses for the session's photos, in an arbitrary world (unit scale, arbitrary rotation).
Result run_sfm(const Session& s, const SfmParams& p, SfmResult& out, const Progress& progress = {});

// Bundle adjustment: cameras (x_cam = R X + t), points and optionally one shared focal length.
// obs are (camera, point, pixel). Camera 0 stays fixed. Returns the final RMS reprojection error in
// pixels; `inlier` marks observations within max_px after the last iteration.
struct BaObs {
    int cam, pt;
    cv::Vec2d uv;
};
double bundle_adjust(std::vector<cv::Matx33d>& R, std::vector<cv::Vec3d>& t, std::vector<cv::Vec3d>& X, Intrinsics& K,
                     const std::vector<BaObs>& obs, bool refine_focal, int iterations, double max_px,
                     std::vector<char>* inlier = nullptr);

// Up in a camera's frame from the phone's gravity sensor (device axes, pointing up), given the
// camera sensor's orientation relative to the device's natural orientation (back camera).
cv::Vec3d device_up_to_camera(const cv::Vec3d& g_device, int sensor_orientation_deg);

// Turns an SfM result into the world the rest of the engine expects: +y up (from gravity when the
// frames carry it, else from the cameras' orbit), and a working scale with the cameras about
// `distance` metres from what they look at. The scale is assumed, not measured. Writes the poses,
// intrinsics and points into `s` and drops frames that weren't placed.
Result apply_sfm(Session& s, const SfmResult& r, double distance = 0.3);

}  // namespace psom
