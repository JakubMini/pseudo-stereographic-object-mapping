#pragma once
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "psom/depth.h"
#include "psom/result.h"
#include "psom/session.h"

namespace psom {

struct Plane {
    cv::Vec3d n{0, 1, 0};  // unit, pointing up (towards the cameras)
    double d = 0;          // n.X = d on the plane
    bool found = false;
    double height(const cv::Vec3d& X) const { return n.dot(X) - d; }
};

// Oriented box in world metres. axes rows are the box's unit x, y, z; z is the support-plane normal.
struct CropBox {
    cv::Vec3d center;
    cv::Matx33d axes = cv::Matx33d::eye();
    cv::Vec3d half{0.1, 0.1, 0.1};
    std::string source = "auto";

    cv::Vec3d to_local(const cv::Vec3d& X) const { return axes * (X - center); }
    cv::Vec3d to_world(const cv::Vec3d& L) const { return axes.t() * L + center; }
    bool contains(const cv::Vec3d& X) const {
        cv::Vec3d L = to_local(X);
        return std::abs(L[0]) <= half[0] && std::abs(L[1]) <= half[1] && std::abs(L[2]) <= half[2];
    }
};

// Least-squares point closest to all camera axes (plus the hit-test hint if any).
cv::Vec3d scene_center(const Session& s);

// World points from depth maps (every `step`th pixel) with sigma below max_sigma.
std::vector<cv::Vec3d> depth_points(const Session& s, const std::vector<DepthMap>& maps, int step, double max_sigma);

// RANSAC for a near-horizontal plane (world +y is up in ARCore) below the scene centre.
Plane find_support_plane(const std::vector<cv::Vec3d>& pts, const cv::Vec3d& center, double radius);

// Support plane plus the largest cluster above it near the scene centre.
Result auto_crop(const Session& s, const std::vector<DepthMap>& maps, CropBox& box, Plane& plane);

// Minimum-area rectangle of 2D points (rotating calipers over the convex hull). `axis` is the unit
// direction of the longer side; OpenCV moved its version between 4 and 5, so this one is ours.
struct Rect2 {
    cv::Vec2d axis{1, 0};
    double length = 0, width = 0;
    cv::Vec2d center{0, 0};
};
Rect2 min_area_rect(const std::vector<cv::Vec2d>& pts);

// Height above the support plane where the crop box starts; keeps the table out of the TSDF.
constexpr double kCropFloor = 0.003;

}  // namespace psom
