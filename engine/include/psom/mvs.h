#pragma once
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "psom/depth.h"
#include "psom/session.h"

namespace psom {

struct MvsParams {
    int width = 640;            // images are downscaled to this width
    int planes = 96;            // inverse-depth hypotheses
    int neighbours = 4;
    int best_k = 2;             // costs aggregated per pixel; occluded views drop out
    int window = 7;             // ZNCC window
    // ...for photos shrunk 2x or more to `width` (phone photos): measured on a real scan, 7 left twice
    // the holes of 11 on a glossy cover and plain cloth; native-size images keep sharper edges at 7
    int window_shrunk = 11;
    double min_zncc = 0.5;
    double min_texture = 2.0;   // grey-level std in the window, after contrast equalisation
    double max_rel_sigma = 0.02;
    double consist_rel = 0.02;  // depth agreement, relative
    double consist_px = 2.0;    // reprojection agreement, pixels
    int min_consistent = 1;     // independent views that must agree; fusion averages the rest
};

struct MvsView {
    Intrinsics K;
    cv::Mat gray;  // CV_32F, 0..255
    cv::Matx44d T_wc;
};

MvsView make_view(const cv::Mat& gray8, const Intrinsics& K, const cv::Matx44d& T_wc, int width);

// Overlap-aware: shared visible feature points x baseline-angle prior x sharpness.
std::vector<int> select_neighbours(const Session& s, int ref, const cv::Vec3d& center, int k);

// Near and far depth for a reference frame, from feature points in view, else from the scene centre.
std::pair<double, double> depth_range(const Session& s, const Frame& f, const cv::Vec3d& center);

DepthMap plane_sweep(const MvsView& ref, const std::vector<const MvsView*>& nbrs, double near, double far,
                     const MvsParams& p);

// Keeps a pixel only if at least p.min_consistent neighbours agree in depth and reprojection.
// maps[i] pairs with poses[i]; nbrs[i] are indices into maps. Only Mvs maps are filtered.
void consistency_filter(std::vector<DepthMap>& maps, const std::vector<cv::Matx44d>& poses,
                        const std::vector<std::vector<int>>& nbrs, const MvsParams& p);

}  // namespace psom
