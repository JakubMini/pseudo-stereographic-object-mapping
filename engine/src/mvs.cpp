#include "psom/mvs.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace psom {

namespace {

cv::Matx33d kmat(const Intrinsics& K) { return {K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1}; }

bool visible(const Frame& f, const cv::Vec3d& X) {
    const cv::Vec3d Xc = f.to_cam(X);
    if (Xc[2] < 0.05) return false;
    const cv::Vec2d uv = f.K.project(Xc);
    return uv[0] >= 0 && uv[1] >= 0 && uv[0] < f.K.w && uv[1] < f.K.h;
}

double angle_deg(const cv::Vec3d& a, const cv::Vec3d& b) {
    const double c = a.dot(b) / (cv::norm(a) * cv::norm(b) + 1e-12);
    return std::acos(std::clamp(c, -1.0, 1.0)) * 180.0 / CV_PI;
}

// Points near the object only, so a far background doesn't stretch the depth range or the overlap.
std::vector<cv::Vec3d> near_points(const Session& s, const Frame& f, const cv::Vec3d& center) {
    const double radius = std::max(0.25, 0.6 * cv::norm(f.center() - center));
    std::vector<cv::Vec3d> out;
    for (auto& p : s.points)
        if (cv::norm(p.X - center) < radius) out.push_back(p.X);
    return out;
}

}  // namespace

MvsView make_view(const cv::Mat& gray8, const Intrinsics& K, const cv::Matx44d& T_wc, int width) {
    MvsView v;
    const int w = std::min(width, gray8.cols);
    const int h = int(std::lround(double(gray8.rows) * w / gray8.cols));
    cv::Mat small;
    cv::resize(gray8, small, {w, h}, 0, 0, cv::INTER_AREA);
    small.convertTo(v.gray, CV_32F);
    v.K = K.scaled(w, h);
    v.T_wc = T_wc;
    return v;
}

std::vector<int> select_neighbours(const Session& s, int ref, const cv::Vec3d& center, int k) {
    const Frame& r = s.frames[ref];
    const auto pts = near_points(s, r, center);
    std::vector<const cv::Vec3d*> vis_ref;
    for (auto& X : pts)
        if (visible(r, X)) vis_ref.push_back(&X);

    std::vector<double> sharp;
    for (auto& f : s.frames)
        if (f.sharpness > 0) sharp.push_back(f.sharpness);
    double med = 0;
    if (!sharp.empty()) {
        std::nth_element(sharp.begin(), sharp.begin() + sharp.size() / 2, sharp.end());
        med = sharp[sharp.size() / 2];
    }

    std::vector<std::pair<double, int>> scored;
    for (int n = 0; n < int(s.frames.size()); ++n) {
        if (n == ref) continue;
        const Frame& f = s.frames[n];
        const double theta = angle_deg(r.center() - center, f.center() - center);
        double prior;
        if (theta < 5)
            prior = (theta / 5) * (theta / 5);  // too little baseline: depth is ill-conditioned
        else if (theta <= 30)
            prior = 1;
        else
            prior = std::exp(-(theta - 30) * (theta - 30) / (2 * 10.0 * 10.0));
        if (theta > 60) prior = 0;
        double overlap;
        if (vis_ref.size() >= 20) {
            int shared = 0;
            for (auto* X : vis_ref) shared += visible(f, *X);
            overlap = double(shared) / vis_ref.size();
        } else {
            overlap = std::max(0.0, std::cos(angle_deg(r.forward(), f.forward()) * CV_PI / 180.0));
        }
        const double sh = med > 0 ? std::min(1.0, f.sharpness / med) : 1.0;
        const double score = prior * overlap * sh;
        if (score > 0.05) scored.push_back({score, n});
    }
    std::sort(scored.begin(), scored.end(), [](auto& a, auto& b) { return a.first > b.first; });
    std::vector<int> out;
    for (int i = 0; i < int(scored.size()) && i < k; ++i) out.push_back(scored[i].second);
    return out;
}

std::pair<double, double> depth_range(const Session& s, const Frame& f, const cv::Vec3d& center) {
    std::vector<double> z;
    for (auto& X : near_points(s, f, center))
        if (visible(f, X)) z.push_back(f.to_cam(X)[2]);
    double near, far;
    if (z.size() >= 20) {
        std::sort(z.begin(), z.end());
        near = z[z.size() * 5 / 100] * 0.8;
        far = z[z.size() * 95 / 100] * 1.25;
    } else {
        double zc = f.to_cam(center)[2];
        if (zc <= 0.05) zc = 0.4;
        near = zc * 0.5;
        far = zc * 1.6;
    }
    near = std::max(near, 0.05);
    far = std::clamp(far, near * 1.1, 3.0);
    return {near, far};
}

DepthMap plane_sweep(const MvsView& ref, const std::vector<const MvsView*>& nbrs, double near, double far,
                     const MvsParams& p) {
    DepthMap out;
    out.resize(ref.K);
    out.source = DepthMap::Source::Mvs;
    if (nbrs.empty()) return out;

    const cv::Size size(ref.K.w, ref.K.h), win(p.window, p.window);
    const cv::Mat& I = ref.gray;
    cv::Mat muI, II, varI;
    cv::boxFilter(I, muI, CV_32F, win);
    cv::boxFilter(I.mul(I), II, CV_32F, win);
    varI = II - muI.mul(muI);

    const cv::Matx33d Kr_inv = kmat(ref.K).inv();
    const size_t N = nbrs.size();
    std::vector<cv::Matx33d> Rs(N), Kn(N);
    std::vector<cv::Vec3d> ts(N);
    std::vector<cv::Mat> ones(N);
    for (size_t j = 0; j < N; ++j) {
        const cv::Matx44d T = nbrs[j]->T_wc.inv() * ref.T_wc;  // ref camera -> neighbour camera
        Rs[j] = T.get_minor<3, 3>(0, 0);
        ts[j] = {T(0, 3), T(1, 3), T(2, 3)};
        Kn[j] = kmat(nbrs[j]->K);
        ones[j] = cv::Mat(nbrs[j]->gray.size(), CV_8U, cv::Scalar(255));
    }

    const int P = std::max(p.planes, 3);
    const double inv_far = 1.0 / far, inv_near = 1.0 / near, step = (inv_near - inv_far) / (P - 1);
    const size_t npx = size_t(size.area());
    std::vector<float> best(npx, 1e9f), cm1(npx, 2.f), cp1(npx, 2.f), prev(npx, 2.f);
    std::vector<int> best_idx(npx, -1);
    std::vector<uint8_t> pending(npx, 0);
    std::vector<cv::Mat> cost(N);
    const cv::Mat erode_k = cv::getStructuringElement(cv::MORPH_RECT, win);
    const float tex2 = float(p.min_texture * p.min_texture);
    const int k_use = int(std::min<size_t>(std::max(p.best_k, 1), N));
    const cv::Vec3d centre_ray = Kr_inv * cv::Vec3d(ref.K.cx, ref.K.cy, 1);

    for (int k = 0; k < P; ++k) {
        const double d = 1.0 / (inv_far + k * step);  // fronto-parallel plane z = d in the reference camera
        for (size_t j = 0; j < N; ++j) {
            cost[j].create(size, CV_32F);
            // plane behind the neighbour: no valid correspondence
            if ((Rs[j] * (centre_ray * d) + ts[j])[2] <= 0.01) {
                cost[j].setTo(2.f);
                continue;
            }
            // x_n ~ K_n (R + t n^T / d) K_r^-1 x_r with n = (0,0,1): the plane-induced homography,
            // i.e. back-project each reference pixel to the plane and project it into the neighbour.
            cv::Matx33d A = Rs[j];
            for (int r = 0; r < 3; ++r) A(r, 2) += ts[j][r] / d;
            const cv::Matx33d H = Kn[j] * A * Kr_inv;
            cv::Mat Jw, Mw;
            cv::warpPerspective(nbrs[j]->gray, Jw, cv::Mat(H), size, cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
                                cv::BORDER_CONSTANT, 0);
            cv::warpPerspective(ones[j], Mw, cv::Mat(H), size, cv::INTER_NEAREST | cv::WARP_INVERSE_MAP,
                                cv::BORDER_CONSTANT, 0);
            cv::erode(Mw, Mw, erode_k);
            cv::Mat muJ, JJ, IJ;
            cv::boxFilter(Jw, muJ, CV_32F, win);
            cv::boxFilter(Jw.mul(Jw), JJ, CV_32F, win);
            cv::boxFilter(I.mul(Jw), IJ, CV_32F, win);
            for (int y = 0; y < size.height; ++y) {
                const float *mi = muI.ptr<float>(y), *vi = varI.ptr<float>(y), *mj = muJ.ptr<float>(y),
                            *jj = JJ.ptr<float>(y), *ij = IJ.ptr<float>(y);
                const uint8_t* m = Mw.ptr<uint8_t>(y);
                float* c = cost[j].ptr<float>(y);
                for (int x = 0; x < size.width; ++x) {
                    const float vj = jj[x] - mj[x] * mj[x];
                    if (!m[x] || vj < tex2 || vi[x] < tex2) {
                        c[x] = 2.f;
                        continue;
                    }
                    const float zncc = (ij[x] - mi[x] * mj[x]) / std::sqrt(vi[x] * vj);
                    c[x] = 1.f - std::clamp(zncc, -1.f, 1.f);
                }
            }
        }
        // best-k aggregation: a neighbour where this pixel is occluded has a high cost and drops out
        float cs[16];
        for (size_t px = 0; px < npx; ++px) {
            const int y = int(px / size.width), x = int(px % size.width);
            for (size_t j = 0; j < N; ++j) cs[j] = cost[j].ptr<float>(y)[x];
            std::partial_sort(cs, cs + k_use, cs + N);
            float agg = 0;
            for (int q = 0; q < k_use; ++q) agg += cs[q];
            agg /= k_use;
            if (pending[px]) {
                cp1[px] = agg;
                pending[px] = 0;
            }
            if (agg < best[px]) {
                best[px] = agg;
                best_idx[px] = k;
                cm1[px] = k > 0 ? prev[px] : 2.f;
                cp1[px] = 2.f;
                pending[px] = 1;
            }
            prev[px] = agg;
        }
    }

    constexpr double kCostNoise = 0.03;  // typical ZNCC cost jitter near the minimum
    for (size_t px = 0; px < npx; ++px) {
        const int y = int(px / size.width), x = int(px % size.width);
        if (varI.ptr<float>(y)[x] < tex2) continue;                 // low texture
        const int bi = best_idx[px];
        if (bi <= 0 || bi >= P - 1) continue;                        // minimum at the range edge
        if (1.0 - best[px] < p.min_zncc) continue;                   // poor match
        const double a = double(cm1[px]) - 2.0 * best[px] + cp1[px];  // curvature, per plane index^2
        if (a <= 1e-6 || cm1[px] >= 2.f || cp1[px] >= 2.f) continue;
        const double delta = std::clamp(0.5 * (cm1[px] - cp1[px]) / a, -0.5, 0.5);
        const double inv = inv_far + (bi + delta) * step;
        const double z = 1.0 / inv;
        const double sigma_idx = std::max(std::sqrt(2 * kCostNoise / a), 0.1);
        const double sigma = z * z * sigma_idx * step;
        if (sigma / z > p.max_rel_sigma) continue;                   // too uncertain
        out.d[px] = float(z);
        out.sigma[px] = float(sigma);
    }
    return out;
}

void consistency_filter(std::vector<DepthMap>& maps, const std::vector<cv::Matx44d>& poses,
                        const std::vector<std::vector<int>>& nbrs, const MvsParams& p) {
    const std::vector<DepthMap> orig = maps;
    for (size_t i = 0; i < maps.size(); ++i) {
        DepthMap& m = maps[i];
        if (m.source != DepthMap::Source::Mvs || m.empty()) continue;
        const cv::Matx44d Ti = poses[i], Ti_inv = Ti.inv();
        std::vector<cv::Matx44d> Tji;  // ref camera -> neighbour camera
        std::vector<const DepthMap*> nm;
        std::vector<cv::Matx44d> Tij;  // neighbour camera -> ref camera
        for (int j : nbrs[i]) {
            if (orig[j].empty()) continue;
            Tji.push_back(poses[j].inv() * Ti);
            Tij.push_back(Ti_inv * poses[j]);
            nm.push_back(&orig[j]);
        }
        for (int y = 0; y < m.K.h; ++y)
            for (int x = 0; x < m.K.w; ++x) {
                const size_t px = size_t(y) * m.K.w + x;
                const double z = m.d[px];
                if (z <= 0) continue;
                const cv::Vec3d Xr = m.K.ray(x, y) * z;
                int agree = 0;
                for (size_t q = 0; q < nm.size(); ++q) {
                    const DepthMap& n = *nm[q];
                    const cv::Vec4d Xn4 = Tji[q] * cv::Vec4d(Xr[0], Xr[1], Xr[2], 1);
                    if (Xn4[2] <= 0.01) continue;
                    const cv::Vec2d uv = n.K.project({Xn4[0], Xn4[1], Xn4[2]});
                    const int u = int(std::lround(uv[0])), v = int(std::lround(uv[1]));
                    if (u < 0 || v < 0 || u >= n.K.w || v >= n.K.h) continue;
                    const double zn = n.d[size_t(v) * n.K.w + u];
                    if (zn <= 0 || std::abs(zn - Xn4[2]) / Xn4[2] > p.consist_rel) continue;
                    const cv::Vec3d Xb = n.K.ray(u, v) * zn;
                    const cv::Vec4d Xb4 = Tij[q] * cv::Vec4d(Xb[0], Xb[1], Xb[2], 1);
                    if (Xb4[2] <= 0.01) continue;
                    const cv::Vec2d back = m.K.project({Xb4[0], Xb4[1], Xb4[2]});
                    if (cv::norm(back - cv::Vec2d(x, y)) > p.consist_px) continue;
                    ++agree;
                }
                if (agree < p.min_consistent) m.d[px] = m.sigma[px] = 0.f;
            }
    }
}

}  // namespace psom
