#include "psom/sfm.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <memory>
#include <numeric>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/features.hpp>
#include <opencv2/geometry.hpp>
#else
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#endif
#include <opencv2/flann.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace psom {

namespace {

cv::Matx33d skew(const cv::Vec3d& v) { return {0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0}; }

cv::Matx33d exp_so3(const cv::Vec3d& w) {
    const double th = cv::norm(w);
    if (th < 1e-12) return cv::Matx33d::eye() + skew(w);
    const cv::Matx33d K = skew(w / th);
    return cv::Matx33d::eye() + std::sin(th) * K + (1 - std::cos(th)) * (K * K);
}

// Rotation that turns unit vector a into unit vector b.
cv::Matx33d rotation_between(const cv::Vec3d& a, const cv::Vec3d& b) {
    const cv::Vec3d v = a.cross(b);
    const double c = a.dot(b);
    if (c < -0.999999) {
        cv::Vec3d p = std::abs(a[0]) < 0.9 ? cv::Vec3d(1, 0, 0) : cv::Vec3d(0, 1, 0);
        return exp_so3(cv::normalize(a.cross(p)) * CV_PI);
    }
    const cv::Matx33d V = skew(v);
    return cv::Matx33d::eye() + V + (V * V) * (1.0 / (1 + c));
}

double huber_cost(double e, double d) { return e <= d ? e * e : 2 * d * e - d * d; }

struct UnionFind {
    std::vector<int> p;
    explicit UnionFind(size_t n) : p(n) { std::iota(p.begin(), p.end(), 0); }
    int find(int x) {
        while (p[x] != x) x = p[x] = p[p[x]];
        return x;
    }
    void unite(int a, int b) { p[find(a)] = find(b); }
};

struct Image {
    std::vector<cv::Point2d> kp;  // full-resolution pixels
    cv::Mat desc;
    std::unique_ptr<cv::flann::Index> index;
};

struct Track {
    std::vector<std::pair<int, int>> obs;  // (image, feature)
    std::vector<char> bad;                 // observation rejected as an outlier
    int pt = -1;
};

// Multi-view linear triangulation in normalised coordinates.
bool triangulate(const std::vector<cv::Matx33d>& R, const std::vector<cv::Vec3d>& t, const std::vector<int>& cams,
                 const std::vector<cv::Vec2d>& xn, cv::Vec3d& X) {
    cv::Mat A(int(cams.size()) * 2, 4, CV_64F);
    for (size_t k = 0; k < cams.size(); ++k) {
        const cv::Matx33d& r = R[cams[k]];
        const cv::Vec3d& tt = t[cams[k]];
        for (int c = 0; c < 4; ++c) {
            const double p0 = c < 3 ? r(0, c) : tt[0], p1 = c < 3 ? r(1, c) : tt[1], p2 = c < 3 ? r(2, c) : tt[2];
            A.at<double>(int(2 * k), c) = xn[k][0] * p2 - p0;
            A.at<double>(int(2 * k + 1), c) = xn[k][1] * p2 - p1;
        }
    }
    cv::Mat w, u, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);
    const double h = vt.at<double>(3, 3);
    if (std::abs(h) < 1e-12) return false;
    X = {vt.at<double>(3, 0) / h, vt.at<double>(3, 1) / h, vt.at<double>(3, 2) / h};
    return std::isfinite(X[0]) && std::isfinite(X[1]) && std::isfinite(X[2]);
}

}  // namespace

cv::Vec3d device_up_to_camera(const cv::Vec3d& g, int sensor_orientation_deg) {
    // The raw image must turn this far clockwise to look upright in the device's natural
    // orientation. Device axes: x right, y up, z out of the screen; the back camera looks along -z.
    const double a = sensor_orientation_deg * CV_PI / 180, c = std::round(std::cos(a)), s = std::round(std::sin(a));
    const cv::Vec3d xc(c, -s, 0), yc(-s, -c, 0), zc(0, 0, -1);
    return cv::normalize(cv::Vec3d(xc.dot(g), yc.dot(g), zc.dot(g)));
}

double bundle_adjust(std::vector<cv::Matx33d>& R, std::vector<cv::Vec3d>& t, std::vector<cv::Vec3d>& X, Intrinsics& K,
                     const std::vector<BaObs>& obs, bool refine_focal, int iterations, double max_px,
                     std::vector<char>* inlier) {
    const int M = int(R.size());
    const int nc = 6 * std::max(0, M - 1) + (refine_focal ? 1 : 0);
    const int fi = refine_focal ? nc - 1 : -1;
    constexpr double kHuber = 2.0;
    std::vector<std::vector<int>> by_pt(X.size());
    for (size_t k = 0; k < obs.size(); ++k) by_pt[obs[k].pt].push_back(int(k));

    auto total_cost = [&](const std::vector<cv::Matx33d>& r, const std::vector<cv::Vec3d>& tt,
                          const std::vector<cv::Vec3d>& x, double f) {
        double c = 0;
        for (auto& o : obs) {
            if (by_pt[o.pt].size() < 2) continue;
            const cv::Vec3d p = r[o.cam] * x[o.pt] + tt[o.cam];
            if (p[2] <= 1e-9) {
                c += huber_cost(100, kHuber);
                continue;
            }
            const double du = f * p[0] / p[2] + K.cx - o.uv[0], dv = f * p[1] / p[2] + K.cy - o.uv[1];
            c += huber_cost(std::sqrt(du * du + dv * dv), kHuber);
        }
        return c;
    };

    double f = K.fx, cost = total_cost(R, t, X, f), lambda = 1e-3;
    using Vec7 = cv::Vec<double, 7>;
    using Mat73 = cv::Matx<double, 7, 3>;
    std::vector<std::array<int, 7>> idx(obs.size());
    for (size_t k = 0; k < obs.size(); ++k)
        for (int j = 0; j < 7; ++j) idx[k][j] = j < 6 ? (obs[k].cam > 0 ? 6 * (obs[k].cam - 1) + j : -1) : fi;

    for (int it = 0; it < iterations; ++it) {
        cv::Mat A = cv::Mat::zeros(nc, nc, CV_64F);
        cv::Mat gy = cv::Mat::zeros(nc, 1, CV_64F);
        std::vector<cv::Matx33d> V(X.size(), cv::Matx33d::zeros());
        std::vector<cv::Vec3d> gp(X.size(), cv::Vec3d(0, 0, 0));
        std::vector<Mat73> W(obs.size());
        for (size_t k = 0; k < obs.size(); ++k) {
            const BaObs& o = obs[k];
            if (by_pt[o.pt].size() < 2) continue;
            const cv::Vec3d RX = R[o.cam] * X[o.pt], p = RX + t[o.cam];
            if (p[2] <= 1e-9) continue;
            const double iz = 1 / p[2];
            const cv::Vec2d r(f * p[0] * iz + K.cx - o.uv[0], f * p[1] * iz + K.cy - o.uv[1]);
            const double e = cv::norm(r), w = e <= kHuber ? 1.0 : kHuber / e;
            const cv::Matx23d Jp(f * iz, 0, -f * p[0] * iz * iz, 0, f * iz, -f * p[1] * iz * iz);
            const cv::Matx23d Jw = Jp * (-skew(RX));
            cv::Matx<double, 2, 7> Jy;
            for (int a = 0; a < 2; ++a) {
                for (int c = 0; c < 3; ++c) Jy(a, c) = Jw(a, c), Jy(a, 3 + c) = Jp(a, c);
                Jy(a, 6) = a == 0 ? p[0] * iz : p[1] * iz;
            }
            const cv::Matx23d JX = Jp * R[o.cam];
            const auto& ix = idx[k];
            for (int a = 0; a < 7; ++a) {
                if (ix[a] < 0) continue;
                double* Arow = A.ptr<double>(ix[a]);
                for (int b = 0; b < 7; ++b) {
                    if (ix[b] < 0) continue;
                    Arow[ix[b]] += w * (Jy(0, a) * Jy(0, b) + Jy(1, a) * Jy(1, b));
                }
                gy.at<double>(ix[a]) += w * (Jy(0, a) * r[0] + Jy(1, a) * r[1]);
            }
            V[o.pt] += w * (JX.t() * JX);
            gp[o.pt] += w * (JX.t() * r);
            W[k] = w * (Jy.t() * JX);
        }

        bool improved = false;
        for (int trial = 0; trial < 8 && !improved; ++trial) {
            cv::Mat S = A.clone(), rhs = -gy;
            for (int d = 0; d < nc; ++d) S.at<double>(d, d) += lambda * std::max(A.at<double>(d, d), 1e-9);
            std::vector<cv::Matx33d> Vi(X.size());
            std::vector<char> ok(X.size(), 0);
            for (size_t pi = 0; pi < X.size(); ++pi) {
                if (by_pt[pi].size() < 2) continue;
                cv::Matx33d Vd = V[pi];
                for (int d = 0; d < 3; ++d) Vd(d, d) += lambda * std::max(V[pi](d, d), 1e-9);
                if (std::abs(cv::determinant(Vd)) < 1e-30) continue;
                Vi[pi] = Vd.inv(cv::DECOMP_CHOLESKY);
                ok[pi] = 1;
                const auto& ks = by_pt[pi];
                for (int ka : ks) {
                    const Mat73 WV = W[ka] * Vi[pi];
                    const Vec7 wg = WV * gp[pi];
                    for (int a = 0; a < 7; ++a)
                        if (idx[ka][a] >= 0) rhs.at<double>(idx[ka][a]) += wg[a];
                    for (int kb : ks) {
                        const cv::Matx<double, 7, 7> B = WV * W[kb].t();
                        for (int a = 0; a < 7; ++a) {
                            if (idx[ka][a] < 0) continue;
                            double* Srow = S.ptr<double>(idx[ka][a]);
                            for (int b = 0; b < 7; ++b)
                                if (idx[kb][b] >= 0) Srow[idx[kb][b]] -= B(a, b);
                        }
                    }
                }
            }
            cv::Mat dy = cv::Mat::zeros(nc, 1, CV_64F);
            if (nc > 0 && !cv::solve(S, rhs, dy, cv::DECOMP_CHOLESKY) && !cv::solve(S, rhs, dy, cv::DECOMP_SVD)) {
                lambda *= 10;
                continue;
            }
            std::vector<cv::Matx33d> R2 = R;
            std::vector<cv::Vec3d> t2 = t, X2 = X;
            for (int c = 1; c < M; ++c) {
                const double* d = dy.ptr<double>(6 * (c - 1));
                R2[c] = exp_so3({d[0], d[1], d[2]}) * R[c];
                t2[c] = t[c] + cv::Vec3d(d[3], d[4], d[5]);
            }
            const double f2 = refine_focal ? f + dy.at<double>(fi) : f;
            for (size_t pi = 0; pi < X.size(); ++pi) {
                if (!ok[pi]) continue;
                cv::Vec3d b = -gp[pi];
                for (int k : by_pt[pi]) {
                    Vec7 d;
                    for (int a = 0; a < 7; ++a) d[a] = idx[k][a] >= 0 ? dy.at<double>(idx[k][a]) : 0;
                    b -= W[k].t() * d;
                }
                X2[pi] = X[pi] + Vi[pi] * b;
            }
            const double c2 = total_cost(R2, t2, X2, f2);
            if (std::isfinite(c2) && c2 < cost && f2 > 0) {
                const double gain = (cost - c2) / std::max(cost, 1e-12);
                R = std::move(R2), t = std::move(t2), X = std::move(X2), f = f2, cost = c2;
                lambda = std::max(lambda / 3, 1e-9);
                improved = true;
                if (gain < 1e-6) it = iterations;  // converged
            } else {
                lambda *= 5;
            }
        }
        if (!improved) break;
    }
    K.fx = K.fy = f;

    double sq = 0;
    size_t n = 0;
    if (inlier) inlier->assign(obs.size(), 0);
    for (size_t k = 0; k < obs.size(); ++k) {
        const BaObs& o = obs[k];
        const cv::Vec3d p = R[o.cam] * X[o.pt] + t[o.cam];
        if (p[2] <= 1e-9) continue;
        const cv::Vec2d r = cv::Vec2d(f * p[0] / p[2] + K.cx, f * p[1] / p[2] + K.cy) - o.uv;
        const double e = cv::norm(r);
        if (e > max_px) continue;
        if (inlier) (*inlier)[k] = 1;
        sq += e * e;
        ++n;
    }
    return n ? std::sqrt(sq / n) : 0;
}

Result run_sfm(const Session& s, const SfmParams& p, SfmResult& out, const Progress& progress) {
    auto P = [&](double f) {
        if (progress) progress("poses", f);
    };
    out = SfmResult{};
    const int N = int(s.frames.size());
    const auto t0 = std::chrono::steady_clock::now();
    auto lap = [&](const char* what) {
        if (p.verbose)
            std::fprintf(stderr, "{\"event\":\"sfm_time\",\"after\":\"%s\",\"s\":%.2f}\n", what,
                         std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    };
    if (N < 3) return Result::fail("poses", "At least 3 photos are needed.");
    Intrinsics K = s.frames[0].K;
    out.f_guess = K.fx;

    // 1. features (RootSIFT)
    std::vector<Image> im(N);
    const cv::Ptr<cv::SIFT> sift = cv::SIFT::create(p.max_features, 3, p.contrast);
    const double scale = std::min(1.0, double(p.max_side) / std::max(s.width, s.height));
    // Photos are independent: in parallel. Progress is only reported from this thread (JNI needs that).
    P(0.05);
    cv::parallel_for_(cv::Range(0, N), [&](const cv::Range& range) {
      for (int i = range.start; i < range.end; ++i) {
        cv::Mat g = cv::imread(s.image_path(s.frames[i]).string(), cv::IMREAD_GRAYSCALE);
        if (g.empty() || g.cols != s.width || g.rows != s.height) continue;
        if (scale < 1) cv::resize(g, g, {}, scale, scale, cv::INTER_AREA);
        if (p.equalize) cv::createCLAHE(2.0, cv::Size(8, 8))->apply(g, g);
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        sift->detectAndCompute(g, cv::noArray(), kps, desc);
        if (kps.size() < 20) continue;
        for (int r = 0; r < desc.rows; ++r) {
            float* d = desc.ptr<float>(r);
            float sum = 0;
            for (int c = 0; c < desc.cols; ++c) sum += std::abs(d[c]);
            for (int c = 0; c < desc.cols; ++c) d[c] = std::sqrt(std::abs(d[c]) / std::max(sum, 1e-9f));
        }
        for (auto& k : kps) im[i].kp.emplace_back((k.pt.x + 0.5) / scale - 0.5, (k.pt.y + 0.5) / scale - 0.5);
        im[i].desc = desc;
        im[i].index = std::make_unique<cv::flann::Index>(im[i].desc, cv::flann::KDTreeIndexParams(4));
      }
    });
    P(0.25);
    if (p.verbose)
        for (int i = 0; i < N; ++i) std::fprintf(stderr, "{\"event\":\"features\",\"frame\":%d,\"n\":%zu}\n", s.frames[i].i, im[i].kp.size());

    lap("features");
    // 2. match each photo with its neighbours in capture order; an orbit closes, so wrap around
    struct Pair {
        int a, b;
        std::vector<std::pair<int, int>> m;
    };
    std::vector<Pair> pairs;
    std::vector<std::pair<int, int>> todo;
    for (int a = 0; a < N; ++a)
        for (int b = a + 1; b < N; ++b)
            if (std::min(b - a, N - (b - a)) <= p.window) todo.emplace_back(a, b);
    std::vector<Pair> found(todo.size());
    cv::parallel_for_(cv::Range(0, int(todo.size())), [&](const cv::Range& range) {
      for (int q = range.start; q < range.end; ++q) {
        const auto [a, b] = todo[q];
        if (!im[a].index || !im[b].index) continue;
        cv::Mat ind, dist;
        im[b].index->knnSearch(im[a].desc, ind, dist, 2, cv::flann::SearchParams(16));
        std::vector<int> best_for_b(im[b].kp.size(), -1);
        std::vector<float> best_d(im[b].kp.size(), 1e30f);
        for (int r = 0; r < ind.rows; ++r) {
            const int j = ind.at<int>(r, 0);
            const float d0 = dist.at<float>(r, 0), d1 = dist.at<float>(r, 1);  // squared L2
            if (j < 0 || d0 > p.ratio * p.ratio * d1) continue;
            if (d0 < best_d[j]) best_d[j] = d0, best_for_b[j] = r;
        }
        std::vector<cv::Point2d> pa, pb;
        std::vector<std::pair<int, int>> m;
        for (size_t j = 0; j < best_for_b.size(); ++j)
            if (best_for_b[j] >= 0) {
                m.emplace_back(best_for_b[j], int(j));
                pa.push_back(im[a].kp[best_for_b[j]]);
                pb.push_back(im[b].kp[j]);
            }
        if (int(m.size()) < p.min_inliers) continue;
        std::vector<uchar> mask;
        const cv::Mat F = cv::findFundamentalMat(pa, pb, cv::FM_RANSAC, p.epipolar_px, 0.999, mask);
        if (F.empty()) continue;
        Pair pr{a, b, {}};
        for (size_t k = 0; k < m.size(); ++k)
            if (mask[k]) pr.m.push_back(m[k]);
        if (p.verbose)
            std::fprintf(stderr, "{\"event\":\"pair\",\"a\":%d,\"b\":%d,\"matches\":%zu,\"inliers\":%zu}\n", a, b, m.size(), pr.m.size());
        if (int(pr.m.size()) >= p.min_inliers) found[q] = std::move(pr);
      }
    });
    for (auto& pr : found)
        if (!pr.m.empty()) pairs.push_back(std::move(pr));
    P(0.5);
    if (pairs.empty())
        return Result::fail("poses", "The photos don't overlap enough to work out where they were taken. Move slower and keep the object in view.");

    lap("matching");
    // 3. tracks: features linked across photos; one that lands twice in a photo is ambiguous
    std::vector<int> offset(N + 1, 0);
    for (int i = 0; i < N; ++i) offset[i + 1] = offset[i] + int(im[i].kp.size());
    UnionFind uf(offset[N]);
    for (auto& pr : pairs)
        for (auto& [fa, fb] : pr.m) uf.unite(offset[pr.a] + fa, offset[pr.b] + fb);
    std::vector<int> root_track(offset[N], -1);
    std::vector<Track> tracks;
    for (int i = 0; i < N; ++i)
        for (int f = 0; f < int(im[i].kp.size()); ++f) {
            const int r = uf.find(offset[i] + f);
            if (root_track[r] < 0) root_track[r] = int(tracks.size()), tracks.emplace_back();
            tracks[root_track[r]].obs.emplace_back(i, f);
        }
    std::vector<std::vector<int>> feat_track(N);
    for (int i = 0; i < N; ++i) feat_track[i].assign(im[i].kp.size(), -1);
    {
        std::vector<Track> kept;
        for (auto& tr : tracks) {
            if (tr.obs.size() < 2) continue;
            bool dup = false;
            for (size_t k = 1; k < tr.obs.size() && !dup; ++k) dup = tr.obs[k].first == tr.obs[k - 1].first;
            if (dup) continue;
            tr.bad.assign(tr.obs.size(), 0);
            for (auto& [i, f] : tr.obs) feat_track[i][f] = int(kept.size());
            kept.push_back(std::move(tr));
        }
        tracks = std::move(kept);
    }

    auto norm_pt = [&](int i, int f, double fx) {
        const cv::Point2d& u = im[i].kp[f];
        return cv::Vec2d((u.x - K.cx) / fx, (u.y - K.cy) / fx);
    };

    // 4. the first two cameras: many matches and a real baseline
    std::vector<int> cam_of(N, -1), cam_img;
    std::vector<cv::Matx33d> R;
    std::vector<cv::Vec3d> t, X;
    {
        std::vector<size_t> order(pairs.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](size_t x, size_t y) { return pairs[x].m.size() > pairs[y].m.size(); });
        double best_score = 0;
        cv::Matx33d bestR;
        cv::Vec3d bestT;
        int best = -1;
        for (size_t q = 0; q < std::min<size_t>(order.size(), 40); ++q) {
            const Pair& pr = pairs[order[q]];
            std::vector<cv::Point2d> na, nb;
            for (auto& [fa, fb] : pr.m) {
                const cv::Vec2d x = norm_pt(pr.a, fa, K.fx), y = norm_pt(pr.b, fb, K.fx);
                na.emplace_back(x[0], x[1]);
                nb.emplace_back(y[0], y[1]);
            }
            cv::Mat mask;
            cv::Mat E = cv::findEssentialMat(na, nb, 1.0, cv::Point2d(0, 0), cv::RANSAC, 0.999, p.epipolar_px / K.fx, 1000, mask);
            if (E.rows < 3) continue;
            E = E.rowRange(0, 3).clone();
            cv::Mat Rm, tm;
            const int good = cv::recoverPose(E, na, nb, Rm, tm, 1.0, cv::Point2d(0, 0), mask);
            if (good < 50) continue;
            cv::Matx33d Rb;
            cv::Vec3d tb;
            for (int a = 0; a < 3; ++a) {
                tb[a] = tm.at<double>(a);
                for (int b = 0; b < 3; ++b) Rb(a, b) = Rm.at<double>(a, b);
            }
            const cv::Vec3d Cb = -(Rb.t() * tb);
            std::vector<double> ang;
            const std::vector<cv::Matx33d> RR{cv::Matx33d::eye(), Rb};
            const std::vector<cv::Vec3d> TT{cv::Vec3d(0, 0, 0), tb};
            for (int k = 0; k < mask.rows; ++k) {
                if (!mask.at<uchar>(k)) continue;
                cv::Vec3d Xk;
                if (!triangulate(RR, TT, {0, 1}, {cv::Vec2d(na[k].x, na[k].y), cv::Vec2d(nb[k].x, nb[k].y)}, Xk)) continue;
                const double c = cv::normalize(Xk).dot(cv::normalize(Xk - Cb));
                ang.push_back(std::acos(std::clamp(c, -1.0, 1.0)) * 180 / CV_PI);
            }
            if (ang.size() < 50) continue;
            std::nth_element(ang.begin(), ang.begin() + ang.size() / 2, ang.end());
            const double med = ang[ang.size() / 2];
            if (med < 2.0) continue;
            const double score = good * std::min(med, 8.0);
            if (score > best_score) best_score = score, best = int(order[q]), bestR = Rb, bestT = tb;
        }
        if (best < 0)
            return Result::fail("poses", "The photos don't show the object from far enough apart. Walk around it rather than turning on the spot.");
        cam_of[pairs[best].a] = 0, cam_of[pairs[best].b] = 1;
        cam_img = {pairs[best].a, pairs[best].b};
        R = {cv::Matx33d::eye(), bestR};
        t = {cv::Vec3d(0, 0, 0), bestT};
    }

    // Triangulates tracks seen by at least two placed cameras that don't have a point yet.
    auto triangulate_new = [&]() {
        for (auto& tr : tracks) {
            if (tr.pt >= 0) continue;
            std::vector<int> cams, which;
            std::vector<cv::Vec2d> xn;
            for (size_t k = 0; k < tr.obs.size(); ++k) {
                const int c = cam_of[tr.obs[k].first];
                if (c < 0 || tr.bad[k]) continue;
                cams.push_back(c), which.push_back(int(k)), xn.push_back(norm_pt(tr.obs[k].first, tr.obs[k].second, K.fx));
            }
            if (cams.size() < 2) continue;
            cv::Vec3d Xk;
            if (!triangulate(R, t, cams, xn, Xk)) continue;
            int good = 0;
            double max_ang = 0;
            std::vector<cv::Vec3d> rays;
            for (size_t k = 0; k < cams.size(); ++k) {
                const cv::Vec3d pc = R[cams[k]] * Xk + t[cams[k]];
                const cv::Vec2d u = norm_pt(tr.obs[which[k]].first, tr.obs[which[k]].second, K.fx);
                const bool ok = pc[2] > 1e-6 && cv::norm(cv::Vec2d(pc[0] / pc[2], pc[1] / pc[2]) - u) * K.fx <= p.max_reproj_px;
                if (!ok) continue;
                ++good;
                const cv::Vec3d ray = cv::normalize(Xk + R[cams[k]].t() * t[cams[k]]);  // from the camera centre
                for (auto& r2 : rays) max_ang = std::max(max_ang, std::acos(std::clamp(ray.dot(r2), -1.0, 1.0)));
                rays.push_back(ray);
            }
            if (good < 2 || max_ang * 180 / CV_PI < p.min_angle_deg) continue;
            tr.pt = int(X.size());
            X.push_back(Xk);
        }
    };

    // Bundle adjusts everything placed so far and drops observations that don't fit.
    auto adjust = [&](bool focal, int iterations) {
        std::vector<BaObs> obs;
        std::vector<std::pair<int, int>> back;  // (track, obs index)
        for (int ti = 0; ti < int(tracks.size()); ++ti) {
            const Track& tr = tracks[ti];
            if (tr.pt < 0) continue;
            for (size_t k = 0; k < tr.obs.size(); ++k) {
                const int c = cam_of[tr.obs[k].first];
                if (c < 0 || tr.bad[k]) continue;
                const cv::Point2d& u = im[tr.obs[k].first].kp[tr.obs[k].second];
                obs.push_back({c, tr.pt, {u.x, u.y}});
                back.emplace_back(ti, int(k));
            }
        }
        std::vector<char> in;
        const double rms = bundle_adjust(R, t, X, K, obs, focal, iterations, p.max_reproj_px, &in);
        std::vector<int> good(X.size(), 0);
        for (size_t k = 0; k < obs.size(); ++k) {
            if (!in[k]) tracks[back[k].first].bad[back[k].second] = 1;
            else ++good[obs[k].pt];
        }
        for (auto& tr : tracks)
            if (tr.pt >= 0 && good[tr.pt] < 2) tr.pt = -1;  // the point stays in X, unused
        return rms;
    };

    triangulate_new();
    adjust(false, 20);
    lap("init");

    // 5. add the photo that sees the most known points, one at a time
    std::vector<int> tried_at(N, 0);
    size_t last_full = cam_img.size();
    for (;;) {
        P(0.5 + 0.45 * cam_img.size() / N);
        int best = -1, best_n = 0;
        for (int i = 0; i < N; ++i) {
            if (cam_of[i] >= 0) continue;
            int n = 0;
            for (int ti : feat_track[i])
                if (ti >= 0 && tracks[ti].pt >= 0) ++n;
            if (n > best_n && n > tried_at[i] * 1.25) best = i, best_n = n;
        }
        if (best < 0 || best_n < 12) break;
        std::vector<cv::Point3d> obj;
        std::vector<cv::Point2d> img;
        for (size_t f = 0; f < feat_track[best].size(); ++f) {
            const int ti = feat_track[best][f];
            if (ti < 0 || tracks[ti].pt < 0) continue;
            const cv::Vec3d& Xk = X[tracks[ti].pt];
            obj.emplace_back(Xk[0], Xk[1], Xk[2]);
            img.push_back(im[best].kp[f]);
        }
        const cv::Matx33d Km(K.fx, 0, K.cx, 0, K.fx, K.cy, 0, 0, 1);
        cv::Mat rvec, tvec, inl;
        const bool ok = cv::solvePnPRansac(obj, img, Km, cv::noArray(), rvec, tvec, false, 300, float(p.pnp_px), 0.999, inl,
                                           cv::SOLVEPNP_EPNP);
        if (p.verbose)
            std::fprintf(stderr, "{\"event\":\"place\",\"k\":%d,\"known\":%d,\"inliers\":%d,\"ok\":%d}\n", best, best_n, ok ? inl.rows : 0, int(ok));
        if (!ok || inl.rows < 12 || inl.rows < 0.25 * obj.size()) {
            tried_at[best] = best_n;
            continue;
        }
        {  // polish the new camera alone on its inliers: cheap, and keeps drift down between full rounds
            std::vector<cv::Point3d> o2;
            std::vector<cv::Point2d> i2;
            for (int r = 0; r < inl.rows; ++r) o2.push_back(obj[inl.at<int>(r)]), i2.push_back(img[inl.at<int>(r)]);
            cv::solvePnPRefineLM(o2, i2, Km, cv::noArray(), rvec, tvec);
        }
        cam_of[best] = int(cam_img.size());
        cam_img.push_back(best);
        R.push_back(exp_so3({rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)}));
        t.emplace_back(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        triangulate_new();
        // Re-adjusting everything after every photo costs O(photos^2) in all; between rounds the new
        // camera's PnP pose is good enough to place the next one.
        if (cam_img.size() >= last_full * p.ba_growth || cam_img.size() < 8) {
            adjust(p.refine_focal && cam_img.size() >= 6, 8);
            last_full = cam_img.size();
            triangulate_new();
        }
    }
    if (cam_img.size() < 3)
        return Result::fail("poses", "Only " + std::to_string(cam_img.size()) + " photos fitted together. Move slower and keep the object in view.");

    lap("placing");
    adjust(p.refine_focal, 30);
    triangulate_new();
    out.rms_px = adjust(p.refine_focal, 15);

    lap("final");
    out.K = K;
    out.K.w = s.width, out.K.h = s.height;
    for (size_t c = 0; c < cam_img.size(); ++c) {
        out.frames.push_back(cam_img[c]);
        const cv::Matx33d Rw = R[c].t();
        const cv::Vec3d C = -(Rw * t[c]);
        cv::Matx44d T = cv::Matx44d::eye();
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) T(a, b) = Rw(a, b);
            T(a, 3) = C[a];
        }
        out.T_wc.push_back(T);
    }
    // keep capture order
    std::vector<size_t> ord(out.frames.size());
    std::iota(ord.begin(), ord.end(), 0);
    std::sort(ord.begin(), ord.end(), [&](size_t a, size_t b) { return out.frames[a] < out.frames[b]; });
    std::vector<int> fr;
    std::vector<cv::Matx44d> Ts;
    for (size_t k : ord) fr.push_back(out.frames[k]), Ts.push_back(out.T_wc[k]);
    out.frames = fr, out.T_wc = Ts;
    for (auto& tr : tracks) {
        if (tr.pt < 0) continue;
        int good = 0;
        for (size_t k = 0; k < tr.obs.size(); ++k) good += cam_of[tr.obs[k].first] >= 0 && !tr.bad[k];
        if (good >= 3) out.points.push_back(X[tr.pt]);
    }
    P(1);
    Result r;
    r.stage = "poses";
    if (int(cam_img.size()) < N)
        r.warn(std::to_string(N - int(cam_img.size())) + " of " + std::to_string(N) + " photos couldn't be placed and were left out");
    return r;
}

Result apply_sfm(Session& s, const SfmResult& r, double distance) {
    Result res;
    res.stage = "poses";
    std::vector<Frame> frames;
    for (size_t k = 0; k < r.frames.size(); ++k) {
        Frame f = s.frames[r.frames[k]];
        f.T_wc = f.T_wc_depth = r.T_wc[k];
        f.K = r.K;
        frames.push_back(f);
    }
    if (frames.size() < 3) return Result::fail("poses", "Too few photos were placed.");

    // up: gravity when the phone recorded it, else the side of the scene the cameras were on
    cv::Vec3d up(0, 0, 0);
    int ng = 0;
    for (auto& f : frames)
        if (f.has_gravity) up += f.R() * device_up_to_camera(f.gravity, s.sensor_orientation), ++ng;
    const bool gravity = ng >= std::max<int>(3, int(frames.size()) / 2) && cv::norm(up) / ng > 0.9;
    if (gravity) {
        up = cv::normalize(up);
    } else {
        cv::Vec3d mean(0, 0, 0);
        for (auto& f : frames) mean += f.center();
        mean /= double(frames.size());
        cv::Matx33d C = cv::Matx33d::zeros();
        for (auto& f : frames) {
            const cv::Vec3d d = f.center() - mean;
            C += d * d.t();
        }
        cv::Mat ev, evec;
        cv::eigen(cv::Mat(C), ev, evec);
        up = cv::Vec3d(evec.at<double>(2, 0), evec.at<double>(2, 1), evec.at<double>(2, 2));
        cv::Vec3d pm(0, 0, 0);
        for (auto& X : r.points) pm += X;
        if (!r.points.empty()) pm /= double(r.points.size());
        if ((mean - pm).dot(up) < 0) up = -up;  // cameras are above the table
        res.warn("no gravity readings: which way is up was guessed from how the camera moved");
    }
    const cv::Matx33d A = rotation_between(up, {0, 1, 0});

    // working scale: the median distance from each camera to what it sees becomes `distance`
    std::vector<double> per_cam;
    for (auto& f : frames) {
        std::vector<double> z;
        for (auto& X : r.points) {
            const cv::Vec3d pc = f.to_cam(X);
            if (pc[2] <= 0) continue;
            const cv::Vec2d u = f.K.project(pc);
            if (u[0] >= 0 && u[0] < f.K.w && u[1] >= 0 && u[1] < f.K.h) z.push_back(pc[2]);
        }
        if (z.size() < 10) continue;
        std::nth_element(z.begin(), z.begin() + z.size() / 2, z.end());
        per_cam.push_back(z[z.size() / 2]);
    }
    if (per_cam.empty()) return Result::fail("poses", "Too few points were found in the photos.");
    std::nth_element(per_cam.begin(), per_cam.begin() + per_cam.size() / 2, per_cam.end());
    const double k = distance / per_cam[per_cam.size() / 2];

    for (auto& f : frames) {
        const cv::Matx33d Rw = A * f.R();
        const cv::Vec3d C = k * (A * f.center());
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) f.T_wc(a, b) = Rw(a, b);
            f.T_wc(a, 3) = C[a];
        }
        f.T_wc_depth = f.T_wc;
    }
    s.frames = std::move(frames);
    s.points.clear();
    for (auto& X : r.points) s.points.push_back({k * (A * X), 1.0});
    s.scale_assumed = true;
    res.ok = true;
    return res;
}

}  // namespace psom
