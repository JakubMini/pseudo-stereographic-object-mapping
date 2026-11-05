#include "psom/isolate.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>


namespace psom {

namespace {

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return 0;
    const size_t k = std::min(v.size() - 1, size_t(q * (v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

constexpr double kPlaneTol = 0.004;      // RANSAC inlier distance
constexpr double kObjMinHeight = 0.006;  // points this far above the plane count as the object

}  // namespace

Rect2 min_area_rect(const std::vector<cv::Vec2d>& in) {
    Rect2 best;
    if (in.empty()) return best;
    std::vector<cv::Vec2d> p = in;
    std::sort(p.begin(), p.end(), [](auto& a, auto& b) { return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]); });
    auto cross = [](const cv::Vec2d& o, const cv::Vec2d& a, const cv::Vec2d& b) {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };
    std::vector<cv::Vec2d> h(2 * p.size());  // monotone chain hull
    size_t k = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
        h[k++] = p[i];
    }
    for (size_t i = p.size() - 1, t = k + 1; i-- > 0;) {
        while (k >= t && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
        h[k++] = p[i];
    }
    h.resize(k > 1 ? k - 1 : k);
    double best_area = 1e300;
    for (size_t i = 0; i < std::max<size_t>(h.size(), 1); ++i) {
        cv::Vec2d e = h.size() >= 2 ? h[(i + 1) % h.size()] - h[i] : cv::Vec2d(1, 0);
        if (cv::norm(e) < 1e-15) continue;
        e /= cv::norm(e);
        const cv::Vec2d nrm(-e[1], e[0]);
        double a0 = 1e300, a1 = -1e300, b0 = 1e300, b1 = -1e300;
        for (auto& q : h) {
            a0 = std::min(a0, q.dot(e)), a1 = std::max(a1, q.dot(e));
            b0 = std::min(b0, q.dot(nrm)), b1 = std::max(b1, q.dot(nrm));
        }
        const double area = (a1 - a0) * (b1 - b0);
        if (area < best_area) {
            best_area = area;
            best.center = e * ((a0 + a1) / 2) + nrm * ((b0 + b1) / 2);
            if (a1 - a0 >= b1 - b0) best.axis = e, best.length = a1 - a0, best.width = b1 - b0;
            else best.axis = nrm, best.length = b1 - b0, best.width = a1 - a0;
        }
    }
    return best;
}

cv::Vec3d scene_center(const Session& s) {
    cv::Matx33d A = cv::Matx33d::zeros();
    cv::Vec3d b(0, 0, 0);
    for (auto& f : s.frames) {
        const cv::Vec3d d = cv::normalize(f.forward());
        const cv::Matx33d M = cv::Matx33d::eye() - d * d.t();
        A += M;
        b += M * f.center();
    }
    if (s.target) {
        const double w = 0.5 * s.frames.size();
        A += w * cv::Matx33d::eye();
        b += w * *s.target;
    }
    cv::Mat eig;
    cv::eigen(cv::Mat(A), eig);
    if (eig.at<double>(2) > 1e-3 * s.frames.size()) return A.solve(b, cv::DECOMP_CHOLESKY);
    // Axes nearly parallel (the user didn't move around): fall back to feature points, then to
    // a point in front of the cameras.
    if (s.points.size() >= 20) {
        std::vector<double> x, y, z;
        for (auto& p : s.points) x.push_back(p.X[0]), y.push_back(p.X[1]), z.push_back(p.X[2]);
        return {percentile(x, 0.5), percentile(y, 0.5), percentile(z, 0.5)};
    }
    cv::Vec3d c(0, 0, 0);
    for (auto& f : s.frames) c += f.center() + 0.4 * cv::normalize(f.forward());
    return c / double(s.frames.size());
}

std::vector<cv::Vec3d> depth_points(const Session& s, const std::vector<DepthMap>& maps, int step, double max_sigma) {
    std::vector<cv::Vec3d> out;
    for (size_t i = 0; i < maps.size() && i < s.frames.size(); ++i) {
        const DepthMap& m = maps[i];
        if (m.empty()) continue;
        for (int y = 0; y < m.K.h; y += step)
            for (int x = 0; x < m.K.w; x += step) {
                const size_t k = size_t(y) * m.K.w + x;
                if (m.d[k] <= 0 || m.sigma[k] > max_sigma) continue;
                out.push_back(s.frames[i].to_world(m.K.ray(x, y) * double(m.d[k])));
            }
    }
    return out;
}

Plane find_support_plane(const std::vector<cv::Vec3d>& pts, const cv::Vec3d& center, double radius) {
    std::vector<cv::Vec3d> cand;
    for (auto& X : pts)
        if (cv::norm(X - center) < radius && X[1] < center[1]) cand.push_back(X);  // world +y is up
    Plane best;
    if (cand.size() < 50) return best;
    std::mt19937 rng(7);
    std::uniform_int_distribution<size_t> pick(0, cand.size() - 1);
    size_t best_n = 0;
    const double cos_max = std::cos(15.0 * CV_PI / 180.0);
    for (int it = 0; it < 600; ++it) {
        const cv::Vec3d a = cand[pick(rng)], b = cand[pick(rng)], c = cand[pick(rng)];
        cv::Vec3d n = (b - a).cross(c - a);
        const double len = cv::norm(n);
        if (len < 1e-9) continue;
        n /= len;
        if (n[1] < 0) n = -n;
        if (n[1] < cos_max) continue;  // not horizontal
        const double d = n.dot(a);
        size_t cnt = 0;
        for (auto& X : cand) cnt += std::abs(n.dot(X) - d) < kPlaneTol;
        if (cnt > best_n) best_n = cnt, best.n = n, best.d = d;
    }
    if (best_n < std::max<size_t>(50, cand.size() / 10)) return Plane{};
    // least-squares refit on the inliers
    cv::Vec3d mean(0, 0, 0);
    std::vector<cv::Vec3d> in;
    for (auto& X : cand)
        if (std::abs(best.n.dot(X) - best.d) < kPlaneTol) in.push_back(X), mean += X;
    mean /= double(in.size());
    cv::Matx33d C = cv::Matx33d::zeros();
    for (auto& X : in) C += (X - mean) * (X - mean).t();
    cv::Mat evals, evecs;
    cv::eigen(cv::Mat(C), evals, evecs);
    cv::Vec3d n(evecs.at<double>(2, 0), evecs.at<double>(2, 1), evecs.at<double>(2, 2));
    if (n[1] < 0) n = -n;
    best.n = cv::normalize(n);
    best.d = best.n.dot(mean);
    best.found = true;
    return best;
}

Result auto_crop(const Session& s, const std::vector<DepthMap>& maps, CropBox& box, Plane& plane) {
    Result r;
    r.stage = "isolate";
    const cv::Vec3d center = scene_center(s);
    std::vector<double> dist;
    for (auto& f : s.frames) dist.push_back(cv::norm(f.center() - center));
    const double radius = std::max(0.15, 0.6 * percentile(dist, 0.5));

    const auto pts = depth_points(s, maps, 2, 0.01);
    if (pts.size() < 200) return Result::fail("isolate", "not enough reliable depth to find the object");

    plane = find_support_plane(pts, center, radius);
    if (!plane.found) {
        r.warn("no support plane found: the bottom stays open and the crop may include background");
        std::vector<double> y;
        for (auto& X : pts)
            if (cv::norm(X - center) < radius) y.push_back(X[1]);
        plane.n = {0, 1, 0};
        plane.d = percentile(y, 0.01);
    }

    const cv::Vec3d origin = center - plane.n * plane.height(center);
    // in-plane basis
    cv::Vec3d u = cv::normalize(cv::Vec3d(1, 0, 0) - plane.n * plane.n[0]);
    cv::Vec3d v = plane.n.cross(u);

    std::vector<cv::Vec3d> obj;
    for (auto& X : pts) {
        const double h = plane.height(X);
        const cv::Vec3d o = X - origin;
        const double ru = o.dot(u), rv = o.dot(v);
        if (h > kObjMinHeight && h < 2 * radius && std::hypot(ru, rv) < radius) obj.push_back(X);
    }
    if (obj.size() < 100) return Result::fail("isolate", "nothing found above the support plane");

    // largest 26-connected cluster on a 5 mm grid
    const double cell = 0.005;
    auto key = [&](const cv::Vec3d& X) {
        const int64_t a = int64_t(std::floor(X[0] / cell)) + (1 << 20), b = int64_t(std::floor(X[1] / cell)) + (1 << 20),
                      c = int64_t(std::floor(X[2] / cell)) + (1 << 20);
        return (a << 42) | (b << 21) | c;
    };
    std::unordered_map<int64_t, std::vector<int>> grid;
    for (int i = 0; i < int(obj.size()); ++i) grid[key(obj[i])].push_back(i);
    std::unordered_map<int64_t, int> label;
    std::vector<size_t> sizes;
    for (auto& [k, _] : grid) {
        if (label.count(k)) continue;
        const int id = int(sizes.size());
        sizes.push_back(0);
        std::vector<int64_t> stack{k};
        label[k] = id;
        while (!stack.empty()) {
            const int64_t c = stack.back();
            stack.pop_back();
            sizes[id] += grid[c].size();
            const int64_t ca = c >> 42, cb = (c >> 21) & ((1 << 21) - 1), cc = c & ((1 << 21) - 1);
            for (int da = -1; da <= 1; ++da)
                for (int db = -1; db <= 1; ++db)
                    for (int dc = -1; dc <= 1; ++dc) {
                        const int64_t nk = ((ca + da) << 42) | ((cb + db) << 21) | (cc + dc);
                        if (grid.count(nk) && !label.count(nk)) label[nk] = id, stack.push_back(nk);
                    }
        }
    }
    const int biggest = int(std::max_element(sizes.begin(), sizes.end()) - sizes.begin());
    std::vector<cv::Vec2d> foot;
    std::vector<double> hs;
    std::vector<cv::Vec3d> keep;
    for (auto& X : obj)
        if (label[key(X)] == biggest) {
            const cv::Vec3d o = X - origin;
            foot.emplace_back(o.dot(u), o.dot(v));
            hs.push_back(plane.height(X));
            keep.push_back(X);
        }
    if (keep.size() < 50) return Result::fail("isolate", "the object cluster is too small");
    if (sizes.size() > 1) {
        size_t rest = 0;
        for (size_t i = 0; i < sizes.size(); ++i)
            if (int(i) != biggest) rest += sizes[i];
        if (rest > keep.size() / 5) r.warn("other things were found near the object and left out of the crop");
    }

    // box axes from the minimum-area rectangle of the footprint, x along the longer side
    const cv::Vec2d e1 = min_area_rect(foot).axis;
    const cv::Vec3d ax = cv::normalize(u * e1[0] + v * e1[1]);
    const cv::Vec3d ay = plane.n.cross(ax);

    std::vector<double> lx, ly;
    for (auto& X : keep) lx.push_back((X - origin).dot(ax)), ly.push_back((X - origin).dot(ay));
    const double m = 0.01;
    const double x0 = percentile(lx, 0.01) - m, x1 = percentile(lx, 0.99) + m;
    const double y0 = percentile(ly, 0.01) - m, y1 = percentile(ly, 0.99) + m;
    const double z0 = plane.found ? kCropFloor : percentile(hs, 0.005) - m;
    const double z1 = percentile(hs, 0.995) + m;

    box.source = "auto";
    box.axes = cv::Matx33d(ax[0], ax[1], ax[2], ay[0], ay[1], ay[2], plane.n[0], plane.n[1], plane.n[2]);
    box.half = {(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2};
    box.center = origin + ax * ((x0 + x1) / 2) + ay * ((y0 + y1) / 2) + plane.n * ((z0 + z1) / 2);
    r.ok = true;
    return r;
}

}  // namespace psom
