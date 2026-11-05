#include <gtest/gtest.h>

#include <random>

#include <opencv2/imgcodecs.hpp>

#include "psom/depth.h"
#include "psom/isolate.h"
#include "psom/mvs.h"
#include "util.h"

namespace {

struct Fixture {
    psom::Session s;
    cv::Vec3d center;
    std::vector<psom::MvsView> views;
    psom::MvsParams p;
};

Fixture& fx() {
    static Fixture f = [] {
        Fixture f;
        psom::SynthSpec sp;  // box with raw depth, used here as the reference
        const fs::path dir = synth_copy("box_depth", sp, "mvs");
        EXPECT_TRUE(psom::load_session(dir / "session", f.s).ok);
        f.center = psom::scene_center(f.s);
        for (auto& fr : f.s.frames)
            f.views.push_back(psom::make_view(cv::imread(f.s.image_path(fr).string(), cv::IMREAD_GRAYSCALE), fr.K, fr.T_wc, f.p.width));
        return f;
    }();
    return f;
}

psom::DepthMap sweep(int i, int corrupt = -1) {
    Fixture& f = fx();
    std::vector<const psom::MvsView*> nv;
    std::vector<psom::MvsView> noisy;
    noisy.reserve(8);
    for (int j : psom::select_neighbours(f.s, i, f.center, f.p.neighbours)) {
        if (j == corrupt) {
            noisy.push_back(f.views[j]);
            cv::randu(noisy.back().gray, 0, 255);  // a view that sees something else entirely
            nv.push_back(&noisy.back());
        } else {
            nv.push_back(&f.views[j]);
        }
    }
    const auto [near, far] = psom::depth_range(f.s, f.s.frames[i], f.center);
    return psom::plane_sweep(f.views[i], nv, near, far, f.p);
}

// median |mvs - raw| / raw over pixels both have
double median_rel_error(const psom::DepthMap& m, int frame) {
    psom::DepthMap raw;
    std::string why;
    EXPECT_TRUE(psom::load_raw_depth(fx().s, fx().s.frames[frame], raw, why)) << why;
    std::vector<double> e;
    for (int y = 0; y < m.K.h; ++y)
        for (int x = 0; x < m.K.w; ++x) {
            const float z = m.d[size_t(y) * m.K.w + x];
            if (z <= 0) continue;
            const int u = x * raw.K.w / m.K.w, v = y * raw.K.h / m.K.h;
            const float zr = raw.d[size_t(v) * raw.K.w + u];
            if (zr > 0) e.push_back(std::abs(z - zr) / zr);
        }
    if (e.empty()) return 1;
    std::nth_element(e.begin(), e.begin() + e.size() / 2, e.end());
    return e[e.size() / 2];
}

double valid_fraction(const psom::DepthMap& m) { return double(m.valid()) / m.d.size(); }

}  // namespace

TEST(Mvs, NeighboursHaveAUsefulBaseline) {
    Fixture& f = fx();
    const auto nb = psom::select_neighbours(f.s, 0, f.center, 4);
    ASSERT_EQ(nb.size(), 4u);
    for (int j : nb) {
        const cv::Vec3d a = f.s.frames[0].center() - f.center, b = f.s.frames[j].center() - f.center;
        const double deg = std::acos(a.dot(b) / cv::norm(a) / cv::norm(b)) * 180 / CV_PI;
        EXPECT_GT(deg, 3);
        EXPECT_LT(deg, 40);
    }
}

TEST(Mvs, PlaneSweepMatchesTheReferenceDepth) {
    const psom::DepthMap m = sweep(0);
    EXPECT_GT(valid_fraction(m), 0.5);
    EXPECT_LT(median_rel_error(m, 0), 0.01);
}

TEST(Mvs, SigmaGrowsWithDepth) {
    const psom::DepthMap m = sweep(0);
    std::vector<std::pair<float, float>> zs;
    for (size_t k = 0; k < m.d.size(); ++k)
        if (m.d[k] > 0) zs.push_back({m.d[k], m.sigma[k]});
    std::sort(zs.begin(), zs.end());
    auto med_sigma = [&](size_t a, size_t b) {
        std::vector<float> s;
        for (size_t k = a; k < b; ++k) s.push_back(zs[k].second);
        std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
        return s[s.size() / 2];
    };
    const size_t q = zs.size() / 4;
    EXPECT_GT(med_sigma(3 * q, zs.size()), med_sigma(0, q));
}

TEST(Mvs, AnOccludedOrUnrelatedNeighbourDropsOut) {
    Fixture& f = fx();
    const auto nb = psom::select_neighbours(f.s, 0, f.center, f.p.neighbours);
    const psom::DepthMap clean = sweep(0), bad = sweep(0, nb[0]);
    EXPECT_GT(valid_fraction(bad), 0.7 * valid_fraction(clean));
    EXPECT_LT(median_rel_error(bad, 0), 0.012);
}

TEST(Mvs, ConsistencyRejectsDepthTheOtherViewsDisagreeWith) {
    Fixture& f = fx();
    const auto nb = psom::select_neighbours(f.s, 0, f.center, f.p.neighbours);
    std::vector<psom::DepthMap> maps(f.s.frames.size());
    std::vector<cv::Matx44d> poses;
    for (auto& fr : f.s.frames) poses.push_back(fr.T_wc);
    std::vector<std::vector<int>> nbrs(maps.size());
    maps[0] = sweep(0);
    nbrs[0] = nb;
    for (int j : nb) maps[j] = sweep(j);
    // push a block of the reference 5 % further away: no neighbour can agree with it
    std::vector<size_t> injected, untouched;
    for (int y = 0; y < maps[0].K.h; ++y)
        for (int x = 0; x < maps[0].K.w; ++x) {
            const size_t k = size_t(y) * maps[0].K.w + x;
            if (maps[0].d[k] <= 0) continue;
            if (x >= 200 && x < 280 && y >= 200 && y < 280) maps[0].d[k] *= 1.05f, injected.push_back(k);
            else untouched.push_back(k);
        }
    ASSERT_GT(injected.size(), 1000u);
    psom::consistency_filter(maps, poses, nbrs, f.p);
    size_t killed = 0, kept = 0;
    for (size_t k : injected) killed += maps[0].d[k] == 0;
    for (size_t k : untouched) kept += maps[0].d[k] > 0;
    EXPECT_GT(double(killed) / injected.size(), 0.95);
    EXPECT_GT(double(kept) / untouched.size(), 0.6);
}
