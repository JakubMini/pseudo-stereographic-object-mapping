#include <gtest/gtest.h>

#include <cmath>
#include <map>

#include "psom/repair.h"
#include "psom/tsdf.h"
#include "util.h"

namespace {

const cv::Vec3d kC(0, 0.05, 0);
constexpr double kR = 0.03;

cv::Matx44d look_at(const cv::Vec3d& C, const cv::Vec3d& t) {
    const cv::Vec3d f = cv::normalize(t - C), r = cv::normalize(f.cross(cv::Vec3d(0, 1, 0))), d = f.cross(r);
    return {r[0], d[0], f[0], C[0], r[1], d[1], f[1], C[1], r[2], d[2], f[2], C[2], 0, 0, 0, 1};
}

// Exact depth of a sphere seen from T.
psom::DepthMap sphere_depth(const cv::Matx44d& T) {
    psom::DepthMap m;
    m.resize({200, 200, 99.5, 74.5, 200, 150});
    m.source = psom::DepthMap::Source::Raw;
    const cv::Matx33d R = T.get_minor<3, 3>(0, 0);
    const cv::Vec3d C(T(0, 3), T(1, 3), T(2, 3));
    for (int y = 0; y < 150; ++y)
        for (int x = 0; x < 200; ++x) {
            const cv::Vec3d rc = m.K.ray(x, y), d = cv::normalize(R * rc), oc = C - kC;
            const double b = oc.dot(d), c = oc.dot(oc) - kR * kR, disc = b * b - c;
            if (disc < 0) continue;
            const double t = -b - std::sqrt(disc);
            m.d[size_t(y) * 200 + x] = float(t / cv::norm(rc));  // along z
            m.sigma[size_t(y) * 200 + x] = 0.001f;
        }
    return m;
}

std::vector<cv::Matx44d> orbit() {
    std::vector<cv::Matx44d> poses;
    for (double el : {-40.0, 0.0, 40.0})
        for (int k = 0; k < 8; ++k) {
            const double e = el * CV_PI / 180, a = 2 * CV_PI * k / 8 + el;
            poses.push_back(look_at(kC + 0.25 * cv::Vec3d(std::cos(e) * std::cos(a), std::sin(e), std::cos(e) * std::sin(a)), kC));
        }
    return poses;
}

psom::CropBox box() {
    psom::CropBox b;
    b.center = kC;
    b.half = {0.04, 0.04, 0.04};
    return b;
}

}  // namespace

TEST(Tsdf, SphereFromAllSidesIsClosedAndRound) {
    psom::SparseTsdf t(box(), {0.001, 0.005, size_t(1) << 30, 0.5});
    for (auto& T : orbit()) ASSERT_TRUE(t.integrate(sphere_depth(T), T));
    const psom::Mesh m = t.extract();
    ASSERT_FALSE(m.f.empty());
    const psom::Topology topo = psom::validate(m);
    EXPECT_TRUE(topo.closed_valid) << (topo.reasons.empty() ? "" : topo.reasons[0]);
    double sum = 0, worst = 0;
    for (auto& v : m.v) {
        const double e = cv::norm(cv::Vec3d(v) - kC) - kR;
        sum += e;
        worst = std::max(worst, std::abs(e));
    }
    EXPECT_LT(std::abs(sum / m.v.size()), 0.0005);  // half a voxel
    EXPECT_LT(worst, 0.0015);
    ASSERT_TRUE(m.has_channels());
    for (size_t i = 0; i < m.v.size(); i += 97) {
        EXPECT_GT(m.weight[i], 0.5f);
        EXPECT_GT(m.sigma[i], 0.f);
        EXPECT_GE(m.diversity[i], 0.f);
        EXPECT_LE(m.diversity[i], 1.f);
    }
}

TEST(Tsdf, SeenFromOneSideStaysOpen) {
    psom::SparseTsdf t(box(), {0.001, 0.005, size_t(1) << 30, 0.5});
    const auto poses = orbit();
    for (int k = 16; k < 24; ++k) t.integrate(sphere_depth(poses[k]), poses[k]);  // only from above
    const psom::Topology topo = psom::validate(t.extract());
    EXPECT_FALSE(topo.closed_valid);
    EXPECT_GT(topo.boundary_edges, 0u);
}

TEST(Tsdf, MemoryBudgetIsEnforced) {
    const size_t budget = 10 * psom::SparseTsdf::block_bytes();
    psom::SparseTsdf t(box(), {0.001, 0.005, budget, 0.5});
    const auto poses = orbit();
    EXPECT_FALSE(t.integrate(sphere_depth(poses[0]), poses[0]));
    EXPECT_TRUE(t.over_budget());
    EXPECT_LE(t.blocks(), 10u);
    EXPECT_LE(t.bytes(), budget);
    EXPECT_TRUE(t.extract().f.empty());  // an incomplete TSDF is never meshed
}

TEST(Tsdf, UnseenSideIsGuessedClosedAndMarked) {
    psom::CropBox b = box();
    b.axes = cv::Matx33d(1, 0, 0, 0, 0, -1, 0, 1, 0);  // local z is up, as in the pipeline: the floor is below the sphere
    psom::SparseTsdf t(b, {0.001, 0.005, size_t(1) << 30, 0.5});
    const auto poses = orbit();
    for (size_t k = 8; k < 12; ++k) t.integrate(sphere_depth(poses[k]), poses[k]);  // half way round, level: the back never seen
    ASSERT_FALSE(psom::validate(t.extract()).closed_valid);

    const psom::SparseTsdf::FillReport rep = t.fill_unseen(size_t(256) << 20);
    EXPECT_GT(rep.voxels, 0u);
    EXPECT_FALSE(rep.budget_hit);
    const psom::Mesh m = t.extract();
    // closed everywhere except where the guess meets the floor, which the base cap closes on the table
    std::map<std::pair<int, int>, int> edges;
    for (auto& f : m.f)
        for (int k = 0; k < 3; ++k) ++edges[std::minmax(f[k], f[(k + 1) % 3])];
    size_t open = 0, off_floor = 0;
    for (auto& [e, n] : edges) {
        if (n == 2) continue;
        ++open;
        const double h = b.to_local(cv::Vec3d(m.v[e.first]))[2] + b.half[2];
        if (n != 1 || h > 0.002) ++off_floor;
    }
    EXPECT_EQ(off_floor, 0u) << open << " open edges";
    size_t guessed = 0;
    double worst_seen = 0, worst_guess = 0;
    for (size_t i = 0; i < m.v.size(); ++i) {
        const double e = std::abs(cv::norm(cv::Vec3d(m.v[i]) - kC) - kR);
        if (m.weight[i] < 0.25f) ++guessed, worst_guess = std::max(worst_guess, e);
        else worst_seen = std::max(worst_seen, e);
    }
    EXPECT_GT(guessed, m.v.size() / 50);
    EXPECT_LT(guessed, m.v.size() * 3 / 4);
    EXPECT_LT(worst_seen, 0.0015);   // what was seen is not moved by the guess
    // a membrane across the gap: flatter than the sphere, inside it, never further out than the radius
    EXPECT_LT(worst_guess, kR);
}
