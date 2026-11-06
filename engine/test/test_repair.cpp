#include <gtest/gtest.h>

#include <cmath>

#include "psom/repair.h"
#include "util.h"

using psom::Mesh;

namespace {

psom::Plane floor_plane() {
    psom::Plane p;
    p.n = {0, 1, 0};
    p.d = 0;
    p.found = true;
    return p;
}

// Subdivided icosahedron on a sphere: closed, manifold, genus 0.
Mesh icosphere(int levels, double r) {
    const double t = (1 + std::sqrt(5.0)) / 2;
    Mesh m;
    for (auto v : std::vector<cv::Vec3d>{{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                                         {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}})
        m.v.push_back(cv::Vec3f(cv::normalize(v) * r));
    m.f = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
           {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (int l = 0; l < levels; ++l) {
        std::map<std::pair<int, int>, int> mid;
        auto split = [&](int a, int b) {
            auto k = std::minmax(a, b);
            auto it = mid.find(k);
            if (it != mid.end()) return it->second;
            m.v.push_back(cv::Vec3f(cv::normalize(cv::Vec3d(m.v[a]) + cv::Vec3d(m.v[b])) * r));
            return mid[k] = int(m.v.size()) - 1;
        };
        std::vector<cv::Vec3i> next;
        for (auto& f : m.f) {
            const int a = split(f[0], f[1]), b = split(f[1], f[2]), c = split(f[2], f[0]);
            next.insert(next.end(), {{f[0], a, c}, {f[1], b, a}, {f[2], c, b}, {a, b, c}});
        }
        m.f.swap(next);
    }
    return m;
}

}  // namespace

TEST(Topology, ClosedCubeIsValidAndOutward) {
    const psom::Topology t = psom::validate(cube(0.05));
    EXPECT_TRUE(t.closed_valid);
    EXPECT_EQ(t.components, 1u);
    EXPECT_NEAR(t.signed_volume, 0.05 * 0.05 * 0.05, 1e-9);
}

TEST(Topology, OpenSurfaceIsReported) {
    const psom::Topology t = psom::validate(cube(0.05, {0, 0, 0}, {"top"}));
    EXPECT_FALSE(t.closed_valid);
    EXPECT_EQ(t.boundary_edges, 4u);
}

TEST(Topology, FlippedFaceIsInconsistent) {
    Mesh m = cube(0.05);
    std::swap(m.f[3][1], m.f[3][2]);
    const psom::Topology t = psom::validate(m);
    EXPECT_FALSE(t.closed_valid);
    EXPECT_GT(t.inconsistent_edges, 0u);
}

TEST(Topology, FinMakesANonManifoldEdge) {
    Mesh m = cube(0.05);
    m.v.emplace_back(0.025f, 0.1f, -0.05f);
    m.f.emplace_back(0, 2, int(m.v.size()) - 1);  // a third face on edge 0-2
    const psom::Topology t = psom::validate(m);
    EXPECT_FALSE(t.closed_valid);
    EXPECT_GT(t.nonmanifold_edges, 0u);
}

TEST(Topology, CubesTouchingAtACornerMakeANonManifoldVertex) {
    Mesh a = cube(0.05), b = cube(0.05, {0.05, 0.05, 0.05});
    // weld b's corner 0 onto a's corner 7
    const int off = int(a.v.size());
    for (auto& v : b.v) a.v.push_back(v);
    for (auto f : b.f) {
        for (int k = 0; k < 3; ++k) f[k] = f[k] == 0 ? 7 : f[k] + off;
        a.f.push_back(f);
    }
    a.compact();
    const psom::Topology t = psom::validate(a);
    EXPECT_FALSE(t.closed_valid);
    EXPECT_EQ(t.nonmanifold_vertices, 1u);
}

TEST(Repair, CapsTheBaseOnTheSupportPlane) {
    Mesh m = cube(0.05, {0, 0, 0}, {"bottom"});
    psom::RepairParams p;
    const psom::RepairReport r = psom::repair(m, floor_plane(), p);
    EXPECT_TRUE(r.capped);
    EXPECT_NEAR(r.cap_area, 0.05 * 0.05, 1e-9);
    EXPECT_TRUE(psom::validate(m).closed_valid);
}

TEST(Repair, LeavesALargeUnseenHoleOpen) {
    Mesh m = cube(0.05, {0, 0, 0}, {"top"});
    const psom::RepairReport r = psom::repair(m, floor_plane(), psom::RepairParams{});
    EXPECT_FALSE(r.capped);
    EXPECT_EQ(r.holes_open, 1);
    EXPECT_FALSE(r.warnings.empty());
    EXPECT_FALSE(psom::validate(m).closed_valid);
}

TEST(Repair, DoesNotCapALoopAboveThePlane) {
    Mesh m = cube(0.05, {0, 0.02, 0}, {"bottom"});  // floats 2 cm above the table
    const psom::RepairReport r = psom::repair(m, floor_plane(), psom::RepairParams{});
    EXPECT_FALSE(r.capped);
    EXPECT_EQ(r.holes_open, 1);
}

TEST(Repair, FillsASmallHole) {
    Mesh m = cube(0.005, {0, 0.1, 0}, {"top"});  // 20 mm rim, below the limit
    const psom::RepairReport r = psom::repair(m, floor_plane(), psom::RepairParams{});
    EXPECT_EQ(r.holes_filled, 1);
    EXPECT_TRUE(psom::validate(m).closed_valid);
}

TEST(Repair, DecimationKeepsAClosedManifold) {
    Mesh m = icosphere(5, 0.03);  // 20480 faces
    ASSERT_TRUE(psom::validate(m).closed_valid);
    const double v0 = psom::signed_volume(m);
    psom::RepairParams p;
    p.face_budget = 2000;
    p.max_decimation_error = 0.001;
    const psom::RepairReport r = psom::repair(m, psom::Plane{}, p);
    EXPECT_TRUE(r.decimated);
    EXPECT_LE(m.f.size(), 2400u);
    const psom::Topology t = psom::validate(m);
    EXPECT_TRUE(t.closed_valid) << (t.reasons.empty() ? "" : t.reasons[0]);
    EXPECT_NEAR(psom::signed_volume(m), v0, 0.02 * v0);
}

TEST(Repair, BoundaryLoopsFollowTheWinding) {
    const Mesh m = cube(0.05, {0, 0, 0}, {"top"});
    const auto loops = psom::boundary_loops(m);
    ASSERT_EQ(loops.size(), 1u);
    EXPECT_EQ(loops[0].size(), 4u);
}

TEST(Repair, DropsAPieceFloatingAboveTheTableButKeepsOneStandingOnIt) {
    auto at = [](Mesh m, cv::Vec3d o) {
        for (auto& v : m.v) v += cv::Vec3f(float(o[0]), float(o[1]), float(o[2]));
        return m;
    };
    auto add = [](Mesh& a, const Mesh& b) {
        const int off = int(a.v.size());
        for (auto& v : b.v) a.v.push_back(v);
        for (auto f : b.f) a.f.push_back(f + cv::Vec3i(off, off, off));
    };
    Mesh m = at(icosphere(3, 0.03), {0, 0.03, 0});  // the object, on the table
    add(m, at(icosphere(1, 0.01), {0, 0.1, 0}));      // a speck in the air, ~6 % of the faces
    const psom::RepairReport r = psom::repair(m, floor_plane(), psom::RepairParams{});
    EXPECT_EQ(r.components_removed, 1);
    EXPECT_EQ(psom::validate(m).components, 1u);

    Mesh two = at(icosphere(3, 0.03), {0, 0.03, 0});
    add(two, at(icosphere(1, 0.01), {0.08, 0.01, 0}));  // the same size, standing beside it
    psom::repair(two, floor_plane(), psom::RepairParams{});
    EXPECT_EQ(psom::validate(two).components, 2u);
}
