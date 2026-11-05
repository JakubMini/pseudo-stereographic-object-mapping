// Poses from photos alone. Synthetic gates again: real-phone runs are measured separately.
#include <gtest/gtest.h>

#include <random>

#include <nlohmann/json.hpp>

#include "psom/pipeline.h"
#include "psom/sfm.h"
#include "util.h"

using json = nlohmann::json;

namespace {

psom::SynthSpec box_photos() {
    psom::SynthSpec s;
    s.photos = true;
    return s;
}

json read(const fs::path& p) { return json::parse(slurp(p)); }

cv::Matx33d rot(const cv::Vec3d& w) {
    cv::Matx33d R = cv::Matx33d::eye();
    const double th = cv::norm(w);
    if (th < 1e-12) return R;
    const cv::Vec3d a = w / th;
    const cv::Matx33d K(0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0);
    return R + std::sin(th) * K + (1 - std::cos(th)) * (K * K);
}

}  // namespace

TEST(Sfm, BundleAdjustmentRecoversPosesPointsAndFocal) {
    std::mt19937 rng(3);
    std::normal_distribution<double> g(0, 1);
    std::uniform_real_distribution<double> u(-0.1, 0.1);
    psom::Intrinsics truthK{500, 500, 320, 240, 640, 480};
    std::vector<cv::Matx33d> R;
    std::vector<cv::Vec3d> t, X;
    for (int c = 0; c < 8; ++c) {  // an arc around the origin, looking at it
        const double a = c * 0.25;
        const cv::Vec3d C(0.4 * std::sin(a), 0.1, -0.4 * std::cos(a));
        const cv::Vec3d f = cv::normalize(-C), r = cv::normalize(f.cross(cv::Vec3d(0, 1, 0))), d = f.cross(r);
        const cv::Matx33d Rc(r[0], r[1], r[2], d[0], d[1], d[2], f[0], f[1], f[2]);  // camera-from-world
        R.push_back(Rc);
        t.push_back(-(Rc * C));
    }
    for (int k = 0; k < 300; ++k) X.emplace_back(u(rng), u(rng), u(rng));
    std::vector<psom::BaObs> obs;
    for (int c = 0; c < 8; ++c)
        for (int k = 0; k < 300; ++k) {
            const cv::Vec3d p = R[c] * X[k] + t[c];
            const cv::Vec2d uv = truthK.project(p) + cv::Vec2d(g(rng), g(rng)) * 0.3;
            obs.push_back({c, k, uv});
        }
    // perturb everything but camera 0, and the focal length by 6 %
    auto R2 = R;
    auto t2 = t;
    auto X2 = X;
    for (int c = 1; c < 8; ++c) R2[c] = rot({g(rng) * 0.01, g(rng) * 0.01, g(rng) * 0.01}) * R[c], t2[c] += cv::Vec3d(g(rng), g(rng), g(rng)) * 0.005;
    for (auto& x : X2) x += cv::Vec3d(g(rng), g(rng), g(rng)) * 0.003;
    psom::Intrinsics K = truthK;
    K.fx = K.fy = 530;
    std::vector<char> in;
    const double rms = psom::bundle_adjust(R2, t2, X2, K, obs, true, 50, 3.0, &in);
    EXPECT_LT(rms, 0.5);
    EXPECT_NEAR(K.fx, 500, 5);
    // only up to scale: compare the camera centres' shape, normalised by the 0-1 baseline
    auto centre = [](const cv::Matx33d& r, const cv::Vec3d& tt) { return -(r.t() * tt); };
    const double s = cv::norm(centre(R[1], t[1]) - centre(R[0], t[0])) / cv::norm(centre(R2[1], t2[1]) - centre(R2[0], t2[0]));
    for (int c = 2; c < 8; ++c)
        EXPECT_LT(cv::norm(centre(R2[c], t2[c]) * s - centre(R[c], t[c])), 0.01) << c;
}

TEST(Sfm, GravityInDeviceAxesBecomesUpInTheCamera) {
    // Phone upright in portrait: up is +y on the device. A 90-degree sensor's raw image has its
    // left edge at the top, so up in the camera is -x.
    const cv::Vec3d a = psom::device_up_to_camera({0, 9.81, 0}, 90);
    EXPECT_NEAR(a[0], -1, 1e-9);
    // flat on its back, camera looking down: up is behind the camera
    const cv::Vec3d b = psom::device_up_to_camera({0, 0, 9.81}, 90);
    EXPECT_NEAR(b[2], -1, 1e-9);
    // sensor in the device's own orientation: up is -y in the image (which points down)
    const cv::Vec3d c = psom::device_up_to_camera({0, 9.81, 0}, 0);
    EXPECT_NEAR(c[1], -1, 1e-9);
}

TEST(Sfm, PhotosOfABoxArePlacedWhereTheyWereTaken) {
    const fs::path p = synth_copy("box_photos", box_photos(), "sfm_box");
    psom::Session s;
    ASSERT_TRUE(psom::load_session(p / "session", s).ok);
    ASSERT_TRUE(s.photos);
    psom::SfmResult r;
    const psom::Result res = psom::run_sfm(s, psom::SfmParams{}, r);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(r.frames.size(), s.frames.size());
    EXPECT_LT(r.rms_px, 0.6);
    EXPECT_NEAR(r.K.fx, 500, 5) << "the session's guess was 540";

    // up from gravity, and a sensible working scale
    ASSERT_TRUE(psom::apply_sfm(s, r).ok);
    std::vector<cv::Matx44d> truth;
    for (auto& l : read_lines(p / "session" / "frames.jsonl")) {
        const json j = json::parse(l);
        cv::Matx44d T;
        for (int k = 0; k < 16; ++k) T.val[k] = j["truth_pose"][k].get<double>();
        truth.push_back(psom::gl_to_cv(T));
    }
    for (size_t i = 0; i < s.frames.size(); ++i) {
        const cv::Vec3d up_true = truth[i].get_minor<3, 3>(0, 0).t() * cv::Vec3d(0, 1, 0);
        const cv::Vec3d up = s.frames[i].R().t() * cv::Vec3d(0, 1, 0);
        EXPECT_GT(up.dot(up_true), std::cos(CV_PI / 180)) << "frame " << i << " is tilted by more than 1 degree";
    }
    // the orbit's shape: ratio of distances between camera centres matches the truth
    const double s_true = cv::norm(cv::Vec3d(truth[0](0, 3), truth[0](1, 3), truth[0](2, 3)) -
                                   cv::Vec3d(truth[12](0, 3), truth[12](1, 3), truth[12](2, 3)));
    const double s_est = cv::norm(s.frames[0].center() - s.frames[12].center());
    for (size_t i = 1; i < s.frames.size(); i += 5) {
        const double dt = cv::norm(cv::Vec3d(truth[0](0, 3), truth[0](1, 3), truth[0](2, 3)) -
                                   cv::Vec3d(truth[i](0, 3), truth[i](1, 3), truth[i](2, 3))) / s_true;
        const double de = cv::norm(s.frames[0].center() - s.frames[i].center()) / s_est;
        EXPECT_NEAR(de, dt, 0.01) << i;
    }
}

TEST(Sfm, PhotoScanNeedsTheScaleBeforeExportThenMeasuresTheBox) {
    const fs::path p = synth_copy("box_photos", box_photos(), "sfm_e2e");
    const psom::Result r = psom::reconstruct(p, psom::Options{});
    ASSERT_TRUE(r.ok) << r.stage << ": " << r.error;
    const json rep = read(p / "recon" / "report.json");
    EXPECT_EQ(rep["poses"]["source"], "photos");
    EXPECT_FALSE(rep["dimensions_mm"]["scale_known"].get<bool>());
    EXPECT_TRUE(rep["topology"]["closed_valid"].get<bool>()) << rep["topology"].dump();

    // A STEP in millimetres would claim a measurement nobody made.
    const psom::Result refused = psom::export_project(p);
    EXPECT_FALSE(refused.ok);
    EXPECT_NE(refused.error.find("scale"), std::string::npos);

    // The user measures one length (the box's 100 mm side); the other two must then follow.
    const double L = rep["dimensions_mm"]["length"], W = rep["dimensions_mm"]["width"], H = rep["dimensions_mm"]["height"];
    json proj = read(p / "project.json");
    proj["scale"] = {{"factor", 100.0 / L}, {"a", {0, 0, 0}}, {"b", {L / 1000, 0, 0}}, {"known_mm", 100.0}};
    std::ofstream(p / "project.json") << proj.dump(2);
    EXPECT_NEAR(W * 100.0 / L, 60, 4);
    EXPECT_NEAR(H * 100.0 / L, 40, 4);
    const psom::Result e = psom::export_project(p);
    ASSERT_TRUE(e.ok) << e.error;
}
