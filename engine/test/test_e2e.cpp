// Synthetic scans through the whole pipeline. These gates are necessary, not sufficient: they say
// nothing about real phones, whose errors are measured separately.
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "psom/pipeline.h"
#include "psom/step.h"
#include "util.h"

using json = nlohmann::json;

namespace {

json read(const fs::path& p) { return json::parse(slurp(p)); }

void write(const fs::path& p, const json& j) { std::ofstream(p) << j.dump(2); }

bool has(const psom::Result& r, const std::string& needle) {
    for (auto& w : r.warnings)
        if (w.find(needle) != std::string::npos) return true;
    return false;
}

psom::SynthSpec box_depth() { return {}; }

struct Gate {
    std::string name, shape;
    cv::Vec3d dims;
    bool depth;
    double tol_mm;
};

class SyntheticGate : public ::testing::TestWithParam<Gate> {};

}  // namespace

TEST_P(SyntheticGate, ReconstructsWithinToleranceAndExportsASolid) {
    const Gate g = GetParam();
    psom::SynthSpec sp;
    sp.shape = g.shape;
    sp.dims_mm = g.dims;
    sp.depth = g.depth;
    const fs::path p = synth_copy(g.name, sp, "gate_" + g.name);

    const psom::Result r = psom::reconstruct(p, psom::Options{});
    ASSERT_TRUE(r.ok) << r.stage << ": " << r.error;
    const json rep = read(p / "recon" / "report.json");
    EXPECT_EQ(rep["depth"][g.depth ? "raw_frames" : "mvs_frames"].get<int>(), sp.frames);
    EXPECT_TRUE(rep["topology"]["closed_valid"].get<bool>()) << rep["topology"].dump();
    EXPECT_LE(rep["topology"]["faces"].get<int>(), 20000);

    const psom::Result e = psom::export_project(p);
    ASSERT_TRUE(e.ok) << e.error;
    const psom::StepCheck c = psom::check_step(p / "recon" / "model.step");
    EXPECT_TRUE(c.valid);
    EXPECT_EQ(c.solids, 1);
    EXPECT_EQ(read(p / "project.json")["status"], "exported");

    write(p / "truth.json", {{"name", g.name}, {"synthetic", true}, {"dims_mm", {g.dims[0], g.dims[1], g.dims[2]}}});
    std::string summary;
    ASSERT_TRUE(psom::evaluate(p, p / "truth.json", summary).ok);
    const json ev = json::parse(summary);
    EXPECT_LE(ev["max_abs_error_mm"].get<double>(), g.tol_mm) << summary;
}

INSTANTIATE_TEST_SUITE_P(Shapes, SyntheticGate,
                         ::testing::Values(Gate{"box_depth", "box", {100, 60, 40}, true, 2.0},
                                           Gate{"box_mvs", "box", {100, 60, 40}, false, 4.0},
                                           Gate{"cylinder_depth", "cylinder", {50, 50, 80}, true, 2.0},
                                           Gate{"cylinder_mvs", "cylinder", {50, 50, 80}, false, 4.0}),
                         [](const auto& info) { return info.param.name; });

TEST(Pipeline, LowTextureLeavesHolesInsteadOfInventingSurface) {
    psom::SynthSpec sp;
    sp.depth = false;
    sp.low_texture = true;
    const fs::path p = synth_copy("lowtex", sp, "lowtex");
    const psom::Result r = psom::reconstruct(p, psom::Options{});
    if (!r.ok) {
        // failing outright is also honest, as long as it says where
        EXPECT_FALSE(r.stage.empty());
        EXPECT_FALSE(read(p / "project.json")["last_error"].is_null());
        return;
    }
    EXPECT_TRUE(has(r, "low texture"));
    const json rep = read(p / "recon" / "report.json");
    EXPECT_FALSE(rep["topology"]["closed_valid"].get<bool>());
    EXPECT_GT(rep["repair"]["holes_left_open"].get<int>(), 0);
    ASSERT_TRUE(psom::export_project(p).ok);
    EXPECT_EQ(psom::check_step(p / "recon" / "model.step").solids, 0);
}

TEST(Pipeline, TrackingLossIsReported) {
    psom::SynthSpec sp;
    sp.tracking_loss = true;
    const fs::path p = synth_copy("tracking", sp, "tracking");
    const psom::Result r = psom::reconstruct(p, psom::Options{});
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_TRUE(has(r, "tracking was lost"));
    EXPECT_EQ(read(p / "recon" / "report.json")["session"]["tracking_loss"].size(), 1u);
}

TEST(Pipeline, InterruptedCaptureIsRecovered) {
    psom::SynthSpec sp;
    sp.interrupted = true;
    const fs::path p = synth_copy("interrupted", sp, "interrupted");
    EXPECT_EQ(read(p / "project.json")["status"], "capturing");
    const psom::Result r = psom::reconstruct(p, psom::Options{});
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_TRUE(has(r, "recovered an interrupted capture"));
    EXPECT_EQ(read(p / "project.json")["status"], "reconstructed");
}

TEST(Pipeline, FailureKeepsTheProjectAndRetryWorks) {
    const fs::path p = synth_copy("box_depth", box_depth(), "retry");
    const std::string before = tree_digest(p / "session");
    psom::Options tiny;
    tiny.mem_mb = 5;
    const psom::Result r = psom::reconstruct(p, tiny);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.stage, "fusion");
    EXPECT_NE(r.error.find("memory budget"), std::string::npos);
    EXPECT_EQ(tree_digest(p / "session"), before);  // the capture is untouched
    json proj = read(p / "project.json");
    EXPECT_EQ(proj["status"], "captured");
    EXPECT_EQ(proj["last_error"]["stage"], "fusion");
    EXPECT_FALSE(fs::exists(p / "recon" / "mesh.ply"));

    const psom::Result again = psom::reconstruct(p, psom::Options{});
    ASSERT_TRUE(again.ok) << again.error;
    proj = read(p / "project.json");
    EXPECT_EQ(proj["status"], "reconstructed");
    EXPECT_TRUE(proj["last_error"].is_null());
    EXPECT_TRUE(read(p / "recon" / "report.json")["depth"]["cached"].get<bool>());  // depth reused
    EXPECT_EQ(proj["runs"].size(), 2u);
}

TEST(Pipeline, ExportNeedsAReconstruction) {
    const fs::path p = synth_copy("box_depth", box_depth(), "early_export");
    const psom::Result r = psom::export_project(p);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.stage, "export");
    EXPECT_FALSE(fs::exists(p / "recon" / "model.step"));
    EXPECT_EQ(read(p / "project.json")["status"], "captured");
}

TEST(Pipeline, ACorruptMeshIsNotExported) {
    const fs::path p = synth_copy("box_depth", box_depth(), "corrupt");
    ASSERT_TRUE(psom::reconstruct(p, psom::Options{}).ok);
    std::string ply = slurp(p / "recon" / "mesh.ply");
    ply.resize(ply.size() / 2);
    std::ofstream(p / "recon" / "mesh.ply", std::ios::binary) << ply;
    EXPECT_FALSE(psom::export_project(p).ok);
    EXPECT_FALSE(fs::exists(p / "recon" / "model.step"));
    EXPECT_EQ(read(p / "project.json")["status"], "reconstructed");
}

TEST(Pipeline, CropAndScaleChangeWithoutRescanning) {
    const fs::path p = synth_copy("box_depth", box_depth(), "edit");
    const std::string session = tree_digest(p / "session");
    ASSERT_TRUE(psom::reconstruct(p, psom::Options{}).ok);
    ASSERT_TRUE(psom::export_project(p).ok);
    const json first = read(p / "recon" / "report.json");

    // recalibrate: the same mesh, 10 % larger
    json proj = read(p / "project.json");
    proj["scale"]["factor"] = 1.1;
    write(p / "project.json", proj);
    ASSERT_TRUE(psom::export_project(p).ok);
    const json scaled = read(p / "recon" / "report.json");
    for (int k = 0; k < 3; ++k)
        EXPECT_NEAR(scaled["export"]["bbox_mm"][k].get<double>(), 1.1 * first["export"]["bbox_mm"][k].get<double>(), 0.01);

    // re-crop: cut the box at 25 mm; the top is then missing, so it must come out open
    proj = read(p / "project.json");
    proj["scale"]["factor"] = 1.0;
    json crop = proj["crop"];
    const cv::Vec3d n(crop["axes"][2][0], crop["axes"][2][1], crop["axes"][2][2]);
    const double hz = crop["half"][2], top = crop["center"][0].get<double>() * 0;  // silence unused warnings
    (void)top;
    const double z0 = -hz;                       // crop bottom in box frame
    const double new_half = (0.025 - 0.003) / 2;  // from the floor (3 mm) to 25 mm
    const double shift = (z0 + new_half) - 0.0;   // new centre relative to the old one, along n
    for (int k = 0; k < 3; ++k) crop["center"][k] = crop["center"][k].get<double>() + n[k] * shift;
    crop["half"][2] = new_half;
    crop["source"] = "user";
    proj["crop"] = crop;
    write(p / "project.json", proj);
    const psom::Result r = psom::reconstruct(p, psom::Options{});
    ASSERT_TRUE(r.ok) << r.error;
    const json cut = read(p / "recon" / "report.json");
    EXPECT_NEAR(cut["dimensions_mm"]["height"].get<double>(), 25.0, 1.5);
    EXPECT_FALSE(cut["topology"]["closed_valid"].get<bool>());
    EXPECT_EQ(read(p / "project.json")["crop"]["source"], "user");  // kept, not replaced by auto
    EXPECT_EQ(tree_digest(p / "session"), session);
}
