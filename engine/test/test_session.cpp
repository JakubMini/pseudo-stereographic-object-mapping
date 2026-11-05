#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "psom/session.h"
#include "util.h"

using json = nlohmann::json;

namespace {

psom::SynthSpec small() {
    psom::SynthSpec s;
    s.frames = 8;
    s.width = 160;
    s.height = 120;
    return s;
}

bool has_warning(const psom::Result& r, const std::string& needle) {
    for (auto& w : r.warnings)
        if (w.find(needle) != std::string::npos) return true;
    return false;
}

}  // namespace

TEST(Session, GlToCvFlipsCameraYAndZ) {
    const cv::Matx44d T = cv::Matx44d::eye();
    const cv::Matx44d C = psom::gl_to_cv(T);
    EXPECT_EQ(C(1, 1), -1);
    EXPECT_EQ(C(2, 2), -1);
    EXPECT_EQ(C(0, 0), 1);
    const cv::Matx44d back = psom::gl_to_cv(C);
    for (int k = 0; k < 16; ++k) EXPECT_DOUBLE_EQ(back.val[k], T.val[k]);
}

TEST(Session, LoadsAndCamerasLookAtTheObject) {
    const fs::path p = synth_copy("small", small(), "load");
    psom::Session s;
    const psom::Result r = psom::load_session(p / "session", s);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(s.frames.size(), 8u);
    EXPECT_TRUE(s.capture_ended);
    EXPECT_TRUE(s.depth_usable);
    // the synthetic camera looks at (0, h/2, 0); after GL->CV, +z (forward) must point there
    const cv::Vec3d target(0, 0.02, 0);
    for (auto& f : s.frames) {
        const cv::Vec3d to = cv::normalize(target - f.center());
        EXPECT_GT(to.dot(f.forward()), 0.999);
        EXPECT_TRUE(f.has_depth);
    }
}

TEST(Session, TruncatedLastLineIsDropped) {
    psom::SynthSpec sp = small();
    sp.interrupted = true;
    const fs::path p = synth_copy("small_cut", sp, "cut");
    psom::Session s;
    const psom::Result r = psom::load_session(p / "session", s);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(s.frames.size(), 7u);
    EXPECT_FALSE(s.capture_ended);
    EXPECT_TRUE(has_warning(r, "truncated"));
    EXPECT_TRUE(has_warning(r, "interrupted"));
}

TEST(Session, BadFramesAreDroppedWithReasons) {
    const fs::path p = synth_copy("small", small(), "bad");
    auto lines = read_lines(p / "session" / "frames.jsonl");
    ASSERT_EQ(lines.size(), 8u);
    json l1 = json::parse(lines[1]), l2 = json::parse(lines[2]), l3 = json::parse(lines[3]), l4 = json::parse(lines[4]),
         l5 = json::parse(lines[5]);
    l2["t_ns"] = l1["t_ns"];                                    // not increasing
    for (int k : {0, 1, 2}) l3["pose"][k] = l3["pose"][k].get<double>() * 1.1;  // not orthonormal
    l4["K"][2] = -5.0;                                          // principal point outside
    l5["depth_t_ns"] = l5["t_ns"].get<int64_t>() + 50'000'000;  // depth 50 ms off
    lines[2] = l2.dump();
    lines[3] = l3.dump();
    lines[4] = l4.dump();
    lines[5] = l5.dump();
    write_lines(p / "session" / "frames.jsonl", lines);

    psom::Session s;
    const psom::Result r = psom::load_session(p / "session", s);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(s.frames.size(), 5u);
    EXPECT_TRUE(has_warning(r, "timestamp not increasing"));
    EXPECT_TRUE(has_warning(r, "not orthonormal"));
    EXPECT_TRUE(has_warning(r, "principal point outside"));
    EXPECT_EQ(s.depth_dropped_skew, 1);
    for (auto& f : s.frames) EXPECT_EQ(f.has_depth, f.i != 5);
}

TEST(Session, DepthWithTheWrongAspectIsIgnored) {
    const fs::path p = synth_copy("small", small(), "aspect");
    json meta = json::parse(slurp(p / "session" / "session.json"));
    meta["depth"]["width"] = 150;
    std::ofstream(p / "session" / "session.json") << meta.dump();
    psom::Session s;
    const psom::Result r = psom::load_session(p / "session", s);
    ASSERT_TRUE(r.ok);
    EXPECT_FALSE(s.depth_usable);
    EXPECT_TRUE(has_warning(r, "alignment"));
    for (auto& f : s.frames) EXPECT_FALSE(f.has_depth);
}

TEST(Session, UnknownConventionIsRefused) {
    const fs::path p = synth_copy("small", small(), "conv");
    json meta = json::parse(slurp(p / "session" / "session.json"));
    meta["conventions"]["pose"] = "something_else";
    std::ofstream(p / "session" / "session.json") << meta.dump();
    psom::Session s;
    const psom::Result r = psom::load_session(p / "session", s);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.stage, "load");
}

TEST(Session, TooFewFramesFails) {
    const fs::path p = synth_copy("small", small(), "few");
    auto lines = read_lines(p / "session" / "frames.jsonl");
    lines.resize(2);
    write_lines(p / "session" / "frames.jsonl", lines);
    psom::Session s;
    EXPECT_FALSE(psom::load_session(p / "session", s).ok);
}

TEST(Session, LateDepthIsKeptWithItsOwnPose) {
    const fs::path p = synth_copy("small", small(), "late");
    auto lines = read_lines(p / "session" / "frames.jsonl");
    json l3 = json::parse(lines[3]);
    l3["depth_t_ns"] = l3["t_ns"].get<int64_t>() - 66'000'000;  // two frames late
    json pose = json::parse(lines[2])["pose"];                  // the pose of the frame it came from
    l3["depth_pose"] = pose;
    lines[3] = l3.dump();
    json l4 = json::parse(lines[4]);
    l4["depth_t_ns"] = l4["t_ns"].get<int64_t>() - 66'000'000;  // late and no pose: still dropped
    lines[4] = l4.dump();
    write_lines(p / "session" / "frames.jsonl", lines);

    psom::Session s;
    ASSERT_TRUE(psom::load_session(p / "session", s).ok);
    EXPECT_TRUE(s.frames[3].has_depth);
    EXPECT_TRUE(s.frames[3].depth_has_pose);
    EXPECT_NEAR(cv::norm(s.frames[3].T_wc_depth.get_minor<3, 1>(0, 3) - s.frames[2].T_wc.get_minor<3, 1>(0, 3)), 0, 1e-9);
    EXPECT_FALSE(s.frames[4].has_depth);
    EXPECT_EQ(s.depth_dropped_skew, 1);
}
