// psom: the desktop face of the engine.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "psom/pipeline.h"
#include "psom/session.h"
#include "psom/sfm.h"
#include "psom/synth.h"

using json = nlohmann::json;

namespace {

int usage() {
    std::fprintf(stderr,
                 "usage:\n"
                 "  psom synth <project> [--shape box|cylinder] [--dims L,W,H] [--no-depth] [--photos] [--low-texture]\n"
                 "             [--tracking-loss] [--interrupted] [--frames N] [--pose-noise MM] [--seed N]\n"
                 "  psom reconstruct <project> [--force-mvs] [--mem-mb N] [--faces N] [--mvs-width PX] [--mvs-window PX] [--verbose]\n"
                 "  psom poses <project> [--verbose]     photo scans: camera poses only, nothing written\n"
                 "  psom export <project>\n"
                 "  psom evaluate <project> <truth.json>\n");
    return 2;
}

int report(const psom::Result& r, const std::string& extra = {}) {
    json j = {{"ok", r.ok}, {"stage", r.stage}, {"error", r.error}, {"warnings", r.warnings}};
    if (!extra.empty()) j["result"] = json::parse(extra);
    std::cout << j.dump(2) << std::endl;
    return r.ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) return usage();
    const std::string cmd = argv[1];
    const std::filesystem::path project = argv[2];
    auto flag = [&](const char* name) {
        for (int i = 3; i < argc; ++i)
            if (!std::strcmp(argv[i], name)) return true;
        return false;
    };
    auto value = [&](const char* name) -> const char* {
        for (int i = 3; i + 1 < argc; ++i)
            if (!std::strcmp(argv[i], name)) return argv[i + 1];
        return nullptr;
    };
    const auto progress = [](const std::string& stage, double f) {
        std::fprintf(stderr, "{\"event\":\"progress\",\"stage\":\"%s\",\"fraction\":%.2f}\n", stage.c_str(), f);
    };

    if (cmd == "synth") {
        psom::SynthSpec s;
        if (auto v = value("--shape")) s.shape = v;
        if (auto v = value("--dims")) {
            if (std::sscanf(v, "%lf,%lf,%lf", &s.dims_mm[0], &s.dims_mm[1], &s.dims_mm[2]) != 3) return usage();
        }
        if (auto v = value("--frames")) s.frames = std::atoi(v);
        if (auto v = value("--pose-noise")) s.pose_noise_mm = std::atof(v);
        if (auto v = value("--seed")) s.seed = uint32_t(std::atoi(v));
        s.depth = !flag("--no-depth");
        s.photos = flag("--photos");
        s.low_texture = flag("--low-texture");
        s.tracking_loss = flag("--tracking-loss");
        s.interrupted = flag("--interrupted");
        if (s.shape != "box" && s.shape != "cylinder") return usage();
        return report(psom::synth_project(project, s));
    }
    if (cmd == "reconstruct") {
        psom::Options o;
        o.force_mvs = flag("--force-mvs");
        o.verbose = flag("--verbose");
        if (auto v = value("--mem-mb")) o.mem_mb = size_t(std::atoll(v));
        if (auto v = value("--faces")) o.face_budget = size_t(std::atoll(v));
        if (auto v = value("--mvs-width")) o.mvs.width = std::atoi(v);
        if (auto v = value("--mvs-window")) o.mvs.window = o.mvs.window_shrunk = std::atoi(v);
        return report(psom::reconstruct(project, o, progress));
    }
    if (cmd == "export") return report(psom::export_project(project, progress));
    if (cmd == "poses") {
        psom::Session s;
        psom::Result r = psom::load_session(project / "session", s);
        if (!r.ok) return report(r);
        psom::SfmParams p;
        p.verbose = flag("--verbose");
        p.refine_focal = !s.focal_calibrated;
        if (auto v = value("--max-side")) p.max_side = std::atoi(v);
        if (auto v = value("--features")) p.max_features = std::atoi(v);
        if (auto v = value("--window")) p.window = std::atoi(v);
        if (auto v = value("--growth")) p.ba_growth = std::atof(v);
        psom::SfmResult out;
        r = psom::run_sfm(s, p, out);
        json j = {{"placed", out.frames.size()}, {"photos", s.frames.size()}, {"rms_px", out.rms_px}, {"focal_px", out.K.fx},
                  {"points", out.points.size()}};
        return report(r, j.dump());
    }
    if (cmd == "evaluate") {
        if (argc < 4) return usage();
        std::string summary;
        const psom::Result r = psom::evaluate(project, argv[3], summary);
        return report(r, summary);
    }
    return usage();
}
