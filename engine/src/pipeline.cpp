#include "psom/pipeline.h"

#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "psom/depth.h"
#include "psom/isolate.h"
#include "psom/repair.h"
#include "psom/session.h"
#include "psom/sfm.h"
#include "psom/step.h"
#include "psom/tsdf.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace psom {

namespace {

std::string now_utc() {
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char b[32];
    std::strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
    return b;
}

bool write_json(const fs::path& p, const json& j) {
    const fs::path tmp = p.string() + ".tmp";
    {
        std::ofstream o(tmp);
        if (!o) return false;
        o << j.dump(2) << "\n";
        if (!o) return false;
    }
    std::error_code ec;
    fs::rename(tmp, p, ec);
    return !ec;
}

json read_json(const fs::path& p) {
    std::ifstream in(p);
    if (!in) return json();
    json j = json::parse(in, nullptr, false);
    return j.is_discarded() ? json() : j;
}

json default_project() {
    return {{"format", "psom-project/1"},
            {"status", "captured"},
            {"last_error", nullptr},
            {"crop", nullptr},
            {"scale", {{"factor", 1.0}, {"a", nullptr}, {"b", nullptr}, {"known_mm", nullptr}}},
            {"runs", json::array()}};
}

json load_project(const fs::path& project, Result& r) {
    json p = read_json(project / "project.json");
    if (!p.is_object()) {
        if (fs::exists(project / "project.json")) r.warn("project.json was unreadable and has been recreated");
        p = default_project();
    }
    if (!p.contains("runs") || !p["runs"].is_array()) p["runs"] = json::array();
    if (!p.contains("scale") || !p["scale"].is_object()) p["scale"] = default_project()["scale"];
    if (p.value("status", std::string()) == "capturing") {
        p["status"] = "captured";
        r.warn("recovered an interrupted capture");
    }
    return p;
}

json vec(const cv::Vec3d& v) { return {v[0], v[1], v[2]}; }

json crop_json(const CropBox& b) {
    return {{"source", b.source},
            {"center", vec(b.center)},
            {"axes", {{b.axes(0, 0), b.axes(0, 1), b.axes(0, 2)}, {b.axes(1, 0), b.axes(1, 1), b.axes(1, 2)},
                      {b.axes(2, 0), b.axes(2, 1), b.axes(2, 2)}}},
            {"half", vec(b.half)}};
}

bool parse_vec(const json& j, cv::Vec3d& v) {
    if (!j.is_array() || j.size() != 3) return false;
    for (int k = 0; k < 3; ++k) {
        if (!j[k].is_number()) return false;
        v[k] = j[k].get<double>();
        if (!std::isfinite(v[k])) return false;
    }
    return true;
}

// A user crop comes from the app: check it before trusting it.
bool parse_crop(const json& j, CropBox& b, std::string& why) {
    if (!j.is_object()) return why = "crop is not an object", false;
    cv::Vec3d c, h, a0, a1, a2;
    if (!j.contains("axes") || !j["axes"].is_array() || j["axes"].size() != 3) return why = "crop axes missing", false;
    if (!parse_vec(j.value("center", json()), c) || !parse_vec(j.value("half", json()), h) ||
        !parse_vec(j["axes"][0], a0) || !parse_vec(j["axes"][1], a1) || !parse_vec(j["axes"][2], a2))
        return why = "crop has missing or non-finite numbers", false;
    if (h[0] <= 0.002 || h[1] <= 0.002 || h[2] <= 0.002) return why = "crop box is too small", false;
    if (std::abs(cv::norm(a0) - 1) > 1e-3 || std::abs(cv::norm(a1) - 1) > 1e-3 || std::abs(cv::norm(a2) - 1) > 1e-3 ||
        std::abs(a0.dot(a1)) > 1e-3 || std::abs(a0.dot(a2)) > 1e-3 || std::abs(a1.dot(a2)) > 1e-3)
        return why = "crop axes are not orthonormal", false;
    b.center = c;
    b.half = h;
    b.axes = cv::Matx33d(a0[0], a0[1], a0[2], a1[0], a1[1], a1[2], a2[0], a2[1], a2[2]);
    b.source = j.value("source", std::string("user"));
    return true;
}

json stats(std::vector<float> v, double scale = 1) {
    if (v.empty()) return nullptr;
    auto q = [&](double f) {
        const size_t k = std::min(v.size() - 1, size_t(f * (v.size() - 1)));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k] * scale;
    };
    return {{"p10", q(0.1)}, {"median", q(0.5)}, {"p90", q(0.9)}};
}

json topo_json(const Topology& t) {
    return {{"closed_valid", t.closed_valid}, {"faces", t.faces}, {"vertices", t.vertices},
            {"boundary_edges", t.boundary_edges}, {"nonmanifold_edges", t.nonmanifold_edges},
            {"inconsistent_edges", t.inconsistent_edges}, {"nonmanifold_vertices", t.nonmanifold_vertices},
            {"degenerate_faces", t.degenerate_faces}, {"components", t.components},
            {"volume_cm3", t.signed_volume * 1e6}, {"reasons", t.reasons}};
}

std::string fmt(const char* f, double x) {
    char b[64];
    std::snprintf(b, sizeof b, f, x);
    return b;
}

json mat_json(const cv::Matx44d& T) {
    json a = json::array();
    for (double x : T.val) a.push_back(x);
    return a;
}

// Photo scans: poses from the images, cached in recon/sfm.json (keyed by the frames and parameters).
Result solve_poses(const fs::path& recon, Session& s, const Progress& progress, bool verbose, json& info) {
    SfmParams sp;
    sp.verbose = verbose;
    sp.refine_focal = !s.focal_calibrated;
    json want = {{"version", 1}, {"frames", json::array()},
                 {"params", {{"max_side", sp.max_side}, {"max_features", sp.max_features}, {"contrast", sp.contrast}, {"equalize", sp.equalize}, {"window", sp.window},
                             {"ratio", sp.ratio}, {"min_inliers", sp.min_inliers}, {"epipolar_px", sp.epipolar_px},
                             {"pnp_px", sp.pnp_px}, {"refine_focal", sp.refine_focal}, {"max_reproj_px", sp.max_reproj_px}, {"min_angle_deg", sp.min_angle_deg}}}};
    for (auto& f : s.frames) want["frames"].push_back(f.i);
    SfmResult r;
    Result res;
    res.stage = "poses";
    json have = read_json(recon / "sfm.json");
    bool cached = have.is_object() && have.value("key", json()) == want;
    if (cached) {
        try {
            r.K = s.frames[0].K;
            r.K.fx = r.K.fy = have["focal_px"].get<double>();
            r.f_guess = have["focal_guess_px"].get<double>();
            r.rms_px = have["rms_px"].get<double>();
            for (auto& f : have["frames"]) {
                r.frames.push_back(f["k"].get<int>());
                cv::Matx44d T;
                for (int k = 0; k < 16; ++k) T.val[k] = f["T_wc"][k].get<double>();
                r.T_wc.push_back(T);
            }
            for (auto& X : have["points"]) r.points.emplace_back(X[0].get<double>(), X[1].get<double>(), X[2].get<double>());
            for (auto& w : have["warnings"]) res.warn(w.get<std::string>());
            cached = !r.frames.empty() && r.frames.back() < int(s.frames.size());
        } catch (const std::exception&) {
            cached = false;
        }
    }
    if (!cached) {
        r = SfmResult{};
        Result sr = run_sfm(s, sp, r, progress);
        if (!sr.ok) return sr;
        res.absorb(sr);
        json fr = json::array(), pts = json::array();
        for (size_t k = 0; k < r.frames.size(); ++k) fr.push_back({{"k", r.frames[k]}, {"T_wc", mat_json(r.T_wc[k])}});
        for (auto& X : r.points) pts.push_back({X[0], X[1], X[2]});
        write_json(recon / "sfm.json", {{"key", want}, {"id", now_utc() + fmt("-%.4f", r.rms_px)}, {"focal_px", r.K.fx},
                                        {"focal_guess_px", r.f_guess}, {"rms_px", r.rms_px}, {"warnings", sr.warnings},
                                        {"frames", fr}, {"points", pts}});
        have = read_json(recon / "sfm.json");
    }
    const size_t total = s.frames.size();
    Result ar = apply_sfm(s, r);
    if (!ar.ok) return ar;
    res.absorb(ar);
    info = {{"source", "photos"}, {"placed", s.frames.size()}, {"photos", total}, {"rms_px", r.rms_px},
            {"focal_px", r.K.fx}, {"focal_guess_px", r.f_guess}, {"points", s.points.size()},
            {"id", have.is_object() ? have.value("id", std::string()) : std::string()}};
    res.ok = true;
    return res;
}

}  // namespace

Result reconstruct(const fs::path& project, const Options& o, const Progress& progress) {
    auto P = [&](const std::string& stage, double f) {
        if (progress) progress(stage, f);
    };
    Result res;
    res.stage = "done";
    json proj = load_project(project, res);
    proj["runs"].push_back({{"started_utc", now_utc()}, {"ok", false}, {"stage", "running"}, {"error", nullptr}});
    const fs::path recon = project / "recon";
    std::error_code ec;
    fs::create_directories(recon / "depth", ec);
    write_json(project / "project.json", proj);

    auto failed = [&](Result r) {
        r.warnings.insert(r.warnings.begin(), res.warnings.begin(), res.warnings.end());
        proj["last_error"] = {{"stage", r.stage}, {"error", r.error}};
        proj["runs"].back() = {{"started_utc", proj["runs"].back()["started_utc"]}, {"ok", false}, {"stage", r.stage}, {"error", r.error}};
        write_json(project / "project.json", proj);
        return r;
    };

    // 1. load
    P("load", 0);
    Session s;
    Result lr = load_session(project / "session", s);
    if (!lr.ok) return failed(lr);
    res.absorb(lr);
    json poses = {{"source", "arcore"}};
    if (s.photos) {
        Result pr = solve_poses(recon, s, progress, o.verbose, poses);
        if (!pr.ok) return failed(pr);
        res.absorb(pr);
    }
    if (o.verbose) {
        const cv::Vec3d c = scene_center(s);
        std::fprintf(stderr, "{\"event\":\"scene\",\"frames\":%zu,\"points\":%zu,\"centre\":[%.3f,%.3f,%.3f]}\n", s.frames.size(), s.points.size(), c[0], c[1], c[2]);
    }

    // 2. depth: raw where valid, MVS elsewhere; cached as a whole
    P("depth", 0);
    const size_t n = s.frames.size();
    std::vector<DepthMap> maps(n);
    MvsParams mvs = o.mvs;
    if (s.width >= 2 * mvs.width) mvs.window = mvs.window_shrunk;
    json want = {{"version", 1}, {"force_mvs", o.force_mvs}, {"frames", json::array()}, {"poses", poses.value("id", std::string())},
                 {"mvs", {{"width", mvs.width}, {"planes", mvs.planes}, {"neighbours", mvs.neighbours},
                          {"best_k", mvs.best_k}, {"window", mvs.window}, {"min_zncc", mvs.min_zncc},
                          {"min_texture", mvs.min_texture}, {"max_rel_sigma", mvs.max_rel_sigma},
                          {"consist_rel", mvs.consist_rel}, {"consist_px", mvs.consist_px},
                          {"min_consistent", mvs.min_consistent}}}};
    for (auto& f : s.frames) want["frames"].push_back(f.i);
    const json have = read_json(recon / "depth" / "manifest.json");
    bool cached = have.is_object() && have.value("complete", false) && have.value("key", json()) == want;
    for (size_t i = 0; cached && i < n; ++i)
        cached = load_depth(recon / "depth" / (frame_stem(s.frames[i].i) + ".bin"), maps[i]);
    int raw_frames = 0, mvs_frames = 0;
    if (!cached) {
        fs::remove_all(recon / "depth", ec);
        fs::create_directories(recon / "depth", ec);
        maps.assign(n, DepthMap{});
        std::vector<size_t> need;
        std::map<std::string, int> raw_why;
        for (size_t i = 0; i < n; ++i) {
            std::string why;
            if (!o.force_mvs && s.frames[i].has_depth && load_raw_depth(s, s.frames[i], maps[i], why)) continue;
            if (s.frames[i].has_depth && !o.force_mvs) ++raw_why[why];
            maps[i] = DepthMap{};
            need.push_back(i);
        }
        for (auto& [why, k] : raw_why) res.warn("raw depth unusable on " + std::to_string(k) + " frame(s): " + why);
        if (!need.empty()) {
            const cv::Vec3d center = scene_center(s);
            std::vector<MvsView> views(n);
            int bad_images = 0;
            for (size_t i = 0; i < n; ++i) {
                cv::Mat g = cv::imread(s.image_path(s.frames[i]).string(), cv::IMREAD_GRAYSCALE);
                if (g.empty() || g.cols != s.width || g.rows != s.height) {
                    ++bad_images;
                    continue;
                }
                views[i] = make_view(g, s.frames[i].K, s.frames[i].T_wc, mvs.width);
            }
            if (bad_images) res.warn(std::to_string(bad_images) + " image(s) unreadable or the wrong size; skipped");
            std::vector<std::vector<int>> nbrs(n);
            std::vector<cv::Matx44d> poses(n);
            for (size_t i = 0; i < n; ++i) poses[i] = s.frames[i].T_wc;
            const int k = std::clamp(mvs.neighbours, 1, 16);
            // Photos are independent: in parallel. Progress only from this thread (JNI needs that).
            const auto caller = std::this_thread::get_id();
            std::atomic<size_t> done{0};
            cv::parallel_for_(cv::Range(0, int(need.size())), [&](const cv::Range& range) {
              for (int q = range.start; q < range.end; ++q) {
                const size_t i = need[q];
                if (std::this_thread::get_id() == caller) P("depth", double(done) / need.size());
                ++done;
                if (views[i].gray.empty()) continue;
                std::vector<const MvsView*> nv;
                for (int j : select_neighbours(s, int(i), center, k))
                    if (!views[j].gray.empty()) nv.push_back(&views[j]), nbrs[i].push_back(j);
                const auto [near, far] = depth_range(s, s.frames[i], center);
                maps[i] = plane_sweep(views[i], nv, near, far, mvs);
                if (o.verbose)
                    std::fprintf(stderr, "{\"event\":\"mvs\",\"frame\":%d,\"neighbours\":%zu,\"near\":%.3f,\"far\":%.3f,\"valid\":%zu}\n",
                                 s.frames[i].i, nv.size(), near, far, maps[i].valid());
              }
            });
            for (size_t i = 0; i < n; ++i) poses[i] = pose_for(s.frames[i], maps[i]);
            consistency_filter(maps, poses, nbrs, mvs);
            if (o.verbose)
                for (size_t i : need)
                    std::fprintf(stderr, "{\"event\":\"consistent\",\"frame\":%d,\"valid\":%zu}\n", s.frames[i].i, maps[i].valid());
        }
        for (size_t i = 0; i < n; ++i) {
            if (maps[i].empty()) maps[i].resize(s.frames[i].K.scaled(1, 1));  // placeholder: no depth
            if (!save_depth(recon / "depth" / (frame_stem(s.frames[i].i) + ".bin"), maps[i])) {
                return failed(Result::fail("depth", "could not write the depth cache"));
            }
        }
        write_json(recon / "depth" / "manifest.json", {{"key", want}, {"complete", true}});
    }
    size_t mvs_px = 0, mvs_valid = 0;
    int with_depth = 0;
    for (auto& m : maps) {
        const size_t v = m.valid();
        if (m.source == DepthMap::Source::Raw) ++raw_frames;
        if (m.source == DepthMap::Source::Mvs) ++mvs_frames, mvs_px += m.d.size(), mvs_valid += v;
        with_depth += v > 0;
    }
    const double mvs_rejected = mvs_px ? 1.0 - double(mvs_valid) / mvs_px : 0.0;
    if (with_depth < 3)
        return failed(Result::fail("depth", with_depth == 0 ? std::string("No frame had depth that could be trusted.")
                                                            : "Only " + std::to_string(with_depth) + " frames had depth that could be trusted."));
    if (mvs_rejected > 0.6)
        res.warn("most image depth was rejected as unreliable (" + fmt("%.0f", mvs_rejected * 100) +
                 " %): low texture, blur or too little overlap");
    if (with_depth < int(n) / 2) res.warn("only " + std::to_string(with_depth) + " of " + std::to_string(n) + " keyframes have usable depth");

    // 3. isolate
    P("isolate", 0);
    CropBox box;
    Plane plane;
    Result ar = auto_crop(s, maps, box, plane);
    const bool user_crop = proj.contains("crop") && proj["crop"].is_object() && proj["crop"].value("source", "") == "user";
    if (user_crop) {
        std::string why;
        if (!parse_crop(proj["crop"], box, why)) return failed(Result::fail("isolate", "the edited crop box is invalid: " + why));
        if (!ar.ok) {
            plane = Plane{};
            res.warn("automatic isolation failed (" + ar.error + "); using your crop box without a support plane");
        }
    } else if (!ar.ok) {
        return failed(ar);
    } else {
        proj["crop"] = crop_json(box);
    }
    if (ar.ok) res.absorb(ar);
    const double diag = 2 * cv::norm(box.half);
    if (diag > 2.0) return failed(Result::fail("isolate", "the crop box is larger than 2 m"));

    // viewing directions covered, around the crop box
    int covered = 0;
    bool top = false;
    {
        bool bins[3][12] = {};
        const cv::Vec3d ax(box.axes(0, 0), box.axes(0, 1), box.axes(0, 2)), ay(box.axes(1, 0), box.axes(1, 1), box.axes(1, 2)),
            az(box.axes(2, 0), box.axes(2, 1), box.axes(2, 2));
        for (auto& f : s.frames) {
            const cv::Vec3d d = cv::normalize(f.center() - box.center);
            const double el = std::asin(std::clamp(d.dot(az), -1.0, 1.0)) * 180 / CV_PI;
            if (el < 0) continue;
            double a = std::atan2(d.dot(ay), d.dot(ax)) * 180 / CV_PI;
            if (a < 0) a += 360;
            bins[std::min(2, int(el / 30))][std::min(11, int(a / 30))] = true;
        }
        for (int e = 0; e < 3; ++e)
            for (int a = 0; a < 12; ++a) covered += bins[e][a], top |= e == 2 && bins[e][a];
        if (covered < 22) res.warn("only " + std::to_string(covered) + " of 36 viewing directions were covered; move further around the object");
    }
    // Too little of the object seen to say anything about its shape: fail plainly rather than
    // hand over a model that is mostly guesswork.
    if (covered < 6)
        return failed(Result::fail("isolate", "Only " + std::to_string(covered) +
                                                  " of 36 directions around the object were seen. Walk further around it."));

    // 4. fusion within the memory budget
    P("fusion", 0);
    const size_t budget = o.mem_mb * size_t(1) << 20;
    double voxel = std::clamp(diag / 400, 0.0005, 0.002);
    bool coarsened = false;
    Mesh mesh;
    size_t blocks = 0, bytes = 0;
    for (int attempt = 0;; ++attempt) {
        SparseTsdf tsdf(box, {voxel, std::max(5 * voxel, 0.005), budget, 0.5});
        bool ok = true;
        for (size_t i = 0; i < n && ok; ++i) {
            if (maps[i].valid() == 0) continue;
            ok = tsdf.integrate(maps[i], pose_for(s.frames[i], maps[i]));
            P("fusion", double(i + 1) / n);
        }
        if (ok) {
            blocks = tsdf.blocks();
            bytes = tsdf.bytes();
            P("mesh", 0);
            mesh = tsdf.extract();
            break;
        }
        if (attempt == 0) {
            voxel *= 2;
            coarsened = true;
            res.warn("voxels coarsened to " + fmt("%.1f", voxel * 1000) + " mm to fit the memory budget");
            continue;
        }
        return failed(Result::fail("fusion", "the object exceeds the memory budget (" + std::to_string(o.mem_mb) +
                                                 " MB) even at " + fmt("%.1f", voxel * 1000) +
                                                 " mm voxels. Crop tighter or allow more memory."));
    }
    if (mesh.f.empty()) return failed(Result::fail("mesh", "no surface found inside the crop box"));

    // 5. repair and validate
    P("repair", 0);
    RepairParams rp;
    rp.plane_tol = kCropFloor + 2 * voxel + 0.001;
    rp.plane_max = kCropFloor + 0.004;
    rp.max_hole_perimeter = std::max(0.03, 10 * voxel);
    rp.face_budget = o.face_budget;
    rp.progress = [&](double f) { P("repair", f); };
    const RepairReport rr = repair(mesh, plane, rp);
    for (auto& w : rr.warnings) res.warn(w);
    if (mesh.f.empty()) return failed(Result::fail("repair", "nothing was left after removing fragments"));
    const Topology topo = validate(mesh);
    if (!topo.closed_valid) {
        std::string why;
        for (auto& x : topo.reasons) why += (why.empty() ? "" : "; ") + x;
        res.warn("the mesh is not a closed valid shell (" + why + "); export will be an open shell");
    }
    if (!write_ply(recon / "mesh.ply", mesh)) return failed(Result::fail("mesh", "could not write mesh.ply"));
    {
        Mesh back;
        std::string err;
        if (!read_ply(recon / "mesh.ply", back, err) || back.f.size() != mesh.f.size() || back.v.size() != mesh.v.size())
            return failed(Result::fail("mesh", "mesh.ply did not read back intact: " + err));
    }

    // 6. report
    if (!s.tracking_loss.empty())
        res.warn("tracking was lost " + std::to_string(s.tracking_loss.size()) + " time(s) (" + s.tracking_loss[0].reason +
                 "): there are no frames from those moments");

    const double factor = proj["scale"].value("factor", 1.0);
    const bool scale_known = !s.scale_assumed || proj["scale"].value("known_mm", json()).is_number();
    if (!scale_known) res.warn("the size is a guess until you set the scale: photos alone don't measure it");
    json dims = nullptr;
    {
        const cv::Vec3d ax(box.axes(0, 0), box.axes(0, 1), box.axes(0, 2)), ay(box.axes(1, 0), box.axes(1, 1), box.axes(1, 2));
        const cv::Vec3d up = plane.found ? plane.n : cv::Vec3d(box.axes(2, 0), box.axes(2, 1), box.axes(2, 2));
        std::vector<cv::Vec2d> pts;
        double hmin = 1e9, hmax = -1e9;
        for (auto& v : mesh.v) {
            const cv::Vec3d P = cv::Vec3d(v) - box.center;
            pts.emplace_back(P.dot(ax), P.dot(ay));
            const double h = plane.found ? plane.height(cv::Vec3d(v)) : P.dot(up);
            hmin = std::min(hmin, h);
            hmax = std::max(hmax, h);
        }
        // Robust extents: the 0.5-99.5 % of vertices along the rectangle's axes, so a few noise bumps
        // don't set the size. The STEP keeps the bumps; only the reported dimensions ignore them.
        const Rect2 rect = min_area_rect(pts);
        const cv::Vec2d e(rect.axis), q(-rect.axis[1], rect.axis[0]);
        std::vector<double> pa, pb, ph;
        for (size_t i = 0; i < pts.size(); ++i) {
            pa.push_back(pts[i].dot(e));
            pb.push_back(pts[i].dot(q));
            const cv::Vec3d V(mesh.v[i]);
            ph.push_back(plane.found ? plane.height(V) : (V - box.center).dot(up));
        }
        auto pct = [](std::vector<double>& v, double f) {
            const size_t k = std::min(v.size() - 1, size_t(f * (v.size() - 1)));
            std::nth_element(v.begin(), v.begin() + k, v.end());
            return v[k];
        };
        const double a = pct(pa, 0.995) - pct(pa, 0.005), b = pct(pb, 0.995) - pct(pb, 0.005);
        hmax = pct(ph, 0.995);
        hmin = pct(ph, 0.005);
        const double k = 1000 * factor;
        dims = {{"length", a * k}, {"width", b * k}, {"height", (hmax - (plane.found ? 0.0 : hmin)) * k}, {"scale_factor", factor}, {"scale_known", scale_known},
                {"frame", plane.found ? "support plane" : "crop box"},
                {"method", "0.5-99.5 percentile extents of mesh vertices"}};
    }
    json tl = json::array();
    for (auto& t : s.tracking_loss) tl.push_back({{"from_ns", t.from_ns}, {"to_ns", t.to_ns}, {"reason", t.reason}});
    json report = {
        {"ok", true},
        {"warnings", res.warnings},
        {"poses", poses},
        {"session", {{"keyframes", n}, {"dropped", s.frames_dropped}, {"capture_ended", s.capture_ended},
                     {"rejected_by_app", s.rejected}, {"tracking_loss", tl}, {"depth_skew_dropped", s.depth_dropped_skew}}},
        {"depth", {{"raw_frames", raw_frames}, {"mvs_frames", mvs_frames}, {"frames_with_depth", with_depth},
                   {"mvs_rejected_fraction", mvs_rejected}, {"cached", cached}}},
        {"coverage", {{"bins_covered", covered}, {"bins_total", 36}, {"fraction", covered / 36.0}, {"top_seen", top}}},
        {"isolation", {{"support_plane", plane.found}, {"crop", crop_json(box)}}},
        {"fusion", {{"voxel_mm", voxel * 1000}, {"blocks", blocks}, {"memory_mb", bytes / 1048576.0},
                    {"budget_mb", o.mem_mb}, {"coarsened", coarsened}}},
        {"repair", {{"faces_before", rr.faces_before}, {"components_removed", rr.components_removed},
                    {"base_capped_on_support_plane", rr.capped}, {"base_cap_area_cm2", rr.cap_area * 1e4}, {"base_rim_max_snap_mm", rr.cap_max_snap * 1000},
                    {"base_rim_max_smoothing_mm", rr.cap_max_smooth * 1000},
                    {"small_holes_filled", rr.holes_filled}, {"small_holes_area_mm2", rr.holes_area * 1e6},
                    {"holes_left_open", rr.holes_open}, {"open_rim_mm", rr.open_perimeter * 1000}, {"decimated", rr.decimated}}},
        {"topology", topo_json(topo)},
        {"confidence", {{"observation_weight", stats(mesh.weight)},
                        {"view_diversity", stats(mesh.diversity)},
                        {"fused_sigma_mm", stats(mesh.sigma, 1000)},
                        {"note", "Three separate channels. None of them is a measure of accuracy; fused sigma assumes independent noise per view."}}},
        {"dimensions_mm", dims},
        {"export", nullptr},
    };
    if (!write_json(recon / "report.json", report)) return failed(Result::fail("report", "could not write report.json"));
    fs::remove(recon / "model.step", ec);  // stale: it belonged to the previous mesh

    proj["status"] = "reconstructed";
    proj["last_error"] = nullptr;
    proj["runs"].back() = {{"started_utc", proj["runs"].back()["started_utc"]}, {"ok", true}, {"stage", "done"}, {"error", nullptr}};
    if (!write_json(project / "project.json", proj)) return Result::fail("report", "could not write project.json");
    P("done", 1);
    res.ok = true;
    return res;
}

Result export_project(const fs::path& project, const Progress& progress) {
    Result res;
    res.stage = "done";
    json proj = load_project(project, res);
    auto failed = [&](Result r) {
        proj["last_error"] = {{"stage", r.stage}, {"error", r.error}};
        write_json(project / "project.json", proj);
        return r;
    };
    const std::string status = proj.value("status", std::string());
    if (status != "reconstructed" && status != "exported") return failed(Result::fail("export", "reconstruct the scan first"));
    if (progress) progress("export", 0);
    Mesh m;
    std::string err;
    if (!read_ply(project / "recon" / "mesh.ply", m, err)) return failed(Result::fail("export", "mesh.ply: " + err));
    const double factor = proj["scale"].value("factor", 1.0);
    if (!std::isfinite(factor) || factor < 0.1 || factor > 10) return failed(Result::fail("export", "scale factor out of range"));
    // A photo scan's size is a guess: a STEP in millimetres would claim a measurement nobody made.
    const json rep = read_json(project / "recon" / "report.json");
    if (rep.is_object() && rep.value("poses", json::object()).value("source", "") == "photos" &&
        !proj["scale"].value("known_mm", json()).is_number())
        return failed(Result::fail("export", "Set the scale first. Photos alone don't tell how big the object is."));
    const Topology topo = validate(m);
    StepInfo info;
    Result r = export_step(m, 1000 * factor, topo, project / "recon" / "model.step", info);
    if (!r.ok) return failed(r);
    res.absorb(r);

    json report = read_json(project / "recon" / "report.json");
    if (!report.is_object()) report = json::object();
    report["export"] = {{"solid", info.solid}, {"faces", info.faces}, {"volume_cm3", info.volume_mm3 / 1000},
                        {"bbox_mm", vec(info.bbox_mm)}, {"scale_factor", factor}, {"warnings", r.warnings},
                        {"geometry", "faceted B-rep: planar triangles, not a parametric CAD model"}};
    if (!write_json(project / "recon" / "report.json", report)) return failed(Result::fail("export", "could not update report.json"));
    proj["status"] = "exported";
    proj["last_error"] = nullptr;
    if (!write_json(project / "project.json", proj)) return Result::fail("export", "could not write project.json");
    res.ok = true;
    return res;
}

Result evaluate(const fs::path& project, const fs::path& truth_path, std::string& summary) {
    const json report = read_json(project / "recon" / "report.json");
    const json truth = read_json(truth_path);
    if (!report.is_object() || !report["dimensions_mm"].is_object()) return Result::fail("evaluate", "no report with dimensions; reconstruct first");
    if (!truth.is_object() || !truth.contains("dims_mm") || !truth["dims_mm"].is_array() || truth["dims_mm"].size() != 3)
        return Result::fail("evaluate", "truth.json needs dims_mm: [length, width, height]");
    const json& d = report["dimensions_mm"];
    const double m[3] = {d["length"].get<double>(), d["width"].get<double>(), d["height"].get<double>()};
    double t[3];
    for (int k = 0; k < 3; ++k) t[k] = truth["dims_mm"][k].get<double>();
    if (t[1] > t[0]) std::swap(t[0], t[1]);
    json out = {{"truth_mm", {t[0], t[1], t[2]}}, {"measured_mm", {m[0], m[1], m[2]}},
                {"error_mm", {m[0] - t[0], m[1] - t[1], m[2] - t[2]}}, {"name", truth.value("name", std::string())},
                {"synthetic", truth.value("synthetic", false)}};
    double worst = 0;
    for (int k = 0; k < 3; ++k) worst = std::max(worst, std::abs(m[k] - t[k]));
    out["max_abs_error_mm"] = worst;
    if (!write_json(project / "recon" / "evaluation.json", out)) return Result::fail("evaluate", "could not write evaluation.json");
    summary = out.dump();
    Result r;
    r.stage = "done";
    return r;
}

}  // namespace psom
