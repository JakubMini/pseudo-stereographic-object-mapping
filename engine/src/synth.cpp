#include "psom/synth.h"

#include <cmath>
#include <fstream>
#include <random>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "psom/session.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace psom {

namespace {

struct Hit {
    double t = 1e30;
    cv::Vec3d P, n;
    bool object = false;
};

double hash3(int x, int y, int z, uint32_t seed) {
    uint32_t h = uint32_t(x) * 374761393u + uint32_t(y) * 668265263u + uint32_t(z) * 2147483647u + seed * 144665u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return double((h ^ (h >> 16)) & 0xFFFFFF) / double(0xFFFFFF);
}

// trilinear value noise; a function of the surface point only, so it looks the same from every view
double noise(const cv::Vec3d& P, double cell, uint32_t seed) {
    const cv::Vec3d q = P / cell;
    const int x = int(std::floor(q[0])), y = int(std::floor(q[1])), z = int(std::floor(q[2]));
    const double fx = q[0] - x, fy = q[1] - y, fz = q[2] - z;
    double r = 0;
    for (int c = 0; c < 8; ++c) {
        const int dx = c & 1, dy = (c >> 1) & 1, dz = (c >> 2) & 1;
        r += hash3(x + dx, y + dy, z + dz, seed) * (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy) * (dz ? fz : 1 - fz);
    }
    return r;
}

struct Scene {
    std::string shape;
    cv::Vec3d dims;  // metres
    bool low_texture;
    uint32_t seed;

    void table(const cv::Vec3d& o, const cv::Vec3d& d, Hit& h) const {
        if (d[1] >= 0) return;
        const double t = -o[1] / d[1];
        const cv::Vec3d P = o + d * t;
        if (t > 0 && t < h.t && std::abs(P[0]) < 0.5 && std::abs(P[2]) < 0.5) h = {t, P, {0, 1, 0}, false};
    }
    void box(const cv::Vec3d& o, const cv::Vec3d& d, Hit& h) const {
        const cv::Vec3d lo(-dims[0] / 2, 0, -dims[1] / 2), hi(dims[0] / 2, dims[2], dims[1] / 2);
        double t0 = -1e30, t1 = 1e30;
        int axis = -1;
        double sign = 0;
        for (int k = 0; k < 3; ++k) {
            if (std::abs(d[k]) < 1e-12) {
                if (o[k] < lo[k] || o[k] > hi[k]) return;
                continue;
            }
            double a = (lo[k] - o[k]) / d[k], b = (hi[k] - o[k]) / d[k];
            double s = -1;
            if (a > b) std::swap(a, b), s = 1;
            if (a > t0) t0 = a, axis = k, sign = s;
            t1 = std::min(t1, b);
        }
        if (t0 > t1 || t0 <= 0 || t0 >= h.t || axis < 0) return;
        cv::Vec3d n(0, 0, 0);
        n[axis] = sign;
        h = {t0, o + d * t0, n, true};
    }
    void cylinder(const cv::Vec3d& o, const cv::Vec3d& d, Hit& h) const {
        const double r = dims[0] / 2, H = dims[2];
        const double a = d[0] * d[0] + d[2] * d[2], b = 2 * (o[0] * d[0] + o[2] * d[2]), c = o[0] * o[0] + o[2] * o[2] - r * r;
        const double disc = b * b - 4 * a * c;
        if (a > 1e-12 && disc >= 0) {
            const double t = (-b - std::sqrt(disc)) / (2 * a);
            const cv::Vec3d P = o + d * t;
            if (t > 0 && t < h.t && P[1] >= 0 && P[1] <= H) h = {t, P, cv::normalize(cv::Vec3d(P[0], 0, P[2])), true};
        }
        if (std::abs(d[1]) > 1e-12) {
            const double t = (H - o[1]) / d[1];
            const cv::Vec3d P = o + d * t;
            if (t > 0 && t < h.t && P[0] * P[0] + P[2] * P[2] <= r * r) h = {t, P, {0, 1, 0}, true};
        }
    }
    Hit cast(const cv::Vec3d& o, const cv::Vec3d& d) const {
        Hit h;
        table(o, d, h);
        if (shape == "cylinder") cylinder(o, d, h);
        else box(o, d, h);
        return h;
    }
    double shade(const Hit& h) const {
        if (h.t >= 1e29) return 0.5;  // far background
        double albedo;
        if (h.object && low_texture) albedo = 0.75;
        else albedo = 0.2 + 0.45 * noise(h.P, 0.003, seed) + 0.3 * noise(h.P, 0.0012, seed + 1);
        const cv::Vec3d light = cv::normalize(cv::Vec3d(0.3, 1.0, 0.5));
        return albedo * (0.4 + 0.6 * std::max(0.0, h.n.dot(light)));
    }
};

cv::Matx44d look_at(const cv::Vec3d& C, const cv::Vec3d& target) {
    const cv::Vec3d f = cv::normalize(target - C);
    const cv::Vec3d r = cv::normalize(f.cross(cv::Vec3d(0, 1, 0)));
    const cv::Vec3d dn = f.cross(r);
    return {r[0], dn[0], f[0], C[0], r[1], dn[1], f[1], C[1], r[2], dn[2], f[2], C[2], 0, 0, 0, 1};
}

}  // namespace

Result synth_project(const fs::path& project, const SynthSpec& sp) {
    const fs::path dir = project / "session";
    std::error_code ec;
    fs::remove_all(project, ec);
    for (auto sub : {"frames", "depth", "conf"}) fs::create_directories(dir / sub, ec);
    if (ec) return Result::fail("synth", "cannot create " + dir.string());

    const Scene scene{sp.shape, sp.dims_mm * 1e-3, sp.low_texture, sp.seed};
    const int W = sp.width, H = sp.height, DW = 160;
    Intrinsics K{500.0 * W / 640, 500.0 * W / 640, (W - 1) / 2.0, (H - 1) / 2.0, W, H};
    const int dh = int(std::lround(double(DW) * H / W));
    const Intrinsics KD = K.scaled(DW, dh);
    std::mt19937 rng(sp.seed);
    std::normal_distribution<double> gauss(0, 1);

    json meta = {
        {"format", "psom-session/1"},
        {"created_utc", "2026-01-01T00:00:00Z"},
        {"device", {{"manufacturer", "psom"}, {"model", "synthetic"}, {"arcore", "none"}}},
        {"conventions",
         {{"pose", "arcore_gl_world_from_camera"}, {"world_up", "+y"}, {"depth_units", "mm"}, {"image_orientation", "sensor"}}},
        {"camera", {{"sensor_orientation_deg", 0}, {"display_rotation_deg", 0}}},
        {"image", {{"width", W}, {"height", H}}},
        {"depth", {{"supported", sp.depth}, {"width", DW}, {"height", dh}, {"max_skew_ns", 16666666}}},
        {"clocks", {{"sensor_minus_arcore_ns", 0}}},
        {"target", nullptr},
    };
    std::ofstream(dir / "session.json") << meta.dump(2);

    std::ofstream frames(dir / "frames.jsonl"), events(dir / "events.jsonl"), points(dir / "points.csv");
    std::ofstream(dir / "imu.csv") << "t_ns,sensor,x,y,z\n";
    points << "t_ns,id,x,y,z,confidence\n";
    events << json{{"t_ns", 1'000'000'000}, {"event", "capture_start"}}.dump() << "\n";

    const cv::Vec3d target(0, sp.dims_mm[2] * 1e-3 / 2, 0);
    const int gap0 = sp.frames / 2, gap1 = gap0 + std::max(2, sp.frames / 8);
    int pid = 0;
    std::string last_line;
    for (int i = 0; i < sp.frames; ++i) {
        const int64_t t = 1'000'000'000 + int64_t(i) * 333'333'333;
        if (sp.tracking_loss && i == gap0)
            events << json{{"t_ns", t}, {"event", "tracking_lost"}, {"reason", "EXCESSIVE_MOTION"}}.dump() << "\n";
        if (sp.tracking_loss && i == gap1) events << json{{"t_ns", t}, {"event", "tracking_resumed"}}.dump() << "\n";
        if (sp.tracking_loss && i >= gap0 && i < gap1) continue;

        const double elev = (i % 2 ? 50.0 : 25.0) * CV_PI / 180, az = 2 * CV_PI * i / sp.frames;
        const cv::Vec3d C = target + sp.radius * cv::Vec3d(std::cos(elev) * std::cos(az), std::sin(elev), std::cos(elev) * std::sin(az));
        const cv::Matx44d T = look_at(C, target);
        const cv::Matx33d R = T.get_minor<3, 3>(0, 0);

        cv::Mat img(H, W, CV_8UC3);
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                double acc = 0;
                for (int s = 0; s < 4; ++s) {  // 2x2 supersampling
                    const cv::Vec3d d = cv::normalize(R * K.ray(x - 0.25 + 0.5 * (s & 1), y - 0.25 + 0.5 * (s >> 1)));
                    acc += scene.shade(scene.cast(C, d));
                }
                const uint8_t g = cv::saturate_cast<uint8_t>(acc / 4 * 255);
                img.at<cv::Vec3b>(y, x) = {g, g, g};
            }
        const std::string stem = frame_stem(i);
        cv::imwrite((dir / "frames" / (stem + ".jpg")).string(), img, {cv::IMWRITE_JPEG_QUALITY, 95});

        if (sp.depth) {
            cv::Mat dm(dh, DW, CV_16U, cv::Scalar(0)), cm(dh, DW, CV_8U, cv::Scalar(0));
            for (int y = 0; y < dh; ++y)
                for (int x = 0; x < DW; ++x) {
                    const cv::Vec3d ray = KD.ray(x, y);
                    const Hit h = scene.cast(C, cv::normalize(R * ray));
                    if (h.t >= 1e29) continue;
                    const double z = (R.t() * (h.P - C))[2];
                    const double zn = z + gauss(rng) * (0.001 + 0.002 * z * z);
                    dm.at<uint16_t>(y, x) = cv::saturate_cast<uint16_t>(zn * 1000);
                    cm.at<uint8_t>(y, x) = 255;
                }
            cv::imwrite((dir / "depth" / (stem + ".png")).string(), dm);
            cv::imwrite((dir / "conf" / (stem + ".png")).string(), cm);
        }

        // feature points: surface samples near the object, as ARCore would track them
        for (int k = 0; k < 60; ++k) {
            const double u = std::uniform_real_distribution<double>(0, W)(rng), v = std::uniform_real_distribution<double>(0, H)(rng);
            const Hit h = scene.cast(C, cv::normalize(R * K.ray(u, v)));
            if (h.t >= 1e29) continue;
            const cv::Vec3d X = h.P + cv::Vec3d(gauss(rng), gauss(rng), gauss(rng)) * 0.001;
            char b[160];
            std::snprintf(b, sizeof b, "%lld,%d,%.6f,%.6f,%.6f,0.9\n", (long long)t, pid++, X[0], X[1], X[2]);
            points << b;
        }

        cv::Mat gray, lap;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::Laplacian(gray, lap, CV_64F);
        cv::Scalar mu, sd;
        cv::meanStdDev(lap, mu, sd);

        // written pose: optional noise, then ARCore's GL convention, row-major
        cv::Matx44d Tw = T;
        if (sp.pose_noise_mm > 0) {
            const double s = sp.pose_noise_mm * 1e-3;
            cv::Vec3d rv(gauss(rng), gauss(rng), gauss(rng));
            rv *= s / sp.radius;
            const double th = cv::norm(rv);
            const cv::Vec3d ax = th > 0 ? rv / th : cv::Vec3d(1, 0, 0);
            const cv::Matx33d Kx(0, -ax[2], ax[1], ax[2], 0, -ax[0], -ax[1], ax[0], 0);
            const cv::Matx33d dR = cv::Matx33d::eye() + std::sin(th) * Kx + (1 - std::cos(th)) * (Kx * Kx);  // Rodrigues
            const cv::Matx33d R2 = dR * R;
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b) Tw(a, b) = R2(a, b);
            for (int a = 0; a < 3; ++a) Tw(a, 3) += gauss(rng) * s;
        }
        const cv::Matx44d Tgl = gl_to_cv(Tw);
        json pose = json::array();
        for (double x : Tgl.val) pose.push_back(x);
        json line = {{"i", i},
                     {"t_ns", t},
                     {"pose", pose},
                     {"K", {K.fx, K.fy, K.cx, K.cy}},
                     {"tracking", "tracking"},
                     {"sharpness", sd[0] * sd[0]},
                     {"depth", sp.depth},
                     {"depth_t_ns", t}};
        last_line = line.dump();
        if (sp.interrupted && i == sp.frames - 1)
            frames << last_line.substr(0, last_line.size() / 2);  // the app died mid-write
        else
            frames << last_line << "\n";
    }
    if (!sp.interrupted)
        events << json{{"t_ns", 1'000'000'000 + int64_t(sp.frames) * 333'333'333},
                       {"event", "capture_end"},
                       {"rejected", {{"blur", 0}, {"motion", 0}, {"tracking", 0}, {"rate", 0}}}}
                      .dump()
               << "\n";

    json proj = {{"format", "psom-project/1"},
                 {"status", sp.interrupted ? "capturing" : "captured"},
                 {"last_error", nullptr},
                 {"crop", nullptr},
                 {"scale", {{"factor", 1.0}, {"a", nullptr}, {"b", nullptr}, {"known_mm", nullptr}}},
                 {"runs", json::array()}};
    std::ofstream(project / "project.json") << proj.dump(2);
    Result r;
    r.stage = "synth";
    return r;
}

}  // namespace psom
