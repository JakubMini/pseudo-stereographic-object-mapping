#include "psom/repair.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>

#include <queue>
#include <unordered_set>

namespace psom {

namespace {

uint64_t ekey(uint32_t a, uint32_t b) { return (uint64_t(a) << 32) | b; }

struct UnionFind {
    std::vector<int> p;
    explicit UnionFind(size_t n) : p(n) { std::iota(p.begin(), p.end(), 0); }
    int find(int x) {
        while (p[x] != x) x = p[x] = p[p[x]];
        return x;
    }
    void unite(int a, int b) { p[find(a)] = find(b); }
};

double tri_area(const Mesh& m, const cv::Vec3i& t) {
    const cv::Vec3d a = m.v[t[0]], b = m.v[t[1]], c = m.v[t[2]];
    return 0.5 * cv::norm((b - a).cross(c - a));
}

std::string mm(double metres) {
    char b[32];
    std::snprintf(b, sizeof b, "%.1f mm", metres * 1000);
    return b;
}

// Keeps the largest component and any holding at least `fraction` of the faces; with a support
// plane, a component under `floating_fraction` also has to come within `reach` of the table, since
// an object rests on it and a piece hanging in the air is noise. Returns how many went.
int drop_small_components(Mesh& m, double fraction, const Plane& plane = {}, double floating_fraction = 0, double reach = 0) {
    UnionFind uf(m.v.size());
    for (auto& f : m.f) uf.unite(f[0], f[1]), uf.unite(f[1], f[2]);
    std::unordered_map<int, size_t> count;
    for (auto& f : m.f) ++count[uf.find(f[0])];
    size_t biggest = 0;
    for (auto& [_, c] : count) biggest = std::max(biggest, c);
    const size_t min_faces = std::max<size_t>(1, size_t(fraction * m.f.size()));
    std::unordered_map<int, double> lowest;
    if (plane.found)
        for (size_t v = 0; v < m.v.size(); ++v) {
            const int r = uf.find(int(v));
            const double h = plane.height(cv::Vec3d(m.v[v]));
            auto it = lowest.find(r);
            if (it == lowest.end()) lowest.emplace(r, h);
            else it->second = std::min(it->second, h);
        }
    auto kept = [&](int r) {
        const size_t c = count[r];
        if (c == biggest) return true;
        if (c < min_faces) return false;
        return !plane.found || c >= floating_fraction * biggest || lowest[r] <= reach;
    };
    std::vector<cv::Vec3i> keep;
    for (auto& f : m.f)
        if (kept(uf.find(f[0]))) keep.push_back(f);
    int removed = 0;
    for (auto& [r, c] : count) removed += !kept(r);
    m.f.swap(keep);
    m.compact();
    return removed;
}

// Quadric edge collapse (Garland-Heckbert). A collapse happens only if the edge's endpoints share
// exactly two neighbours (the link condition, which keeps a manifold manifold), no face flips or
// degenerates, and the quadric error stays under max_err. Boundary vertices never move.
size_t decimate(Mesh& m, size_t target, double max_err, const std::function<void(double)>& progress) {
    const size_t nv = m.v.size();
    struct Quad {
        double q[10] = {};
        double planes = 0;
        void add(const cv::Vec4d& p) {
            planes += 1;
            const double a = p[0], b = p[1], c = p[2], d = p[3];
            const double v[10] = {a * a, a * b, a * c, a * d, b * b, b * c, b * d, c * c, c * d, d * d};
            for (int i = 0; i < 10; ++i) q[i] += v[i];
        }
        void add(const Quad& o) {
            planes += o.planes;
            for (int i = 0; i < 10; ++i) q[i] += o.q[i];
        }
        double eval(const cv::Vec3d& x) const {
            return q[0] * x[0] * x[0] + 2 * q[1] * x[0] * x[1] + 2 * q[2] * x[0] * x[2] + 2 * q[3] * x[0] + q[4] * x[1] * x[1] +
                   2 * q[5] * x[1] * x[2] + 2 * q[6] * x[1] + q[7] * x[2] * x[2] + 2 * q[8] * x[2] + q[9];
        }
    };
    std::vector<cv::Vec3d> P(nv);
    for (size_t i = 0; i < nv; ++i) P[i] = m.v[i];
    std::vector<Quad> Q(nv);
    std::vector<std::vector<int>> vf(nv);
    std::vector<uint8_t> dead_f(m.f.size(), 0), dead_v(nv, 0), locked(nv, 0);
    std::vector<uint32_t> ver(nv, 0);
    std::unordered_map<uint64_t, int> und;
    for (int i = 0; i < int(m.f.size()); ++i) {
        const auto& f = m.f[i];
        cv::Vec3d n = (P[f[1]] - P[f[0]]).cross(P[f[2]] - P[f[0]]);
        const double len = cv::norm(n);
        if (len > 0) {
            n /= len;
            const cv::Vec4d pl(n[0], n[1], n[2], -n.dot(P[f[0]]));
            for (int k = 0; k < 3; ++k) Q[f[k]].add(pl);
        }
        for (int k = 0; k < 3; ++k) {
            vf[f[k]].push_back(i);
            const uint32_t a = f[k], b = f[(k + 1) % 3];
            ++und[ekey(std::min(a, b), std::max(a, b))];
        }
    }
    for (auto& [k, c] : und)
        if (c != 2) locked[k >> 32] = locked[uint32_t(k)] = 1;

    auto neighbours = [&](int v) {
        std::vector<int> out;
        for (int fi : vf[v])
            if (!dead_f[fi])
                for (int k = 0; k < 3; ++k)
                    if (m.f[fi][k] != v) out.push_back(m.f[fi][k]);
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return out;
    };
    struct Cand {
        double cost;
        int u, v;
        uint32_t vu, vv;
        cv::Vec3d p;
        bool operator<(const Cand& o) const { return cost > o.cost; }
    };
    std::priority_queue<Cand> heap;
    auto push = [&](int u, int v) {
        if (locked[u] || locked[v]) return;
        Quad q = Q[u];
        q.add(Q[v]);
        cv::Matx33d A(q.q[0], q.q[1], q.q[2], q.q[1], q.q[4], q.q[5], q.q[2], q.q[5], q.q[7]);
        cv::Vec3d best = (P[u] + P[v]) * 0.5;
        if (std::abs(cv::determinant(A)) > 1e-18) {
            const cv::Vec3d x = A.solve(cv::Vec3d(-q.q[3], -q.q[6], -q.q[8]), cv::DECOMP_LU);
            // stay near the edge: an optimum far away means the quadric is ill-conditioned
            if (cv::norm(x - best) < 2 * cv::norm(P[u] - P[v])) best = x;
        }
        double c = q.eval(best);
        for (const cv::Vec3d& alt : {P[u], P[v]})
            if (q.eval(alt) < c) c = q.eval(alt), best = alt;
        // RMS distance to the merged planes, so the bound reads as a distance
        heap.push({std::max(c, 0.0) / std::max(q.planes, 1.0), u, v, ver[u], ver[v], best});
    };
    for (auto& [k, c] : und)
        if (c == 2) push(int(k >> 32), int(uint32_t(k)));

    size_t alive = m.f.size();
    const double max_cost = max_err * max_err;
    const size_t start_faces = alive;
    size_t steps = 0;
    while (alive > target && !heap.empty()) {
        if (progress && ++steps % 4096 == 0) progress(double(start_faces - alive) / double(start_faces - target));
        const Cand c = heap.top();
        heap.pop();
        if (c.cost > max_cost) break;
        if (dead_v[c.u] || dead_v[c.v] || ver[c.u] != c.vu || ver[c.v] != c.vv) continue;
        const auto nu = neighbours(c.u), nw = neighbours(c.v);
        std::vector<int> common;
        std::set_intersection(nu.begin(), nu.end(), nw.begin(), nw.end(), std::back_inserter(common));
        if (common.size() != 2) continue;  // link condition
        bool ok = true;
        for (int side = 0; side < 2 && ok; ++side)
            for (int fi : vf[side ? c.v : c.u]) {
                if (dead_f[fi]) continue;
                const auto& f = m.f[fi];
                const bool has_u = f[0] == c.u || f[1] == c.u || f[2] == c.u;
                const bool has_v = f[0] == c.v || f[1] == c.v || f[2] == c.v;
                if (has_u && has_v) continue;  // removed by the collapse
                cv::Vec3d a = P[f[0]], b = P[f[1]], d = P[f[2]];
                const cv::Vec3d n0 = (b - a).cross(d - a);
                for (int k = 0; k < 3; ++k)
                    if (f[k] == c.u || f[k] == c.v) (k == 0 ? a : k == 1 ? b : d) = c.p;
                const cv::Vec3d n1 = (b - a).cross(d - a);
                const double l0 = cv::norm(n0), l1 = cv::norm(n1);
                if (l1 < 1e-14 || n0.dot(n1) < 0.3 * l0 * l1) {
                    ok = false;
                    break;
                }
            }
        if (!ok) continue;
        // collapse u into v
        for (int fi : vf[c.u]) {
            if (dead_f[fi]) continue;
            auto& f = m.f[fi];
            if (f[0] == c.v || f[1] == c.v || f[2] == c.v) {
                dead_f[fi] = 1;
                --alive;
                continue;
            }
            for (int k = 0; k < 3; ++k)
                if (f[k] == c.u) f[k] = c.v;
            vf[c.v].push_back(fi);
        }
        dead_v[c.u] = 1;
        P[c.v] = c.p;
        Q[c.v].add(Q[c.u]);
        if (m.has_channels()) {
            m.weight[c.v] = 0.5f * (m.weight[c.v] + m.weight[c.u]);
            m.diversity[c.v] = 0.5f * (m.diversity[c.v] + m.diversity[c.u]);
            m.sigma[c.v] = 0.5f * (m.sigma[c.v] + m.sigma[c.u]);
        }
        ++ver[c.v];
        for (int n : neighbours(c.v)) ++ver[n];
        for (int n : neighbours(c.v)) {
            push(c.v, n);
            for (int nn : neighbours(n))
                if (nn != c.v) push(n, nn);
        }
    }
    std::vector<cv::Vec3i> keep;
    for (size_t i = 0; i < m.f.size(); ++i)
        if (!dead_f[i]) keep.push_back(m.f[i]);
    m.f.swap(keep);
    for (size_t i = 0; i < nv; ++i) m.v[i] = cv::Vec3f(float(P[i][0]), float(P[i][1]), float(P[i][2]));
    m.compact();
    return m.f.size();
}

}  // namespace

Topology validate(const Mesh& m, double min_area) {
    Topology t;
    t.vertices = m.v.size();
    t.faces = m.f.size();
    std::unordered_map<uint64_t, int> und, dir;
    und.reserve(m.f.size() * 2);
    dir.reserve(m.f.size() * 4);
    for (auto& f : m.f) {
        if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2] || tri_area(m, f) < min_area) ++t.degenerate_faces;
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = f[k], b = f[(k + 1) % 3];
            ++und[ekey(std::min(a, b), std::max(a, b))];
            ++dir[ekey(a, b)];
        }
    }
    for (auto& [k, c] : und) {
        const uint32_t a = uint32_t(k >> 32), b = uint32_t(k);
        if (c == 1) ++t.boundary_edges;
        else if (c > 2) ++t.nonmanifold_edges;
        else {
            auto ab = dir.find(ekey(a, b)), ba = dir.find(ekey(b, a));
            if (ab == dir.end() || ba == dir.end() || ab->second != 1 || ba->second != 1) ++t.inconsistent_edges;
        }
    }
    // vertex manifoldness: the faces around a vertex must form one fan
    std::vector<std::vector<int>> vf(m.v.size());
    for (int i = 0; i < int(m.f.size()); ++i)
        for (int k = 0; k < 3; ++k) vf[m.f[i][k]].push_back(i);
    for (size_t v = 0; v < vf.size(); ++v) {
        auto& faces = vf[v];
        if (faces.size() < 2) continue;
        UnionFind uf(faces.size());
        std::unordered_map<int, int> by_other;  // other vertex of an edge through v -> local face
        for (int q = 0; q < int(faces.size()); ++q)
            for (int k = 0; k < 3; ++k) {
                const int o = m.f[faces[q]][k];
                if (o == int(v)) continue;
                auto [it, fresh] = by_other.emplace(o, q);
                if (!fresh) uf.unite(q, it->second);
            }
        int roots = 0;
        for (int q = 0; q < int(faces.size()); ++q) roots += uf.find(q) == q;
        if (roots > 1) ++t.nonmanifold_vertices;
    }
    UnionFind comp(m.v.size());
    for (auto& f : m.f) comp.unite(f[0], f[1]), comp.unite(f[1], f[2]);
    std::vector<uint8_t> used(m.v.size(), 0), root(m.v.size(), 0);
    for (auto& f : m.f)
        for (int k = 0; k < 3; ++k) used[f[k]] = 1;
    for (size_t v = 0; v < m.v.size(); ++v)
        if (used[v] && !root[comp.find(int(v))]) root[comp.find(int(v))] = 1, ++t.components;
    t.signed_volume = signed_volume(m);

    if (t.faces < 4) t.reasons.push_back("too few faces");
    if (t.boundary_edges) t.reasons.push_back(std::to_string(t.boundary_edges) + " boundary edges (open surface)");
    if (t.nonmanifold_edges) t.reasons.push_back(std::to_string(t.nonmanifold_edges) + " edges shared by more than two faces");
    if (t.inconsistent_edges) t.reasons.push_back(std::to_string(t.inconsistent_edges) + " edges with inconsistent face orientation");
    if (t.nonmanifold_vertices) t.reasons.push_back(std::to_string(t.nonmanifold_vertices) + " non-manifold vertices");
    if (t.degenerate_faces) t.reasons.push_back(std::to_string(t.degenerate_faces) + " degenerate faces");
    if (t.components != 1) t.reasons.push_back(std::to_string(t.components) + " separate shells");
    if (t.reasons.empty() && t.signed_volume <= 0) t.reasons.push_back("faces point inwards (negative volume)");
    t.closed_valid = t.reasons.empty();
    return t;
}

std::vector<std::vector<int>> boundary_loops(const Mesh& m) {
    std::unordered_map<uint64_t, int> und;
    for (auto& f : m.f)
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = f[k], b = f[(k + 1) % 3];
            ++und[ekey(std::min(a, b), std::max(a, b))];
        }
    std::unordered_multimap<int, int> next;  // boundary edges in face winding order
    for (auto& f : m.f)
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = f[k], b = f[(k + 1) % 3];
            if (und[ekey(std::min(a, b), std::max(a, b))] == 1) next.emplace(int(a), int(b));
        }
    std::vector<std::vector<int>> loops;
    while (!next.empty()) {
        auto it = next.begin();
        const int start = it->first;
        std::vector<int> loop{start};
        int cur = it->second;
        next.erase(it);
        while (cur != start) {
            loop.push_back(cur);
            auto n = next.find(cur);
            if (n == next.end()) break;  // broken chain (non-manifold boundary)
            cur = n->second;
            next.erase(n);
        }
        if (cur == start && loop.size() >= 3) loops.push_back(loop);
    }
    return loops;
}

bool fill_loop(Mesh& m, const std::vector<int>& loop, const cv::Vec3d& n_in, double* area_out) {
    const size_t N = loop.size();
    if (N < 3) return false;
    const cv::Vec3d n = cv::normalize(n_in);
    cv::Vec3d u = std::abs(n[0]) < 0.9 ? cv::Vec3d(1, 0, 0) : cv::Vec3d(0, 1, 0);
    u = cv::normalize(u - n * n.dot(u));
    const cv::Vec3d w = n.cross(u);
    std::vector<cv::Vec2d> p(N);
    for (size_t i = 0; i < N; ++i) {
        const cv::Vec3d P = m.v[loop[i]];
        p[i] = {P.dot(u), P.dot(w)};
    }
    double a2 = 0;
    for (size_t i = 0; i < N; ++i) a2 += p[i][0] * p[(i + 1) % N][1] - p[(i + 1) % N][0] * p[i][1];
    if (a2 <= 0) return false;  // the loop doesn't wind around n: it doesn't bound a region facing that way

    auto cross = [](const cv::Vec2d& a, const cv::Vec2d& b, const cv::Vec2d& c) {
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    };
    const double eps = 1e-14;
    // a diagonal that already is an edge of the mesh would be shared by three faces; only edges
    // between this loop's vertices can be diagonals, so only those are collected
    std::unordered_map<uint64_t, int> edges;
    auto ek = [](int a, int b) { return ekey(uint32_t(std::min(a, b)), uint32_t(std::max(a, b))); };
    std::vector<char> on_loop(m.v.size(), 0);
    for (int v : loop) on_loop[v] = 1;
    for (auto& f : m.f)
        for (int k = 0; k < 3; ++k)
            if (on_loop[f[k]] && on_loop[f[(k + 1) % 3]]) edges[ek(f[k], f[(k + 1) % 3])] = 1;
    std::vector<int> idx(N);
    std::iota(idx.begin(), idx.end(), 0);
    std::vector<cv::Vec3i> tris;
    double area = 0;
    size_t start = 0;
    while (idx.size() > 3) {
        const size_t n_idx = idx.size();
        bool clipped = false;
        for (int pass = 0; pass < 2 && !clipped; ++pass) {  // pass 0 skips degenerate ears
            for (size_t s = 0; s < n_idx; ++s) {
                const size_t i = (start + s) % n_idx;
                const int a = idx[(i + n_idx - 1) % n_idx], b = idx[i], c = idx[(i + 1) % n_idx];
                const double cr = cross(p[a], p[b], p[c]);
                if (cr < 0 || (pass == 0 && cr <= eps)) continue;  // reflex, or degenerate on the first pass
                bool inside = false;
                for (int q : idx) {
                    if (q == a || q == b || q == c) continue;
                    if (cross(p[a], p[b], p[q]) > 0 && cross(p[b], p[c], p[q]) > 0 && cross(p[c], p[a], p[q]) > 0) {
                        inside = true;
                        break;
                    }
                }
                if (inside || edges.count(ek(loop[a], loop[c]))) continue;
                edges[ek(loop[a], loop[c])] = 1;
                tris.emplace_back(loop[a], loop[b], loop[c]);
                area += 0.5 * cr;
                idx.erase(idx.begin() + i);
                start = i % idx.size();
                clipped = true;
                break;
            }
        }
        if (!clipped) return false;  // not a simple polygon in this projection
    }
    tris.emplace_back(loop[idx[0]], loop[idx[1]], loop[idx[2]]);
    area += 0.5 * cross(p[idx[0]], p[idx[1]], p[idx[2]]);
    m.f.insert(m.f.end(), tris.begin(), tris.end());
    if (area_out) *area_out = area;
    return true;
}

RepairReport repair(Mesh& m, const Plane& plane, const RepairParams& p) {
    RepairReport rep;
    rep.faces_before = m.f.size();
    if (m.f.empty()) return rep;

    // 1. small components
    rep.components_removed = drop_small_components(m, p.min_component_fraction, plane, p.floating_fraction, p.plane_max);

    // 2. close what may be closed: the rim on the support plane, and small holes
    for (auto& loop : boundary_loops(m)) {
        double perim = 0;
        std::vector<double> hs;
        for (size_t i = 0; i < loop.size(); ++i) {
            perim += cv::norm(cv::Vec3d(m.v[loop[i]]) - cv::Vec3d(m.v[loop[(i + 1) % loop.size()]]));
            if (plane.found) hs.push_back(std::abs(plane.height(cv::Vec3d(m.v[loop[i]]))));
        }
        // The base rim: nearly all of it within plane_tol and no notch higher than plane_max.
        bool on_plane = false;
        double max_h = 0;
        if (!hs.empty()) {
            std::sort(hs.begin(), hs.end());
            max_h = hs.back();
            on_plane = hs[hs.size() * 95 / 100] <= p.plane_tol && max_h <= p.plane_max;
        }
        std::vector<int> rev(loop.rbegin(), loop.rend());  // new faces run against the existing winding
        if (on_plane) {
            std::vector<cv::Vec3f> saved;
            for (int v : loop) {
                saved.push_back(m.v[v]);
                const cv::Vec3d P = m.v[v];
                const cv::Vec3d S = P - plane.n * plane.height(P);  // onto the observed support plane
                m.v[v] = cv::Vec3f(float(S[0]), float(S[1]), float(S[2]));
            }
            // A noisy rim can cross itself once flattened. Smooth it along the plane a step at a time
            // until it triangulates, but never move a vertex more than 1 mm in-plane.
            double a = 0, moved = 0;
            bool done = false;
            for (int step = 0; step <= 12 && !done && moved <= 0.001; ++step) {
                if (fill_loop(m, rev, -plane.n, &a)) {
                    done = true;
                    break;
                }
                std::vector<cv::Vec3f> next(loop.size());
                for (size_t i = 0; i < loop.size(); ++i) {
                    const cv::Vec3f& pv = m.v[loop[(i + loop.size() - 1) % loop.size()]];
                    const cv::Vec3f& nv = m.v[loop[(i + 1) % loop.size()]];
                    next[i] = 0.5f * m.v[loop[i]] + 0.25f * (pv + nv);
                }
                for (size_t i = 0; i < loop.size(); ++i) {
                    m.v[loop[i]] = next[i];
                    moved = std::max(moved, cv::norm(cv::Vec3d(next[i]) - cv::Vec3d(saved[i])) -
                                                std::abs(plane.height(cv::Vec3d(saved[i]))));
                }
            }
            if (done) {
                rep.capped = true;
                rep.cap_area += a;
                rep.cap_max_snap = std::max(rep.cap_max_snap, max_h);
                rep.cap_max_smooth = std::max(rep.cap_max_smooth, std::max(moved, 0.0));
                continue;
            }
            for (size_t i = 0; i < loop.size(); ++i) m.v[loop[i]] = saved[i];
            if (perim > p.max_hole_perimeter)
                rep.warnings.push_back("the base outline could not be closed on the support plane");
        }
        if (perim <= p.max_hole_perimeter) {
            cv::Vec3d nw(0, 0, 0);  // Newell normal of the reversed loop
            for (size_t i = 0; i < rev.size(); ++i) {
                const cv::Vec3d a = m.v[rev[i]], b = m.v[rev[(i + 1) % rev.size()]];
                nw += cv::Vec3d((a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1]));
            }
            double a = 0;
            if (cv::norm(nw) > 0 && fill_loop(m, rev, nw, &a)) {
                ++rep.holes_filled;
                rep.holes_area += a;
                continue;
            }
        }
        ++rep.holes_open;
        rep.open_perimeter += perim;
    }
    if (rep.holes_open)
        rep.warnings.push_back(std::to_string(rep.holes_open) + " open hole(s), total rim " + mm(rep.open_perimeter) +
                               ": those surfaces were not seen or not measurable (low texture, poor light or too few views) and are left open");

    // 3. decimate to the face budget, keeping the topology it had
    // (also runs when only a few zero-area faces from the fills need collapsing)
    const Topology before = validate(m);
    if (m.f.size() > p.face_budget || before.degenerate_faces) {
        Mesh cand = m;
        decimate(cand, std::min(p.face_budget, m.f.size() - 2 * before.degenerate_faces), p.max_decimation_error, p.progress);
        const Topology after = validate(cand);
        // accepted when nothing got worse
        const bool kept = (after.closed_valid || !before.closed_valid) && after.degenerate_faces <= before.degenerate_faces &&
                          after.nonmanifold_edges <= before.nonmanifold_edges &&
                          after.nonmanifold_vertices <= before.nonmanifold_vertices &&
                          after.inconsistent_edges <= before.inconsistent_edges && after.components <= before.components &&
                          after.boundary_edges <= before.boundary_edges;
        if (kept) {
            m = std::move(cand);
            rep.decimated = true;
            if (m.f.size() > p.face_budget * 1.2)
                rep.warnings.push_back("decimation stopped at " + std::to_string(m.f.size()) +
                                       " faces to keep the shape within " + mm(p.max_decimation_error));
        } else {
            rep.warnings.push_back("decimation would have broken the topology; the full mesh is kept (" +
                                   std::to_string(m.f.size()) + " faces)");
        }
    }

    // 4. outward orientation
    const Topology t = validate(m);
    if (t.boundary_edges == 0 && t.inconsistent_edges == 0 && t.nonmanifold_edges == 0 && t.signed_volume < 0) {
        for (auto& f : m.f) std::swap(f[1], f[2]);
        rep.flipped = true;
    }
    return rep;
}

}  // namespace psom
