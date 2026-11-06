#include "psom/tsdf.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <opencv2/core/utility.hpp>

namespace psom {

SparseTsdf::SparseTsdf(const CropBox& box, const TsdfParams& p) : box_(box), p_(p) {
    for (int k = 0; k < 3; ++k) dims_[k] = std::max(2, int(std::ceil(2 * box.half[k] / p.voxel)));
}

SparseTsdf::Key SparseTsdf::key(int x, int y, int z) {
    return (uint64_t(uint32_t(x) & 0x1FFFFF) << 42) | (uint64_t(uint32_t(y) & 0x1FFFFF) << 21) |
           uint64_t(uint32_t(z) & 0x1FFFFF);
}

size_t SparseTsdf::block_bytes() {
    // the block, its pointer, its coordinates and a hash node
    return sizeof(Block) + sizeof(std::unique_ptr<Block>) + sizeof(cv::Vec3i) + 48;
}

size_t SparseTsdf::bytes() const { return blocks_.size() * block_bytes(); }

SparseTsdf::Block* SparseTsdf::get_or_alloc(int bx, int by, int bz) {
    const Key k = key(bx, by, bz);
    auto it = index_.find(k);
    if (it != index_.end()) return blocks_[it->second].get();
    if ((blocks_.size() + 1) * block_bytes() > p_.budget_bytes) {
        over_budget_ = true;
        return nullptr;
    }
    index_.emplace(k, uint32_t(blocks_.size()));
    blocks_.push_back(std::make_unique<Block>());
    block_coords_.push_back({bx, by, bz});
    return blocks_.back().get();
}

const SparseTsdf::Voxel* SparseTsdf::voxel_at(int gx, int gy, int gz) const {
    if (gx < 0 || gy < 0 || gz < 0 || gx >= dims_[0] || gy >= dims_[1] || gz >= dims_[2]) return nullptr;
    auto it = index_.find(key(gx / B, gy / B, gz / B));
    if (it == index_.end()) return nullptr;
    return &blocks_[it->second]->v[(gx % B) + B * ((gy % B) + B * (gz % B))];
}

bool SparseTsdf::integrate(const DepthMap& m, const cv::Matx44d& T_wc) {
    if (over_budget_) return false;
    const cv::Matx33d R = T_wc.get_minor<3, 3>(0, 0);
    const cv::Vec3d C(T_wc(0, 3), T_wc(1, 3), T_wc(2, 3));
    const double v = p_.voxel, mu = p_.trunc;

    // 1. allocate the blocks this frame's truncation band touches
    std::vector<uint32_t> touched;
    std::vector<uint8_t> seen(blocks_.size(), 0);
    auto touch = [&](Block* b, int bx, int by, int bz) {
        const uint32_t id = index_.at(key(bx, by, bz));
        if (id >= seen.size()) seen.resize(id + 1, 0);
        if (!seen[id]) seen[id] = 1, touched.push_back(id);
        (void)b;
    };
    const double step = v * B * 0.5;
    for (int y = 0; y < m.K.h; ++y)
        for (int x = 0; x < m.K.w; ++x) {
            const float z = m.d[size_t(y) * m.K.w + x];
            if (z <= 0) continue;
            const cv::Vec3d X = R * (m.K.ray(x, y) * double(z)) + C;
            const cv::Vec3d dir = cv::normalize(X - C);
            for (double t = -mu; t <= mu + 1e-12; t += step) {
                const cv::Vec3d L = box_.to_local(X + dir * t) + box_.half;
                const int gx = int(std::floor(L[0] / v)), gy = int(std::floor(L[1] / v)), gz = int(std::floor(L[2] / v));
                if (gx < 0 || gy < 0 || gz < 0 || gx >= dims_[0] || gy >= dims_[1] || gz >= dims_[2]) continue;
                Block* b = get_or_alloc(gx / B, gy / B, gz / B);
                if (!b) return false;
                touch(b, gx / B, gy / B, gz / B);
            }
        }

    // 2. update every voxel of the touched blocks
    const cv::Matx33d Rt = R.t();
    for (uint32_t id : touched) {
        const cv::Vec3i bc = block_coords_[id];
        Block& blk = *blocks_[id];
        for (int lz = 0; lz < B; ++lz)
            for (int ly = 0; ly < B; ++ly)
                for (int lx = 0; lx < B; ++lx) {
                    const int gx = bc[0] * B + lx, gy = bc[1] * B + ly, gz = bc[2] * B + lz;
                    if (gx >= dims_[0] || gy >= dims_[1] || gz >= dims_[2]) continue;
                    const cv::Vec3d L((gx + 0.5) * v - box_.half[0], (gy + 0.5) * v - box_.half[1],
                                      (gz + 0.5) * v - box_.half[2]);
                    const cv::Vec3d P = box_.to_world(L);
                    const cv::Vec3d Xc = Rt * (P - C);
                    if (Xc[2] <= 0.01) continue;
                    const cv::Vec2d uv = m.K.project(Xc);
                    const int u = int(std::lround(uv[0])), w = int(std::lround(uv[1]));
                    if (u < 0 || w < 0 || u >= m.K.w || w >= m.K.h) continue;
                    const size_t px = size_t(w) * m.K.w + u;
                    const double zm = m.d[px];
                    if (zm <= 0) continue;
                    const double sdf = zm - Xc[2];
                    if (sdf < -mu) continue;  // behind the surface: unobserved
                    const double sig = std::max(double(m.sigma[px]), 1e-4);
                    const double ws = std::clamp((0.001 / sig) * (0.001 / sig), 0.05, 1.0);
                    Voxel& vx = blk.v[lx + B * (ly + B * lz)];
                    const double tsdf = std::min(1.0, sdf / mu);
                    vx.sdf = float((vx.sdf * vx.w + tsdf * ws) / (vx.w + ws));
                    vx.w = float(vx.w + ws);
                    if (std::abs(sdf) <= mu) {
                        const cv::Vec3d view = cv::normalize(C - P);
                        vx.vx += float(ws * view[0]);
                        vx.vy += float(ws * view[1]);
                        vx.vz += float(ws * view[2]);
                        vx.inv_var += float(1.0 / (sig * sig));
                    }
                }
    }
    return true;
}

SparseTsdf::FillReport SparseTsdf::fill_unseen(size_t max_bytes) {
    FillReport rep;
    if (over_budget_) return rep;
    // The solve runs on a dense grid of cells f voxels wide; f grows until it fits in max_bytes.
    int f = 1;
    cv::Vec3i n;
    auto cells = [&] {
        for (int k = 0; k < 3; ++k) n[k] = (dims_[k] + f - 1) / f;
        return size_t(n[0]) * n[1] * n[2];
    };
    while (cells() * 6 * 8 / 7 > max_bytes && f < 8) ++f;  // val + fixed + band, plus the coarser levels
    if (cells() * 6 * 8 / 7 > max_bytes) return rep.skipped = true, rep;
    rep.cell_m = f * p_.voxel;

    // 1. observed cells: the mean of their observed voxels, held fixed
    struct Level {
        cv::Vec3i n;
        std::vector<float> val;
        std::vector<uint8_t> fixed;
        size_t at(int x, int y, int z) const { return size_t(x) + size_t(n[0]) * (size_t(y) + size_t(n[1]) * z); }
    };
    std::vector<Level> lv(1);
    lv[0].n = n;
    lv[0].val.assign(cells(), 0.f);
    lv[0].fixed.assign(cells(), 0);
    {
        std::vector<uint16_t> cnt(cells(), 0);
        for (size_t bi = 0; bi < blocks_.size(); ++bi) {
            const cv::Vec3i bc = block_coords_[bi];
            for (int l = 0; l < B * B * B; ++l) {
                const Voxel& vx = blocks_[bi]->v[l];
                if (vx.w < p_.min_weight) continue;
                const int gx = bc[0] * B + l % B, gy = bc[1] * B + (l / B) % B, gz = bc[2] * B + l / (B * B);
                if (gx >= dims_[0] || gy >= dims_[1] || gz >= dims_[2]) continue;
                const size_t c = lv[0].at(gx / f, gy / f, gz / f);
                lv[0].val[c] += vx.sdf;
                if (cnt[c] < 65535) ++cnt[c];
            }
        }
        for (size_t c = 0; c < cells(); ++c)
            if (cnt[c]) lv[0].val[c] /= cnt[c], lv[0].fixed[c] = 1;
    }
    // Specks of noise would anchor the guess and grow tentacles to reach it: forget near-surface
    // pieces much smaller than the biggest one (repair would drop them as fragments anyway).
    {
        Level& L = lv[0];
        std::vector<int> comp(cells(), -1);
        std::vector<size_t> sizes;
        std::vector<size_t> stack;
        auto surf = [&](size_t i) { return L.fixed[i] && std::abs(L.val[i]) < 0.99f; };
        for (size_t s0 = 0; s0 < cells(); ++s0) {
            if (comp[s0] >= 0 || !surf(s0)) continue;
            const int id = int(sizes.size());
            size_t count = 0;
            comp[s0] = id, stack = {s0};
            while (!stack.empty()) {
                const size_t i = stack.back();
                stack.pop_back();
                ++count;
                const int x = int(i % n[0]), y = int((i / n[0]) % n[1]), z = int(i / (size_t(n[0]) * n[1]));
                for (int dz = -1; dz <= 1; ++dz)
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dx = -1; dx <= 1; ++dx) {
                            const int xx = x + dx, yy = y + dy, zz = z + dz;
                            if (xx < 0 || yy < 0 || zz < 0 || xx >= n[0] || yy >= n[1] || zz >= n[2]) continue;
                            const size_t j = L.at(xx, yy, zz);
                            if (comp[j] < 0 && surf(j)) comp[j] = id, stack.push_back(j);
                        }
            }
            sizes.push_back(count);
        }
        const size_t biggest = sizes.empty() ? 0 : *std::max_element(sizes.begin(), sizes.end());
        std::vector<uint8_t> drop(cells(), 0);
        for (size_t i = 0; i < cells(); ++i)
            if (comp[i] >= 0 && sizes[comp[i]] * 20 < biggest) drop[i] = 1, L.fixed[i] = 0, L.val[i] = 0;
        for (size_t bi = 0; bi < blocks_.size(); ++bi) {
            const cv::Vec3i bc = block_coords_[bi];
            for (int l = 0; l < B * B * B; ++l) {
                Voxel& vx = blocks_[bi]->v[l];
                const int gx = bc[0] * B + l % B, gy = bc[1] * B + (l / B) % B, gz = bc[2] * B + l / (B * B);
                if (vx.w < p_.min_weight || gx >= dims_[0] || gy >= dims_[1] || gz >= dims_[2]) continue;
                if (drop[L.at(gx / f, gy / f, gz / f)]) vx = Voxel{}, ++rep.specks;
            }
        }
    }
    // coarser levels, each half the size, for a quick first guess that the finer ones refine
    while (std::max({lv.back().n[0], lv.back().n[1], lv.back().n[2]}) > 16) {
        const Level& a = lv.back();
        Level b;
        for (int k = 0; k < 3; ++k) b.n[k] = (a.n[k] + 1) / 2;
        const size_t nb = size_t(b.n[0]) * b.n[1] * b.n[2];
        b.val.assign(nb, 0.f);
        b.fixed.assign(nb, 0);
        std::vector<uint8_t> cnt(nb, 0);
        for (int z = 0; z < a.n[2]; ++z)
            for (int y = 0; y < a.n[1]; ++y)
                for (int x = 0; x < a.n[0]; ++x) {
                    const size_t i = a.at(x, y, z);
                    if (!a.fixed[i]) continue;
                    const size_t j = b.at(x / 2, y / 2, z / 2);
                    b.val[j] += a.val[i], ++cnt[j];
                }
        for (size_t j = 0; j < nb; ++j)
            if (cnt[j]) b.val[j] /= cnt[j], b.fixed[j] = 1;
        lv.push_back(std::move(b));
    }

    // 2. Laplace on the free cells (a membrane), red-black SOR, coarse to fine. Outside the box is
    // empty space (+1); below the floor the field just continues (no flux), so the surface meets the
    // floor instead of closing. A thin plate (biharmonic) was tried: it carries curvature on, but
    // also every bump along a hole's noisy edge, and grows blobs.
    auto relax = [&](Level& L, int iterations) {
        const int nx = L.n[0], ny = L.n[1], nz = L.n[2];
        for (int it = 0; it < iterations; ++it)
            for (int colour = 0; colour < 2; ++colour)
                cv::parallel_for_(cv::Range(0, nz), [&](const cv::Range& r) {
                    for (int z = r.start; z < r.end; ++z)
                        for (int y = 0; y < ny; ++y)
                            for (int x = (y + z + colour) & 1; x < nx; x += 2) {
                                const size_t i = L.at(x, y, z);
                                if (L.fixed[i]) continue;
                                float sum = 0;
                                int k = 0;
                                auto nb = [&](int xx, int yy, int zz) {
                                    if (zz < 0) return;  // floor: no flux
                                    if (xx < 0 || yy < 0 || xx >= nx || yy >= ny || zz >= nz) sum += 1.f;
                                    else sum += L.val[L.at(xx, yy, zz)];
                                    ++k;
                                };
                                nb(x - 1, y, z), nb(x + 1, y, z), nb(x, y - 1, z), nb(x, y + 1, z), nb(x, y, z - 1), nb(x, y, z + 1);
                                L.val[i] += 1.8f * (sum / k - L.val[i]);
                            }
                });
    };
    {
        Level& top = lv.back();
        for (size_t i = 0; i < top.val.size(); ++i)
            if (!top.fixed[i]) top.val[i] = 1.f;
        relax(top, 200);
    }
    for (int l = int(lv.size()) - 2; l >= 0; --l) {
        Level& a = lv[l];
        const Level& b = lv[l + 1];
        for (int z = 0; z < a.n[2]; ++z)
            for (int y = 0; y < a.n[1]; ++y)
                for (int x = 0; x < a.n[0]; ++x) {
                    const size_t i = a.at(x, y, z);
                    if (!a.fixed[i]) a.val[i] = b.val[b.at(x / 2, y / 2, z / 2)];
                }
        relax(a, l == 0 ? 40 : 60);
    }

    // 3. write back the free cells along the field's zero crossing, one cell either side
    Level& L = lv[0];
    std::vector<uint8_t> band(L.val.size(), 0);
    for (int z = 0; z < n[2]; ++z)
        for (int y = 0; y < n[1]; ++y)
            for (int x = 0; x < n[0]; ++x) {
                const size_t i = L.at(x, y, z);
                const bool neg = L.val[i] < 0;
                bool cross = false;
                if (x + 1 < n[0]) cross |= (L.val[L.at(x + 1, y, z)] < 0) != neg;
                if (y + 1 < n[1]) cross |= (L.val[L.at(x, y + 1, z)] < 0) != neg;
                if (z + 1 < n[2]) cross |= (L.val[L.at(x, y, z + 1)] < 0) != neg;
                if (!cross) continue;
                for (int dz = -1; dz <= 2; ++dz)
                    for (int dy = -1; dy <= 2; ++dy)
                        for (int dx = -1; dx <= 2; ++dx) {
                            const int xx = x + dx, yy = y + dy, zz = z + dz;
                            if (xx >= 0 && yy >= 0 && zz >= 0 && xx < n[0] && yy < n[1] && zz < n[2]) band[L.at(xx, yy, zz)] = 1;
                        }
            }
    auto field = [&](int gx, int gy, int gz) {  // trilinear, at a voxel centre
        const double p[3] = {(gx + 0.5) / f - 0.5, (gy + 0.5) / f - 0.5, (gz + 0.5) / f - 0.5};
        int i0[3];
        double t[3];
        for (int k = 0; k < 3; ++k) {
            const double q = std::clamp(p[k], 0.0, double(n[k] - 1));
            i0[k] = std::min(int(q), n[k] - 2 < 0 ? 0 : n[k] - 2);
            t[k] = n[k] > 1 ? q - i0[k] : 0;
        }
        double s = 0;
        for (int c = 0; c < 8; ++c) {
            const int x = std::min(i0[0] + (c & 1), n[0] - 1), y = std::min(i0[1] + ((c >> 1) & 1), n[1] - 1),
                      z = std::min(i0[2] + ((c >> 2) & 1), n[2] - 1);
            const double wgt = ((c & 1) ? t[0] : 1 - t[0]) * ((c & 2) ? t[1] : 1 - t[1]) * ((c & 4) ? t[2] : 1 - t[2]);
            s += wgt * L.val[L.at(x, y, z)];
        }
        return float(std::clamp(s, -1.0, 1.0));
    };
    for (int z = 0; z < n[2]; ++z)
        for (int y = 0; y < n[1]; ++y)
            for (int x = 0; x < n[0]; ++x) {
                if (!band[L.at(x, y, z)]) continue;
                for (int gz = z * f; gz < std::min(dims_[2], (z + 1) * f); ++gz)
                    for (int gy = y * f; gy < std::min(dims_[1], (y + 1) * f); ++gy)
                        for (int gx = x * f; gx < std::min(dims_[0], (x + 1) * f); ++gx) {
                            const Key k = key(gx / B, gy / B, gz / B);
                            auto it = index_.find(k);
                            Block* b = nullptr;
                            if (it != index_.end()) b = blocks_[it->second].get();
                            else if ((blocks_.size() + 1) * block_bytes() <= p_.budget_bytes) b = get_or_alloc(gx / B, gy / B, gz / B);
                            if (!b) {
                                rep.budget_hit = true;
                                continue;
                            }
                            Voxel& vx = b->v[(gx % B) + B * ((gy % B) + B * (gz % B))];
                            if (vx.w >= p_.min_weight || vx.w == kGuessed) continue;
                            vx = Voxel{};
                            vx.sdf = field(gx, gy, gz);
                            vx.w = kGuessed;
                            ++rep.voxels;
                        }
            }
    return rep;
}

Mesh SparseTsdf::extract() const {
    Mesh mesh;
    if (over_budget_) return mesh;
    struct Corner {
        cv::Vec3d P;
        double s, w, div, sig;
    };
    const double v = p_.voxel;
    auto corner = [&](int gx, int gy, int gz, const Voxel& vx) {
        const cv::Vec3d L((gx + 0.5) * v - box_.half[0], (gy + 0.5) * v - box_.half[1], (gz + 0.5) * v - box_.half[2]);
        Corner c;
        c.P = box_.to_world(L);
        c.s = std::abs(vx.sdf) < 1e-6 ? 1e-6 : vx.sdf;  // keep vertices strictly inside edges
        c.w = std::max(0.f, vx.w);
        const double vn = std::sqrt(double(vx.vx) * vx.vx + double(vx.vy) * vx.vy + double(vx.vz) * vx.vz);
        c.div = vx.w > 0 ? std::clamp(1.0 - vn / vx.w, 0.0, 1.0) : 0.0;
        c.sig = vx.inv_var > 0 ? 1.0 / std::sqrt(double(vx.inv_var)) : 0.0;
        return c;
    };
    const uint64_t nx = dims_[0], ny = dims_[1];
    auto lattice = [&](int gx, int gy, int gz) { return uint64_t(gx) + nx * (uint64_t(gy) + ny * uint64_t(gz)); };
    std::unordered_map<uint64_t, int> edge_vertex;
    std::vector<cv::Vec3d> vpos;  // double positions, so orientation tests on tiny faces stay exact enough
    // Kuhn split of the cube into 6 tetrahedra around the 0-7 diagonal; consistent across neighbours,
    // so the surface comes out welded and manifold.
    static const int tets[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 2, 3, 7}, {0, 2, 6, 7}, {0, 4, 5, 7}, {0, 4, 6, 7}};

    for (size_t bi = 0; bi < blocks_.size(); ++bi) {
        const cv::Vec3i bc = block_coords_[bi];
        for (int lz = 0; lz < B; ++lz)
            for (int ly = 0; ly < B; ++ly)
                for (int lx = 0; lx < B; ++lx) {
                    const int gx = bc[0] * B + lx, gy = bc[1] * B + ly, gz = bc[2] * B + lz;
                    if (gx + 1 >= dims_[0] || gy + 1 >= dims_[1] || gz + 1 >= dims_[2]) continue;
                    const Voxel* vs[8];
                    bool ok = true, pos = false, neg = false;
                    for (int c = 0; c < 8 && ok; ++c) {
                        vs[c] = voxel_at(gx + (c & 1), gy + ((c >> 1) & 1), gz + ((c >> 2) & 1));
                        if (!vs[c] || (vs[c]->w < p_.min_weight && vs[c]->w != kGuessed)) ok = false;  // unobserved: a hole
                        else (vs[c]->sdf < 0 ? neg : pos) = true;
                    }
                    if (!ok || !pos || !neg) continue;
                    Corner cs[8];
                    uint64_t ids[8];
                    for (int c = 0; c < 8; ++c) {
                        const int cx = gx + (c & 1), cy = gy + ((c >> 1) & 1), cz = gz + ((c >> 2) & 1);
                        cs[c] = corner(cx, cy, cz, *vs[c]);
                        ids[c] = lattice(cx, cy, cz);
                    }
                    auto vert = [&](int a, int b) {
                        const Corner &A = cs[a], &Bc = cs[b];
                        // kept off the lattice points, so no face around a point degenerates to zero area
                        const double t = std::clamp(A.s / (A.s - Bc.s), 0.01, 0.99);
                        const uint64_t lo = std::min(ids[a], ids[b]), hi = std::max(ids[a], ids[b]);
                        const uint64_t k = (lo << 32) ^ hi;
                        auto it = edge_vertex.find(k);
                        if (it != edge_vertex.end()) return it->second;
                        const cv::Vec3d P = A.P + (Bc.P - A.P) * t;
                        const int id = int(mesh.v.size());
                        mesh.v.emplace_back(float(P[0]), float(P[1]), float(P[2]));
                        vpos.push_back(P);
                        mesh.weight.push_back(float(A.w + (Bc.w - A.w) * t));
                        mesh.diversity.push_back(float(A.div + (Bc.div - A.div) * t));
                        mesh.sigma.push_back(float(A.sig + (Bc.sig - A.sig) * t));
                        edge_vertex.emplace(k, id);
                        return id;
                    };
                    auto add = [&](int a, int b, int c) {
                        if (a != b && b != c && a != c) mesh.f.emplace_back(a, b, c);
                    };
                    for (auto& tet : tets) {
                        int in[4], out[4], ni = 0, no = 0;
                        for (int q = 0; q < 4; ++q) (cs[tet[q]].s < 0 ? in[ni++] : out[no++]) = tet[q];
                        if (ni == 0 || no == 0) continue;
                        cv::Vec3d ci(0, 0, 0), co(0, 0, 0);
                        for (int q = 0; q < ni; ++q) ci += cs[in[q]].P;
                        for (int q = 0; q < no; ++q) co += cs[out[q]].P;
                        const cv::Vec3d outward = co / double(no) - ci / double(ni);
                        // orient each polygon as a whole so the quad's diagonal is shared consistently
                        auto area = [&](int a, int b, int c) { return (vpos[b] - vpos[a]).cross(vpos[c] - vpos[a]); };
                        if (ni == 1 || no == 1) {
                            const int* one = ni == 1 ? in : out;
                            const int* three = ni == 1 ? out : in;
                            int a = vert(one[0], three[0]), b = vert(one[0], three[1]), c = vert(one[0], three[2]);
                            if (area(a, b, c).dot(outward) < 0) std::swap(b, c);
                            add(a, b, c);
                        } else {
                            int q[4] = {vert(in[0], out[0]), vert(in[0], out[1]), vert(in[1], out[1]), vert(in[1], out[0])};
                            if ((area(q[0], q[1], q[2]) + area(q[0], q[2], q[3])).dot(outward) < 0) std::swap(q[1], q[3]);
                            add(q[0], q[1], q[2]);
                            add(q[0], q[2], q[3]);
                        }
                    }
                }
    }
    return mesh;
}

}  // namespace psom
