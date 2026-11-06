#pragma once
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "psom/depth.h"
#include "psom/isolate.h"
#include "psom/mesh.h"

namespace psom {

struct TsdfParams {
    double voxel = 0.001;        // metres
    double trunc = 0.005;        // metres
    size_t budget_bytes = size_t(1) << 30;
    double min_weight = 0.5;     // a lattice point needs this much observation to be meshed
};

// Sparse TSDF over 8^3 voxel blocks, allocated only near observed surfaces inside the crop box.
// Every block is counted against the budget; allocation stops when it would be exceeded.
class SparseTsdf {
public:
    static constexpr int B = 8;
    struct Voxel {
        float sdf = 1.f;        // truncated, in units of trunc
        float w = 0.f;          // observation weight; kGuessed where fill_unseen() made it up
        float vx = 0, vy = 0, vz = 0;  // weighted sum of unit view directions
        float inv_var = 0.f;    // sum of 1/sigma^2 of the samples near the surface
    };
    struct Block { Voxel v[B * B * B]; };
    static constexpr float kGuessed = -1.f;

    struct FillReport {
        size_t voxels = 0;        // voxels guessed
        size_t specks = 0;        // observed voxels forgotten as noise (small pieces apart from the rest)
        double cell_m = 0;        // size of the grid the guess was solved on
        bool budget_hit = false;  // ran out of memory part way: some holes stay open
        bool skipped = false;     // no room even for the solve
    };

    SparseTsdf(const CropBox& box, const TsdfParams& p);
    // False once the budget is exhausted; the TSDF is then incomplete and must not be meshed.
    bool integrate(const DepthMap& m, const cv::Matx44d& T_wc);
    bool over_budget() const { return over_budget_; }
    size_t bytes() const;
    size_t blocks() const { return blocks_.size(); }
    static size_t block_bytes();
    // Guesses the surface nobody saw. The measured signed distance is spread into unseen space by
    // diffusion (Davis et al. 2002, "Filling holes in complex surfaces using volumetric diffusion"):
    // a smooth field that matches every observed voxel, whose zero crossing bridges each hole the
    // way a stretched membrane would. Voxels near that crossing are marked guessed (w = kGuessed) so
    // extract() meshes them, with weight 0 on their vertices. The crop's floor is left open; the
    // base cap closes it on the table. Run after the last integrate().
    FillReport fill_unseen(size_t max_bytes);
    // Meshes observed voxels and, after fill_unseen(), guessed ones.
    Mesh extract() const;

private:
    using Key = uint64_t;
    static Key key(int x, int y, int z);
    const Voxel* voxel_at(int gx, int gy, int gz) const;
    Block* get_or_alloc(int bx, int by, int bz);

    CropBox box_;
    TsdfParams p_;
    cv::Vec3i dims_;  // voxels per axis
    std::unordered_map<Key, uint32_t> index_;
    std::vector<std::unique_ptr<Block>> blocks_;
    std::vector<cv::Vec3i> block_coords_;
    bool over_budget_ = false;
};

}  // namespace psom
