#pragma once
#include <functional>
#include <string>
#include <vector>

#include "psom/isolate.h"
#include "psom/mesh.h"

namespace psom {

struct Topology {
    size_t vertices = 0, faces = 0;
    size_t boundary_edges = 0, nonmanifold_edges = 0, inconsistent_edges = 0;
    size_t nonmanifold_vertices = 0, degenerate_faces = 0, components = 0;
    double signed_volume = 0;
    bool closed_valid = false;
    std::vector<std::string> reasons;  // why it isn't closed_valid
};

// min_area in m^2: faces smaller than this count as degenerate.
Topology validate(const Mesh& m, double min_area = 1e-14);

struct RepairParams {
    double plane_tol = 0.005;          // 95 % of a loop this close to the support plane: it's the base rim
    double plane_max = 0.007;          // and no part of it higher than this
    double max_hole_perimeter = 0.03;
    size_t face_budget = 20000;
    double max_decimation_error = 0.0003;  // metres a collapse may move the surface
    double min_component_fraction = 0.02;
    std::function<void(double)> progress;  // 0..1 through decimation, the slow part
};

struct RepairReport {
    int components_removed = 0;
    bool capped = false;
    double cap_area = 0;      // m^2
    double cap_max_snap = 0;  // m, the furthest a rim vertex moved onto the plane
    double cap_max_smooth = 0;  // m, the furthest a rim vertex moved along the plane to untangle it
    int holes_filled = 0;
    double holes_area = 0;    // m^2
    int holes_open = 0;
    double open_perimeter = 0;
    size_t faces_before = 0;
    bool decimated = false;
    bool flipped = false;
    std::vector<std::string> warnings;
};

RepairReport repair(Mesh& m, const Plane& plane, const RepairParams& p);

// Boundary loops as vertex chains following the faces' winding.
std::vector<std::vector<int>> boundary_loops(const Mesh& m);

// Ear-clips a loop projected onto the plane with normal n; triangles face along n. False if it can't.
bool fill_loop(Mesh& m, const std::vector<int>& loop, const cv::Vec3d& n, double* area = nullptr);

}  // namespace psom
