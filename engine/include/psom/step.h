#pragma once
#include <filesystem>
#include <string>

#include "psom/mesh.h"
#include "psom/repair.h"
#include "psom/result.h"

namespace psom {

struct StepInfo {
    bool solid = false;
    size_t faces = 0;
    double volume_mm3 = 0;
    cv::Vec3d bbox_mm{0, 0, 0};
};

// Faceted B-rep: one planar face per triangle, edges shared exactly. A solid only when the mesh is
// closed_valid AND the B-rep has no free or multiply-shared edges AND BRepCheck passes; otherwise
// an open shell with a warning. The file is read back and checked before it replaces `out`.
Result export_step(const Mesh& m, double m_to_mm, const Topology& topo, const std::filesystem::path& out,
                   StepInfo& info);

// Reads a STEP back: counts solids and shells, checks validity, measures volume and bbox.
struct StepCheck {
    bool read = false, valid = false;
    int solids = 0, shells = 0;
    size_t faces = 0;
    double volume_mm3 = 0;
    cv::Vec3d bmin{0, 0, 0}, bmax{0, 0, 0};
};
StepCheck check_step(const std::filesystem::path& p);

}  // namespace psom
