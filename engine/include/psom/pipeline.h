#pragma once
#include <filesystem>
#include <string>

#include "psom/mvs.h"
#include "psom/result.h"

namespace psom {

struct Options {
    bool force_mvs = false;   // ignore raw depth (tests the fallback)
    size_t mem_mb = 1024;     // TSDF memory budget
    size_t face_budget = 20000;
    bool guess = true;        // close what wasn't seen with a smooth guess, reported and marked on the mesh
    bool verbose = false;     // per-frame depth diagnostics on stderr
    MvsParams mvs;
};

// <project>/session -> <project>/recon/{depth,mesh.ply,report.json}. Never writes inside session/.
// project.json status moves to "reconstructed" only on success; on failure last_error is set.
Result reconstruct(const std::filesystem::path& project, const Options& o, const Progress& progress = {});

// recon/mesh.ply -> recon/model.step, using project.json's scale.
Result export_project(const std::filesystem::path& project, const Progress& progress = {});

// Compares report dimensions with truth.json {"dims_mm":[L,W,H]}; writes recon/evaluation.json.
Result evaluate(const std::filesystem::path& project, const std::filesystem::path& truth, std::string& summary);

}  // namespace psom
