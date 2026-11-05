#include "psom/step.h"

#include <cmath>
#include <unordered_map>

#include <APIHeaderSection_MakeHeader.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <Interface_HArray1OfHAsciiString.hxx>
#include <Interface_Static.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Wire.hxx>
#include <functional>
#include <numeric>
#include <gp_Pln.hxx>

namespace fs = std::filesystem;

namespace psom {

namespace {

void units_mm() {
    // Open CASCADE prints transfer statistics on stdout; the CLI's stdout is JSON.
    static const bool quiet = [] {
        Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
        return true;
    }();
    (void)quiet;
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");
    Interface_Static::SetCVal("write.step.unit", "MM");
}

constexpr const char* kFaceted =
    "Faceted B-rep from a scan (psom Mode A): planar triangles, not a feature-based parametric model.";

}  // namespace

StepCheck check_step(const fs::path& p) {
    StepCheck c;
    try {
        units_mm();
        STEPControl_Reader r;
        if (r.ReadFile(p.string().c_str()) != IFSelect_RetDone) return c;
        r.TransferRoots();
        const TopoDS_Shape s = r.OneShape();
        if (s.IsNull()) return c;
        c.read = true;
        for (TopExp_Explorer e(s, TopAbs_SOLID); e.More(); e.Next()) ++c.solids;
        for (TopExp_Explorer e(s, TopAbs_SHELL); e.More(); e.Next()) ++c.shells;
        for (TopExp_Explorer e(s, TopAbs_FACE); e.More(); e.Next()) ++c.faces;
        c.valid = BRepCheck_Analyzer(s).IsValid();
        if (c.solids) {
            GProp_GProps g;
            BRepGProp::VolumeProperties(s, g);
            c.volume_mm3 = g.Mass();
        }
        Bnd_Box b;
        BRepBndLib::AddOptimal(s, b, false, false);
        double x0, y0, z0, x1, y1, z1;
        b.Get(x0, y0, z0, x1, y1, z1);
        c.bmin = {x0, y0, z0};
        c.bmax = {x1, y1, z1};
    } catch (const Standard_Failure&) {
        c.valid = false;
    }
    return c;
}

Result export_step(const Mesh& m, double s, const Topology& topo, const fs::path& out, StepInfo& info) {
    Result r;
    r.stage = "export";
    if (m.f.empty()) return Result::fail("export", "the mesh is empty");
    const fs::path tmp = out.string() + ".tmp";
    try {
        units_mm();
        BRep_Builder B;
        std::vector<TopoDS_Vertex> V(m.v.size());
        for (size_t i = 0; i < m.v.size(); ++i)
            V[i] = BRepBuilderAPI_MakeVertex(gp_Pnt(m.v[i][0] * s, m.v[i][1] * s, m.v[i][2] * s));
        std::unordered_map<uint64_t, TopoDS_Edge> E;
        auto edge = [&](int a, int b) -> TopoDS_Edge {
            const int lo = std::min(a, b), hi = std::max(a, b);
            const uint64_t k = (uint64_t(lo) << 32) | uint32_t(hi);
            auto it = E.find(k);
            if (it == E.end()) {
                BRepBuilderAPI_MakeEdge me(V[lo], V[hi]);
                if (!me.IsDone()) return {};
                it = E.emplace(k, me.Edge()).first;
            }
            return a == lo ? it->second : TopoDS::Edge(it->second.Reversed());
        };
        TopoDS_Shell shell;
        B.MakeShell(shell);
        size_t failed = 0, faces = 0;
        std::vector<TopoDS_Face> built;
        std::vector<int> built_tri;  // mesh face of each built face
        for (auto& t : m.f) {
            const gp_Pnt a = BRep_Tool::Pnt(V[t[0]]), b = BRep_Tool::Pnt(V[t[1]]), c = BRep_Tool::Pnt(V[t[2]]);
            const gp_Vec n = gp_Vec(a, b).Crossed(gp_Vec(a, c));
            if (n.Magnitude() < 1e-12) {
                ++failed;
                continue;
            }
            TopoDS_Wire w;
            B.MakeWire(w);
            bool ok = true;
            for (int k = 0; k < 3 && ok; ++k) {
                const TopoDS_Edge e = edge(t[k], t[(k + 1) % 3]);
                if (e.IsNull()) ok = false;
                else B.Add(w, e);
            }
            if (!ok) {
                ++failed;
                continue;
            }
            w.Closed(true);
            BRepBuilderAPI_MakeFace mf(gp_Pln(a, gp_Dir(n)), w, true);
            if (!mf.IsDone()) {
                ++failed;
                continue;
            }
            B.Add(shell, mf.Face());
            built.push_back(mf.Face());
            built_tri.push_back(int(&t - &m.f[0]));
            ++faces;
        }
        if (faces == 0) return Result::fail("export", "no face could be built");

        // edge sharing as the B-rep sees it
        TopTools_IndexedDataMapOfShapeListOfShape ef;
        TopExp::MapShapesAndAncestors(shell, TopAbs_EDGE, TopAbs_FACE, ef);
        size_t free_edges = 0, multi_edges = 0;
        for (int i = 1; i <= ef.Extent(); ++i) {
            const int n = ef.FindFromIndex(i).Extent();
            free_edges += n == 1;
            multi_edges += n > 2;
        }

        TopoDS_Shape shape = shell;
        bool solid = false;
        if (topo.closed_valid && failed == 0 && free_edges == 0 && multi_edges == 0) {
            shell.Closed(true);
            BRepBuilderAPI_MakeSolid ms(shell);
            if (ms.IsDone()) {
                TopoDS_Solid so = ms.Solid();
                BRepLib::OrientClosedSolid(so);
                if (BRepCheck_Analyzer(so).IsValid()) {
                    shape = so;
                    solid = true;
                } else {
                    r.warn("the closed shell failed solid validation; exported as an open shell");
                }
            }
        }
        if (!solid) {
            std::string why;
            for (auto& x : topo.reasons) why += (why.empty() ? "" : "; ") + x;
            if (failed) why += (why.empty() ? "" : "; ") + std::to_string(failed) + " faces could not be built";
            if (free_edges) why += (why.empty() ? "" : "; ") + std::to_string(free_edges) + " free edges";
            if (multi_edges) why += (why.empty() ? "" : "; ") + std::to_string(multi_edges) + " edges shared by 3+ faces";
            r.warn("exported as an OPEN SHELL, not a solid: " + (why.empty() ? std::string("not closed") : why) +
                   ". Missing surfaces were not invented.");
            // One open shell per edge-connected patch: patches that only touch at a vertex, or edges
            // shared by three faces, can't live in one valid shell.
            std::unordered_map<uint64_t, std::vector<int>> by_edge;
            for (int q = 0; q < int(built.size()); ++q) {
                const auto& t = m.f[built_tri[q]];
                for (int k = 0; k < 3; ++k) {
                    const int a = t[k], b = t[(k + 1) % 3];
                    by_edge[(uint64_t(std::min(a, b)) << 32) | uint32_t(std::max(a, b))].push_back(q);
                }
            }
            std::vector<int> parent(built.size());
            std::iota(parent.begin(), parent.end(), 0);
            std::function<int(int)> find = [&](int x) { return parent[x] == x ? x : parent[x] = find(parent[x]); };
            for (auto& [_, fs2] : by_edge)
                if (fs2.size() == 2) parent[find(fs2[0])] = find(fs2[1]);
            std::unordered_map<int, TopoDS_Shell> shells;
            for (int q = 0; q < int(built.size()); ++q) {
                auto [it, fresh] = shells.try_emplace(find(q));
                if (fresh) B.MakeShell(it->second);
                B.Add(it->second, built[q]);
            }
            TopoDS_Compound comp;
            B.MakeCompound(comp);
            for (auto& [_, sh] : shells) {
                if (!BRepCheck_Analyzer(sh).IsValid()) return Result::fail("export", "an open shell failed B-rep validation");
                B.Add(comp, sh);
            }
            if (shells.size() > 1) r.warn("the surface is in " + std::to_string(shells.size()) + " separate patches");
            shape = comp;
        }

        STEPControl_Writer w;
        if (w.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone) return Result::fail("export", "STEP transfer failed");
        APIHeaderSection_MakeHeader hdr(w.Model());
        hdr.SetName(new TCollection_HAsciiString("psom scan"));
        hdr.SetOriginatingSystem(new TCollection_HAsciiString("pseudo-stereographic-object-mapping"));
        Handle(Interface_HArray1OfHAsciiString) desc = new Interface_HArray1OfHAsciiString(1, solid ? 1 : 2);
        desc->SetValue(1, new TCollection_HAsciiString(kFaceted));
        if (!solid) desc->SetValue(2, new TCollection_HAsciiString("OPEN SHELL: parts of the object were not seen."));
        hdr.SetDescription(desc);
        if (w.Write(tmp.string().c_str()) != IFSelect_RetDone) return Result::fail("export", "STEP write failed");

        // A written file is not success: read it back and compare with the mesh.
        const StepCheck c = check_step(tmp);
        cv::Vec3d mn(1e300, 1e300, 1e300), mx(-1e300, -1e300, -1e300);
        for (auto& v : m.v)
            for (int k = 0; k < 3; ++k) mn[k] = std::min(mn[k], v[k] * s), mx[k] = std::max(mx[k], v[k] * s);
        auto fail_tmp = [&](const std::string& e) {
            std::error_code ec;
            fs::remove(tmp, ec);
            return Result::fail("export", e);
        };
        if (!c.read) return fail_tmp("the written STEP could not be read back");
        if (!c.valid) return fail_tmp("the STEP read back is not a valid B-rep");
        if (solid ? c.solids != 1 : (c.solids != 0 || c.shells < 1)) return fail_tmp("the STEP read back has the wrong shape type");
        if (c.faces != faces) return fail_tmp("the STEP read back has a different face count");
        for (int k = 0; k < 3; ++k)
            if (std::abs(c.bmin[k] - mn[k]) > 0.01 || std::abs(c.bmax[k] - mx[k]) > 0.01)
                return fail_tmp("the STEP read back does not match the mesh's bounding box");
        const double mesh_vol = signed_volume(m) * s * s * s;
        if (solid && std::abs(c.volume_mm3 - mesh_vol) > 0.01 * std::abs(mesh_vol))
            return fail_tmp("the STEP solid's volume differs from the mesh's by more than 1 %");

        std::error_code ec;
        fs::rename(tmp, out, ec);
        if (ec) return fail_tmp("could not move the STEP into place: " + ec.message());
        info.solid = solid;
        info.faces = faces;
        info.volume_mm3 = solid ? c.volume_mm3 : 0;
        info.bbox_mm = mx - mn;
    } catch (const Standard_Failure& e) {
        std::error_code ec;
        fs::remove(tmp, ec);
        return Result::fail("export", std::string("Open CASCADE error: ") + e.GetMessageString());
    }
    r.ok = true;
    return r;
}

}  // namespace psom
