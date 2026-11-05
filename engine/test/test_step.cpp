#include <gtest/gtest.h>

#include "psom/step.h"
#include "util.h"

TEST(Step, ClosedMeshBecomesAValidSolid) {
    const psom::Mesh m = cube(0.05);
    const fs::path out = test_root() / "cube.step";
    psom::StepInfo info;
    const psom::Result r = psom::export_step(m, 1000, psom::validate(m), out, info);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_TRUE(info.solid);
    EXPECT_EQ(info.faces, 12u);
    EXPECT_FALSE(fs::exists(out.string() + ".tmp"));
    const psom::StepCheck c = psom::check_step(out);
    EXPECT_TRUE(c.read);
    EXPECT_TRUE(c.valid);
    EXPECT_EQ(c.solids, 1);
    EXPECT_NEAR(c.volume_mm3, 125000, 1250);
    EXPECT_NEAR(c.bmax[0] - c.bmin[0], 50, 0.01);
    EXPECT_NE(slurp(out).find("psom Mode A"), std::string::npos);  // the writer wraps long header lines
}

TEST(Step, OpenMeshBecomesAnOpenShellWithAWarning) {
    const psom::Mesh m = cube(0.05, {0, 0, 0}, {"top"});
    const fs::path out = test_root() / "open.step";
    psom::StepInfo info;
    const psom::Result r = psom::export_step(m, 1000, psom::validate(m), out, info);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_FALSE(info.solid);
    ASSERT_FALSE(r.warnings.empty());
    EXPECT_NE(r.warnings[0].find("OPEN SHELL"), std::string::npos);
    const psom::StepCheck c = psom::check_step(out);
    EXPECT_EQ(c.solids, 0);
    EXPECT_GE(c.shells, 1);
    EXPECT_EQ(c.faces, 10u);
    EXPECT_NE(slurp(out).find("OPEN SHELL"), std::string::npos);
}

TEST(Step, NeverASolidWithoutAValidatedClosedShell) {
    const psom::Mesh m = cube(0.05);
    psom::Topology t = psom::validate(m);
    t.closed_valid = false;  // whatever the B-rep says, the mesh check has the last word
    t.reasons = {"pretend"};
    psom::StepInfo info;
    const psom::Result r = psom::export_step(m, 1000, t, test_root() / "pretend.step", info);
    ASSERT_TRUE(r.ok);
    EXPECT_FALSE(info.solid);
}

TEST(Step, EmptyMeshFails) {
    psom::StepInfo info;
    const fs::path out = test_root() / "empty.step";
    EXPECT_FALSE(psom::export_step(psom::Mesh{}, 1000, psom::Topology{}, out, info).ok);
    EXPECT_FALSE(fs::exists(out));
}
