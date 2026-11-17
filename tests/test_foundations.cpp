// Glass Dynamics — data, residual stress, laminate and plate foundations.
// SPDX-License-Identifier: MIT
#include <cmath>

#include "gls/contact.hpp"
#include "gls/plate.hpp"
#include "gls_test.hpp"

using namespace gls;

TEST_CASE("data: every file loads without errors and every field is known") {
    Library l;
    for (const char* f : {"/materials.json", "/glazings.json", "/impactors.json"}) {
        std::ifstream in(std::string(GLS_DATA_DIR) + f, std::ios::binary);
        std::stringstream ss;
        ss << in.rdbuf();
        CHECK(l.load(ss.str()).empty());
    }
    CHECK(l.resolve().empty());
    CHECK(l.materials.size() >= 6);
    CHECK(l.glazings.size() >= 12);
    CHECK(l.impactors.size() >= 17);
}

TEST_CASE("data: provenance on every material, starting with its class") {
    for (const auto& m : glstest::library().materials) {
        CHECK(!m.provenance.empty());
        for (const auto& [k, v] : m.provenance)
            CHECK(v.rfind("published", 0) == 0 || v.rfind("calibrated", 0) == 0 || v.rfind("estimated", 0) == 0);
    }
}

TEST_CASE("data: the loader rejects unknown fields and bad values") {
    Library l;
    CHECK(!l.load(R"({"materials":[{"id":"x","kind":"glass","youngs_GPa":72,"colour":"green"}]})").empty());
    CHECK(!l.load(R"({"materials":[{"id":"y","kind":"glass","poisson":0.7}]})").empty());
    CHECK(!l.load(R"({"glazings":[{"id":"z","plies":[]}]})").empty());
    CHECK(!l.load("{ not json").empty());
}

TEST_CASE("residual stress: thermal tempering is parabolic and self-balanced") {
    Ply p;
    p.thickness = 0.006;
    p.treatment = Treatment::FullyTempered;
    p.surfaceCompression = 100e6;
    CHECK_NEAR(p.centralTension(), 50e6, 1);
    CHECK_NEAR(p.residualAt(0), -100e6, 1);
    CHECK_NEAR(p.residualAt(0.003), 50e6, 1);
    CHECK_NEAR(p.compressionDepth() / p.thickness, 0.2113, 1e-3);
    CHECK_NEAR(p.residualAt(p.compressionDepth()), 0, 1e3);
    // Force balance through the thickness.
    double sum = 0;
    for (int i = 0; i < 1000; ++i) sum += p.residualAt((i + 0.5) * p.thickness / 1000);
    CHECK_NEAR(sum / 1000, 0, 2e5);
}

TEST_CASE("residual stress: ion exchange balances a thin layer with the core") {
    Ply p;
    p.thickness = 0.008;
    p.treatment = Treatment::Chemical;
    p.surfaceCompression = 350e6;
    p.layerDepth = 350e-6;
    double sum = 0;
    for (int i = 0; i < 20000; ++i) sum += p.residualAt((i + 0.5) * p.thickness / 20000);
    CHECK_NEAR(sum / 20000, 0, 1e6);
    CHECK(p.centralTension() > 0 && p.centralTension() < 30e6);
}

TEST_CASE("laminate: shear coupling sits between the layered and monolithic limits") {
    const auto& lib = glstest::library();
    for (const auto& g : lib.glazings) {
        const Laminate L = makeLaminate(g);
        CHECK(L.gamma >= 0 && L.gamma <= 1);
        CHECK(L.D >= L.Dlayered * 0.999 && L.D <= L.Dmono * 1.001);
        CHECK_NEAR(L.thickness, g.thickness(), 1e-12);
    }
    // At impact rates PVB couples the windscreen plies almost fully.
    CHECK(makeLaminate(*lib.glazing("windscreen")).gamma > 0.9);
    // A soft PVB (long-term loads) would not.
    GlazingDef soft = *lib.glazing("windscreen");
    Material pvb = *lib.material("pvb");
    pvb.shear = 0.4e6;
    soft.plies[1].mat = &pvb;
    CHECK(makeLaminate(soft).gamma < 0.5);
}

TEST_CASE("plate: natural frequencies of a simply supported plate") {
    // omega_mn = pi^2 (m^2/a^2 + n^2/b^2) sqrt(D / rho h)
    ModalPlate p;
    const double D = 1364, rh = 15;
    p.configure(1.2, 1.0, D, rh, 0.22, 0.0, 8, 8);
    const double f11 = kPi * (1 / 1.44 + 1) * std::sqrt(D / rh) / 2;
    CHECK_NEAR(p.fundamentalHz(), f11, 1e-6 * f11);
}

TEST_CASE("plate: static centre deflection matches Navier (0.0116 F a^2 / D)") {
    ModalPlate p;
    p.configure(1.0, 1.0, 1000, 10, 0.3, 0.01, 60, 60);
    p.addPatch(0.5, 0.5, 1e-4, 1000);
    p.settleStatic();
    CHECK_NEAR(p.deflection(0.5, 0.5), 0.0116 * 1000 * 1.0 / 1000, 0.0116 * 0.01);
}

TEST_CASE("plate: uniform pressure deflection matches Navier (0.00406 p a^4 / D)") {
    ModalPlate p;
    p.configure(1.0, 1.0, 1000, 10, 0.3, 0.01, 31, 31);
    p.addPressure(1000);
    p.settleStatic();
    CHECK_NEAR(p.deflection(0.5, 0.5), 0.00406 * 1000 / 1000, 0.00406 * 0.005);
}

TEST_CASE("plate: near-field correction recovers the converged stress under a load") {
    for (int M : {12, 24, 40}) {
        ModalPlate ref, p;
        const double re = 0.004;
        ref.configure(1, 1, 1, 1, 0.22, 0.01, 600, 600);
        ref.addPatch(0.5, 0.5, re, 1);
        ref.settleStatic();
        p.configure(1, 1, 1, 1, 0.22, 0.01, M, M);
        p.addPatch(0.5, 0.5, re, 1);
        p.settleStatic();
        const double kref = ref.curvature(0.5, 0.5).kxx;
        const double k = p.curvature(0.5, 0.5).kxx + p.nearField(1, 0, re);
        CHECK_NEAR(k, kref, 0.04 * kref);
    }
}

TEST_CASE("plate: the exact modal step conserves energy without damping, any step size") {
    ModalPlate p;
    p.configure(1, 1, 1364, 15, 0.22, 0.0, 16, 16);
    p.addPatch(0.4, 0.6, 0.01, 500);
    for (int i = 0; i < 50; ++i) p.step(1e-5);
    p.clearForces();
    const double e0 = p.energy();
    for (int i = 0; i < 200; ++i) p.step(i % 2 ? 3e-4 : 7e-6);
    CHECK_NEAR(p.energy(), e0, 1e-9 * e0);
}

TEST_CASE("plate: a short pulse only bends the plate near the load (flexural reach)") {
    ModalPlate p;
    p.configure(1, 1, 1364, 15, 0.22, 0.01, 24, 24);
    const double nfLong = p.nearField(1000, 0, 0.002, 1.0);
    const double nfShort = p.nearField(1000, 0, 0.002, 5e-6);
    CHECK(nfShort > 0 && nfShort < 0.6 * nfLong);
}

TEST_CASE("contact: Hertz impact on a rigid wall matches the analytic contact time") {
    // t_c = 2.94 delta_max / v,  delta_max = (15 m v^2 / (16 E* sqrt R))^(2/5)
    const double m = 0.227, R = 0.019, v = 3, Es = contactModulus(210e9, 0.3, 72e9, 0.22);
    SphereContact c;
    c.configure(R, Es, 0, 1, v);
    double x = 0, vel = v, t = 0, dt = 1e-8;
    while (true) {
        const double F = c.force(x, vel);
        vel -= F / m * dt;
        x += vel * dt;
        t += dt;
        if (x < 0 && t > 1e-6) break;
    }
    const double dmax = std::pow(15 * m * v * v / (16 * Es * std::sqrt(R)), 0.4);
    CHECK_NEAR(t, 2.94 * dmax / v, 0.01 * 2.94 * dmax / v);
    CHECK_NEAR(-vel, v, 0.01 * v);  // elastic: no energy lost
}

TEST_CASE("contact: Hunt-Crossley damping gives the ball's coefficient of restitution") {
    const double m = 0.145, R = 0.0368, v = 20, e = 0.55, Es = contactModulus(0.06e9, 0.4, 72e9, 0.22);
    SphereContact c;
    c.configure(R, Es, 0, e, v);
    double x = 0, vel = v, t = 0, dt = 1e-7;
    while (t < 0.02) {
        const double F = c.force(x, vel);
        vel -= F / m * dt;
        x += vel * dt;
        t += dt;
        if (x < 0 && t > 1e-5) break;
    }
    CHECK_RANGE(-vel / v, e - 0.1, e + 0.1);
}

TEST_CASE("contact: Wilbeck soft body delivers the bird's momentum") {
    SoftBodyLoad s;
    s.density = 950;
    s.radius = 0.057;
    s.length = 0.228;
    s.normalSpeed = 128.6;
    const double m = 950 * kPi * 0.057 * 0.057 * 0.228;
    double J = 0;
    for (double t = 0; t < 0.004; t += 1e-7) J += s.force(t) * 1e-7;
    CHECK_NEAR(J, m * 128.6, 0.12 * m * 128.6);
    CHECK(s.hugoniotPressure() > 5 * 950 * 128.6 * 128.6);
}

TEST_CASE("contact: sharp-indentation median cracks grow as load^(2/3)") {
    const double c1 = medianCrackDepth(100, 72e9, 5.5e9, 0.75e6), c8 = medianCrackDepth(800, 72e9, 5.5e9, 0.75e6);
    CHECK_NEAR(c8 / c1, 4.0, 1e-9);
    CHECK_RANGE(medianCrackDepth(1000, 72e9, 5.5e9, 0.75e6), 0.5e-3, 3e-3);
}

TEST_CASE("contact: the interlayer membrane stiffens, then tears at its rim") {
    Membrane m;
    m.configure({{0.76e-3, 0.25e9, 20e6, 32e6, 2.0, false}}, 0.02, 0.2, 2.0, 1.0);
    const double f1 = m.force(0.01), f2 = m.force(0.03);
    CHECK(f1 > 0 && f2 > f1);
    CHECK(!m.torn());
    m.force(0.5);
    CHECK(m.torn());
    CHECK_NEAR(m.shape(0.01, 0.05), 0.05, 1e-12);
    CHECK_NEAR(m.shape(0.2, 0.05), 0.0, 1e-12);
}
