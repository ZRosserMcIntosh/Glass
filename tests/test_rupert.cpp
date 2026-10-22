// Glass Dynamics — Prince Rupert's drop.
#include <cmath>
#include <iostream>

#include "gls/rupert.hpp"
#include "gls_test.hpp"

using namespace gls;

namespace {
const Material& sodaLime() { return *glstest::library().material("soda_lime_float"); }

DropStrike strikeOf(const char* id, double speed = -1) {
    const ImpactorDef& d = *glstest::library().impactor(id);
    DropStrike s;
    s.mass = d.mass;
    s.radius = d.radius;
    s.speed = speed > 0 ? speed : d.speed;
    s.density = d.density;
    s.yield = d.yield;
    s.youngs = d.youngs;
    s.poisson = d.poisson;
    s.rigid = d.kind != ImpactorKind::Projectile;
    return s;
}
}  // namespace

TEST_CASE("rupert: the residual stress balances, a thin skin in high compression round a core in tension") {
    RupertDrop d(DropShape{}, sodaLime());
    double sum = 0;
    constexpr int n = 4000;
    for (int i = 0; i < n; ++i) {
        const double u = (i + 0.5) / n;
        sum += d.residual(u) * u / n;
    }
    CHECK(std::fabs(sum) < 1e5);  // Pa: the axial force over the section is zero
    CHECK_NEAR(d.residual(1), -500e6, 1);
    CHECK_NEAR(d.residual(0), 100e6, 1);
    CHECK_RANGE(d.compressionDepth(), 0.1, 0.25);
    CHECK_RANGE(d.fragmentSize(), 0.5e-3, 3e-3);   // the powder it bursts into
    CHECK_RANGE(1e3 * d.skinDepth(), 0.5, 1.2);
}

TEST_CASE("rupert: snap the tail and a front runs back to the head at the speed of a crack") {
    RupertDrop d(DropShape{}, sodaLime(), 3);
    const double cut = 0.05;
    d.snip(cut);
    CHECK(d.broken());
    const double reach = cut / d.frontSpeed();
    CHECK_RANGE(d.frontSpeed(), 1400, 1950);  // measured on drops: 1.45-1.9 km/s
    d.advance(0.5 * reach);
    CHECK_NEAR(d.brokenFrom(), 0.5 * cut, 1e-9);
    CHECK_NEAR(d.brokenTo(), cut, 1e-12);
    d.advance(2 * reach);
    CHECK_NEAR(d.brokenFrom(), 0, 1e-12);
    int glass = 0;
    double vmax = 0, last = 0;
    for (const auto& f : d.fragments()) {
        if (f.lead) continue;
        ++glass;
        vmax = std::max(vmax, length(f.vel));
        last = std::max(last, f.born);
    }
    std::cout << "    " << glass << " fragments, the head last at " << 1e6 * last << " us, fastest " << vmax << " m/s\n";
    CHECK(glass > 300);
    CHECK_NEAR(last, reach, 2 * d.fragmentSize() / d.frontSpeed());
    CHECK_RANGE(vmax, 3, 40);
}

TEST_CASE("rupert: a lead bullet splashes off the head; a rifle round or a full hammer blow breaks it") {
    {
        RupertDrop d(DropShape{}, sodaLime());
        CHECK(!d.strike(strikeOf("lead_38")));
        CHECK(d.whole());
        int lead = 0;
        for (const auto& f : d.fragments()) lead += f.lead;
        CHECK(lead > 20);
        CHECK(d.outcome().find("splashed") != std::string::npos);
        std::cout << "    .38: " << d.outcome() << "\n";
    }
    {
        RupertDrop d(DropShape{}, sodaLime());
        CHECK(!d.strike(strikeOf("pistol_9mm")));
        CHECK(!d.strike(strikeOf("lead_22")));
    }
    {
        RupertDrop d(DropShape{}, sodaLime());
        CHECK(d.strike(strikeOf("rifle_762")));
        CHECK(d.broken());
        std::cout << "    7.62: " << d.outcome() << "\n";
    }
    {
        // Measured: the head takes about 15 kN. A tap survives; a full swing does not.
        RupertDrop tap(DropShape{}, sodaLime()), swing(DropShape{}, sodaLime());
        CHECK(!tap.strike(strikeOf("hammer", 1)));
        CHECK_RANGE(tap.needed(), 10e3, 30e3);
        CHECK(swing.strike(strikeOf("hammer", 10)));
        std::cout << "    hammer at 1 m/s: " << tap.outcome() << "\n";
    }
}

TEST_CASE("rupert: the same drop breaks the same way") {
    RupertDrop a(DropShape{}, sodaLime(), 9), b(DropShape{}, sodaLime(), 9);
    a.snip(0.04);
    b.snip(0.04);
    CHECK(a.fragments().size() == b.fragments().size());
    bool same = true;
    for (std::size_t i = 0; i < a.fragments().size(); ++i)
        same = same && a.fragments()[i].pos.x == b.fragments()[i].pos.x && a.fragments()[i].vel.y == b.fragments()[i].vel.y;
    CHECK(same);
}
