// Glass Dynamics — crack networks, fragments, and the fracture scenarios
// checked against the test standards they come from.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>

#include "gls/bridge.hpp"
#include "gls/cracks.hpp"
#include "gls/fragments.hpp"
#include "gls_test.hpp"

using namespace gls;
using glstest::shoot;

// ---- Crack network mechanics ----------------------------------------------------

TEST_CASE("cracks: a running crack stops where it meets another") {
    CrackNetwork net;
    net.configure(1.0, 1.0, 0.002);
    const int a = net.spawn({0.5, 0.0005}, {0, 1}, 0, CrackKind::Radial, net.newGroup(), 0, 1500);
    for (int i = 0; i < 2000 && net.tips()[a].alive; ++i) net.advance(a, net.tips()[a].pos + Vec2{0, 0.001}, i * 1e-6);
    CHECK(!net.tips()[a].alive);  // reached the top edge
    const int b = net.spawn({0.1, 0.5}, {1, 0}, 0, CrackKind::Radial, net.newGroup(), 0, 1500);
    for (int i = 0; i < 2000 && net.tips()[b].alive; ++i) net.advance(b, net.tips()[b].pos + Vec2{0.001, 0}, i * 1e-6);
    CHECK(!net.tips()[b].alive);
    CHECK_NEAR(net.tips()[b].pos.x, 0.5, 0.004);  // stopped at the first crack, not the far edge
}

TEST_CASE("cracks: vertex times run forward along every crack") {
    const auto s = shoot("annealed_6", "steel_ball_2260", 8.86);
    for (const auto& G : s.pane->glass())
        for (const auto& sg : G.net.segments()) CHECK(sg.tb >= sg.ta);
}

TEST_CASE("fragments: two crossing cracks cut a pane into four pieces") {
    CrackNetwork net;
    net.configure(0.4, 0.4, 0.002);
    auto run = [&](Vec2 p, Vec2 d) {
        const int k = net.spawn(p, d, 0, CrackKind::Radial, net.newGroup(), 0, 1500);
        for (int i = 0; i < 1000 && net.tips()[k].alive; ++i) net.advance(k, net.tips()[k].pos + d * 0.001, 0);
    };
    run({0.2, 0.0005}, {0, 1});
    run({0.0005, 0.2}, {1, 0});
    run({0.3999, 0.2}, {-1, 0});
    FragmentSystem fs;
    CHECK(fs.build(net, 0, 0.003, 0.006, 2500, 0) == 4);
    double area = 0;
    for (const auto& f : fs.fragments()) area += f.area;
    CHECK_NEAR(area, 0.16, 0.16 * 0.02);
    for (const auto& f : fs.fragments()) CHECK(f.edges != 0);
}

TEST_CASE("fragments: a hard landing breaks a shard; pieces come to rest on the floor") {
    CrackNetwork net;
    net.configure(0.2, 0.2, 0.002);
    FragmentSystem fs;
    fs.build(net, 0, 0.003, 0.006, 2500, 0);
    CHECK(fs.fragments().size() == 1);
    Fragment& f = fs.fragments()[0];
    f.state = FragmentState::Free;
    f.pos = {0.1, 1.5, 0};
    f.omega = {3, 1, 0};
    fs.floorY = 0;
    Rng rng(3);
    for (int i = 0; i < 6000; ++i) fs.step(0.5e-3, i * 0.5e-3, rng);
    CHECK(fs.fragments().size() > 1);  // 0.2 m shard from 1.5 m does not survive
    for (const auto& p : fs.fragments()) {
        if (p.retired) continue;
        CHECK(p.state == FragmentState::Resting);
        CHECK_RANGE(p.pos.y, -1e-3, 0.25);
    }
}

// ---- Determinism ---------------------------------------------------------------------

TEST_CASE("fracture: the same seed gives the same break; another seed a different one") {
    const auto a = shoot("annealed_6", "steel_ball_2260", 8.86, 2, 7);
    const auto b = shoot("annealed_6", "steel_ball_2260", 8.86, 2, 7);
    const auto c = shoot("annealed_6", "steel_ball_2260", 8.86, 2, 8);
    CHECK(a.segments == b.segments);
    CHECK(a.fragments == b.fragments);
    CHECK(a.report.speedOut == b.report.speedOut);
    CHECK(a.segments != c.segments || a.fragments != c.fragments);
}

TEST_CASE("fracture: results do not depend on how the host chops its frames") {
    const auto& lib = glstest::library();
    auto run = [&](double frame) {
        Pane p(*lib.glazing("annealed_4"), budgetForTier(1), 5);
        p.fire(*lib.impactor("baseball"), {0.45, 0.4}, 40, 0, 0, false, 0.05);
        while (p.time() < 0.05) p.advance(p.time() + frame);
        int segs = 0;
        for (const auto& G : p.glass()) segs += static_cast<int>(G.net.segments().size());
        return segs;
    };
    CHECK(run(1.0 / 60) == run(1.0 / 997));
}

// ---- Monolithic glass --------------------------------------------------------------

TEST_CASE("annealed: a 2.26 kg ball from 4 m goes through 6 mm plate glass") {
    const auto s = shoot("annealed_6", "steel_ball_2260", 8.86);
    CHECK(s.report.broke);
    CHECK(s.report.penetrated);
    CHECK(s.fragments >= 3);
    CHECK_RANGE(s.report.speedOut, 4, 8.86);
}

TEST_CASE("annealed: a gentle tap does nothing") {
    const auto s = shoot("annealed_6", "steel_ball_227", 0.8);
    CHECK(!s.report.broke);
    CHECK(s.report.rebounded);
    CHECK(s.segments == 0);
}

TEST_CASE("annealed: a steel BB leaves a Hertzian cone hole") {
    const auto s = shoot("annealed_6", "bb", 120);
    CHECK(s.report.penetrated);
    CHECK(s.report.outcome.find("cone") != std::string::npos);
}

TEST_CASE("annealed: small objects through the glass leave the pane in its frame") {
    // A BB or a bullet holes the pane; the pane around the hole stays put
    // (it used to be carried off with the projectile, or shaken loose).
    for (const char* im : {"bb", "pistol_9mm", "magnum_44", "rifle_762"}) {
        auto s = shoot("annealed_6", im, -1, 2, 1, false, {-1, -1}, 0.3);
        double heldArea = 0;
        int bigFree = 0;
        for (const auto& f : s.pane->fragments().fragments()) {
            if (f.chip || f.retired) continue;
            if (f.state == FragmentState::Held) heldArea += f.area;
            else if (f.radius > 0.05) ++bigFree;
        }
        const GlazingDef& g = s.pane->glazing();
        std::cout << "    " << im << ": " << 100 * heldArea / (g.width * g.height) << " % of the pane still in the frame\n";
        CHECK(s.report.penetrated || std::string(im) == "bb");
        CHECK(heldArea > 0.95 * g.width * g.height);
        CHECK(bigFree == 0);
    }
}

TEST_CASE("buckshot: nine pellets, through float glass, stopped by armour") {
    const auto& lib = glstest::library();
    const ImpactorDef& d = *lib.impactor("buckshot_00");
    CHECK(d.pellets == 9);
    for (const char* glazing : {"annealed_6", "armoured_windscreen"}) {
        const GlazingDef& g = *lib.glazing(glazing);
        Pane p(g, budgetForTier(1), 1);
        const int first = p.fire(d, {0.5 * g.width, 0.5 * g.height}, d.speed, 0, 0, false, 0.85);
        p.advance(0.05);
        CHECK(first == 0);
        CHECK(p.impactors().size() == 9u);
        int through = 0, hit = 0;
        double spreadX = 0;
        for (const auto& I : p.impactors()) {
            if (!I.touched) continue;
            ++hit;
            through += I.report.penetrated ? 1 : 0;
            spreadX = std::max(spreadX, std::fabs(I.hit.x - 0.5 * g.width));
        }
        std::cout << "    " << glazing << ": " << hit << " pellets on the glass, " << through << " through, widest "
                  << 100 * spreadX << " cm from the aim\n";
        CHECK(hit >= 7);
        CHECK_RANGE(spreadX, 0.02, 0.25);
        if (std::string(glazing) == "annealed_6") CHECK(through == hit);
        else CHECK(through == 0);
    }
}

TEST_CASE("heat-strengthened and tempered glass take more than annealed") {
    // Raise the speed of a 227 g ball until each breaks.
    auto threshold = [](const char* g) {
        for (double v = 1; v < 40; v *= 1.15)
            if (shoot(g, "steel_ball_227", v, 1, 2).report.broke) return v;
        return 40.0;
    };
    const double a = threshold("annealed_6"), h = threshold("heat_strengthened_6"), t = threshold("tempered_6");
    std::cout << "    227 g ball breaking speed: annealed " << a << ", heat-strengthened " << h << ", tempered " << t
              << " m/s\n";
    // Under a microsecond load the residual compression adds a fixed amount
    // to a rate-raised intrinsic strength, so the gain is smaller than the 2x
    // and 4x of design (long-term) strengths, but tempered clearly wins.
    CHECK(h > a);
    CHECK(t >= h);
    CHECK(t > 1.4 * a);
}

TEST_CASE("ECE R43: tempered side glass survives the 227 g ball from 2 m") {
    for (std::uint64_t seed : {1, 2, 3}) CHECK(!shoot("car_side_tempered", "steel_ball_227", 6.26, 2, seed).report.broke);
}

TEST_CASE("tempered: a window punch dices the pane, and the dice meet ECE R43") {
    const auto s = shoot("car_side_tempered", "window_punch");
    CHECK(s.report.broke);
    CHECK(s.report.diced);
    CHECK(s.fragments > 200);
    // The energy balance gives 40-400 fragments in a 50 x 50 mm square (R43 7.3.2).
    const GlassPly& G = s.pane->glass()[0];
    const double perSquare = 0.0025 / (G.diceSize * G.diceSize);
    std::cout << "    energy-balance dice " << G.diceSize * 1e3 << " mm, " << perSquare << " per 50 mm square\n";
    CHECK_RANGE(perSquare, 40, 400);
    // The pieces fall out from the break outward; a fringe stays in the frame.
    int held = 0, gone = 0;
    for (const auto& f : s.pane->fragments().fragments()) {
        if (f.chip) continue;
        // A piece that came apart into dice (retired) has gone too.
        (f.retired || f.state != FragmentState::Held ? gone : held)++;
    }
    CHECK(gone > held);
}

namespace {
struct Collapse {
    int remaining = 0, size = 0, heldAbove = 0, above = 0, heldBelow = 0, below = 0, farFromFrame = 0;
    std::size_t sprays = 0;
};
Collapse collapseAt(const Pane& p) {
    Collapse c;
    if (p.sheets().empty()) return c;
    c.remaining = p.sheets()[0].remaining();
    c.size = static_cast<int>(p.sheets()[0].size());
    c.sprays = p.fragments().sprays().size();
    const double w = p.glazing().width, h = p.glazing().height;
    for (const auto& f : p.fragments().fragments()) {
        if (f.retired || f.chip || f.diceSize <= 0) continue;
        const bool held = f.state == FragmentState::Held, up = f.origin.y > 0.5 * h;
        (up ? c.above : c.below)++;
        if (held) (up ? c.heldAbove : c.heldBelow)++;
        if (held && std::min({f.origin.x, w - f.origin.x, f.origin.y, h - f.origin.y}) > 0.06) ++c.farFromFrame;
    }
    return c;
}
}  // namespace

TEST_CASE("tempered: a holed pane comes down from the hole up; a fringe stays in the frame") {
    for (const char* glazing : {"tempered_6", "car_side_tempered"}) {
        auto s = shoot(glazing, "window_punch", -1, 2, 1, false, {-1, -1}, 0.1);
        const Collapse early = collapseAt(*s.pane);
        CHECK(early.size > 1000);
        // Above the hole nothing bears it up: the top comes down first.
        CHECK(static_cast<double>(early.heldAbove) / early.above < 0.5 * early.heldBelow / early.below);
        s.pane->advance(2.0);
        const Collapse late = collapseAt(*s.pane);
        std::cout << "    " << glazing << ": " << late.remaining << " of " << late.size << " dice left in the frame, "
                  << late.sprays << " pieces came apart into dice\n";
        CHECK_RANGE(static_cast<double>(late.remaining) / late.size, 0.02, 0.15);
        CHECK(late.farFromFrame == 0);
        // Pieces drawn coarser than the dice come apart into them, in the air or on landing.
        CHECK(late.sprays > static_cast<std::size_t>(late.size / 2));
    }
}

TEST_CASE("tempered: over the piece budget, no part of the pane goes missing") {
    // A brick on high quality crazes into more regions than the tier's piece
    // budget: they merge into coarser pieces rather than leave glass out.
    auto s = shoot("tempered_6", "brick", -1, 3, 1, false, {-1, -1}, 0.02);
    const GlazingDef& g = s.pane->glazing();
    double area = 0, top = 0;
    for (const auto& f : s.pane->fragments().fragments()) {
        if (f.chip) continue;
        area += f.area;
        top = std::max(top, f.origin.y);
    }
    std::cout << "    pieces cover " << 100 * area / (g.width * g.height) << " % of the pane\n";
    CHECK(area > 0.85 * g.width * g.height);
    CHECK(top > 0.9 * g.height);
    CHECK(static_cast<int>(s.pane->fragments().fragments().size()) <= s.pane->budget().maxFragments);
}

TEST_CASE("tempered: the collapse is the same for the same seed, different for another") {
    auto run = [](std::uint64_t seed) {
        auto s = shoot("tempered_6", "hammer", -1, 1, seed, false, {-1, -1}, 0.05);
        std::vector<int> left;
        for (const double t : {0.15, 0.3, 0.6}) {
            s.pane->advance(t);
            left.push_back(collapseAt(*s.pane).remaining);
        }
        return left;
    };
    const auto a = run(7), b = run(7), c = run(8);
    CHECK(a == b);
    CHECK(a != c);
}

TEST_CASE("tempered: the drawn dice match the energy balance at full resolution") {
    const auto s = shoot("tempered_6", "window_punch", -1, 4, 1, false, {-1, -1}, 0.01);
    const GlassPly& G = s.pane->glass()[0];
    const int n = s.pane->fragmentsIn(0, {0.45 + 0.08, 0.6 + 0.08}, 0.05);
    const double expect = 0.0025 / (G.diceSpacing * G.diceSpacing);
    std::cout << "    50 mm square: " << n << " drawn, " << expect << " from the drawn spacing, real dice "
              << G.diceSize * 1e3 << " mm\n";
    CHECK_RANGE(n, 0.3 * expect, 3 * expect);
}

// 16 CFR 1201 / ANSI Z97.1: safety glazing may break under the 45 kg bag from
// 48 in only into small pieces: the ten largest weigh no more than 10 in^2 of
// the glass. Annealed glass fails it with big shards.
static double tenLargest(const glstest::Shot& s) {
    std::vector<double> a;
    for (const auto& f : s.pane->fragments().fragments())
        if (!f.retired && !f.chip && f.state != FragmentState::Held) a.push_back(f.area);
    std::sort(a.rbegin(), a.rend());
    double sum = 0;
    for (std::size_t i = 0; i < a.size() && i < 10; ++i) sum += a[i];
    return sum;
}

TEST_CASE("ANSI Z97.1: under the 45 kg bag annealed glass fails dangerously, tempered safely") {
    const double limit = 10 * 0.0254 * 0.0254;
    const auto a = shoot("annealed_6", "body_bag", 4.89, 2, 1, false, {-1, -1}, 1.0);
    CHECK(a.report.broke);
    CHECK(tenLargest(a) > limit);
    for (std::uint64_t seed : {1, 2}) {
        const auto t = shoot("tempered_6", "body_bag", 4.89, 2, seed, false, {-1, -1}, 1.0);
        // Lower tiers draw coarser dice than the real glass; judge the real
        // (energy-balance) dice, and that it did dice rather than shard.
        const GlassPly& G = t.pane->glass()[0];
        if (t.report.broke) {
            CHECK(t.report.diced || G.dicing);
            CHECK(10 * G.diceSize * G.diceSize <= limit);
            CHECK(tenLargest(t) * sq(G.diceSize / G.diceSpacing) <= limit);
        }
        std::cout << "    tempered seed " << seed << (t.report.broke ? ": broke safely" : ": held") << "\n";
    }
}

// ---- Laminates --------------------------------------------------------------------

TEST_CASE("ECE R43: the windscreen catches the 2.26 kg ball from 4 m") {
    for (std::uint64_t seed : {1, 2}) {
        const auto s = shoot("windscreen", "steel_ball_2260", 8.86, 2, seed);
        CHECK(s.report.broke);
        CHECK(!s.report.penetrated);
        CHECK_RANGE(s.report.bulge, 0.02, 0.15);
    }
}

TEST_CASE("windscreen: a much harder hit tears the interlayer") {
    const auto s = shoot("windscreen", "steel_ball_2260", 20);
    CHECK(s.report.penetrated);
}

TEST_CASE("windscreen: a road stone stars the outer ply only") {
    const auto s = shoot("windscreen", "stone", 30);
    CHECK(s.report.broke);
    CHECK(!s.report.penetrated);
    CHECK(!s.pane->glass()[0].net.segments().empty());
    CHECK(s.pane->glass()[1].net.segments().empty());
}

TEST_CASE("windscreen: a window punch cannot get through laminated glass") {
    const auto s = shoot("windscreen", "window_punch");
    CHECK(!s.report.penetrated);
    CHECK(s.pane->status().barrierIntact);
}

TEST_CASE("windscreen: an occupant's head from inside is held") {
    const auto s = shoot("windscreen", "headform", 7, 2, 1, true);
    CHECK(!s.report.penetrated);
}

TEST_CASE("ASTM E1996: the hurricane laminate keeps the 2x4 out") {
    const auto s = shoot("hurricane", "timber_2x4", 15.2);
    CHECK(!s.report.penetrated);
}

TEST_CASE("14 CFR 25.775: the aircraft windshield stops a 4 lb bird") {
    const auto s = shoot("aircraft_windshield", "bird_4lb", 128.6);
    CHECK(!s.report.penetrated);
    CHECK(s.pane->status().barrierIntact);
}

// ---- Ballistics (UL 752 classes) -----------------------------------------------------

TEST_CASE("UL 752: 9 mm is stopped by the Level 1 laminate") {
    const auto s = shoot("ul752_l1", "pistol_9mm");
    CHECK(s.report.embedded);
    CHECK(!s.report.penetrated);
}

TEST_CASE("UL 752: .44 Magnum is stopped by the Level 3 glass-clad polycarbonate") {
    const auto s = shoot("ul752_l3", "magnum_44");
    CHECK(s.report.embedded);
}

TEST_CASE("UL 752: a 7.62 mm rifle round defeats Level 3 but not Level 8") {
    CHECK(shoot("ul752_l3", "rifle_762").report.penetrated);
    const auto s = shoot("ul752_l8", "rifle_762");
    CHECK(s.report.embedded);
    CHECK(s.report.depth > 0.02);  // well into the build-up
}

TEST_CASE("ballistics: ordinary glass and windscreens do not stop a 9 mm") {
    const auto a = shoot("annealed_6", "pistol_9mm");
    CHECK(a.report.penetrated);
    CHECK_RANGE(a.report.speedOut, 200, 380);
    CHECK(shoot("windscreen", "pistol_9mm").report.penetrated);
}

TEST_CASE("ballistics: three shots on one Level 3 panel") {
    const auto& lib = glstest::library();
    Pane p(*lib.glazing("ul752_l3"), budgetForTier(1), 3);
    const ImpactorDef& d = *lib.impactor("magnum_44");
    for (const Vec2 at : {Vec2{0.25, 0.3}, Vec2{0.35, 0.3}, Vec2{0.3, 0.38}}) {
        p.fire(d, at, d.speed, 0, 0, false, 0.05);
        p.advance(p.time() + 0.02);
    }
    for (const auto& I : p.impactors()) CHECK(I.report.embedded);
}

// ---- Tiers ----------------------------------------------------------------------------

TEST_CASE("tiers: every tier gives the same outcome for the same shot") {
    for (int tier = 0; tier < kTierCount; ++tier) {
        const auto s = shoot("annealed_6", "steel_ball_2260", 8.86, tier);
        CHECK(s.report.penetrated);
        const auto w = shoot("windscreen", "steel_ball_2260", 8.86, tier);
        CHECK(!w.report.penetrated);
    }
}

// ---- Armoured glazing under fire --------------------------------------------------------

namespace {
// One 9 mm round into a pane at `at`, travelling `pitch` radians off square
// (up the glass), run until it is done.
const ImpactReport& smgRound(Pane& p, Vec2 at, double pitch, double t) {
    const auto& lib = glstest::library();
    const ImpactorDef& b = *lib.impactor("smg_9mm");
    p.advance(t);
    const int k = p.fire(b, at, b.speed, 0, pitch, false, 0.3);
    p.advance(t + 0.01);
    return p.impactors()[k].report;
}
}  // namespace

TEST_CASE("ballistics: an oblique round meets the armour's slant thickness, a grazing one glances off") {
    const auto& lib = glstest::library();
    const GlazingDef& g = *lib.glazing("armoured_windscreen");
    Pane square(g, budgetForTier(1), 1), raked(g, budgetForTier(1), 1), grazing(g, budgetForTier(1), 1);
    const Vec2 c{0.5 * g.width, 0.5 * g.height};
    const ImpactReport a = smgRound(square, c, 0, 0.01);
    const ImpactReport b = smgRound(raked, c, deg(55), 0.01);
    const ImpactReport r = smgRound(grazing, c, deg(78), 0.01);
    std::cout << "    square " << a.depth * 1e3 << " mm deep, at 55 degrees " << b.depth * 1e3 << " mm\n";
    CHECK(a.embedded && b.embedded);
    // The same path through glass reaches less far through the pane.
    CHECK(b.depth < 0.8 * a.depth);
    CHECK(r.rebounded && !r.embedded && !r.penetrated);
    CHECK(r.outcome.find("Ricochet") != std::string::npos);
    CHECK(!grazing.crushes().empty());
}

TEST_CASE("ballistics: a round into an earlier crater goes deeper") {
    const auto& lib = glstest::library();
    Pane p(*lib.glazing("armoured_windscreen"), budgetForTier(1), 2);
    const Vec2 c{0.6, 0.2};
    const double d1 = smgRound(p, c, 0, 0.01).depth;
    const double d2 = smgRound(p, c, 0, 0.05).depth;
    const double d3 = smgRound(p, {0.95, 0.2}, 0, 0.09).depth;  // fresh glass again
    std::cout << "    first " << d1 * 1e3 << " mm, into its crater " << d2 * 1e3 << " mm, fresh glass " << d3 * 1e3 << " mm\n";
    CHECK(d2 > 1.3 * d1);
    CHECK(d3 < 1.2 * d1);
}

TEST_CASE("armoured windscreen: a submachine-gun barrage does not get through, but the driver's view goes") {
    const auto& lib = glstest::library();
    const GlazingDef& g = *lib.glazing("armoured_windscreen");
    Pane p(g, budgetForTier(1), 3);
    Rng rng(42);
    const int rounds = glstest::kOptimisedBuild ? 80 : 16;
    int through = 0, hits = 0;
    double t = 0.02;
    for (int k = 0; k < rounds; ++k) {
        t += (k % 6 == 5) ? 0.3 : 0.075;
        const Vec2 at{0.4 * g.width + 0.15 * rng.normal(), 0.5 * g.height + 0.08 * rng.normal()};
        if (at.x < 0.02 || at.y < 0.02 || at.x > g.width - 0.02 || at.y > g.height - 0.02) continue;
        const ImpactReport& r = smgRound(p, at, deg(50) + rng.uniform(-0.1, 0.1), t);
        ++hits;
        through += r.penetrated ? 1 : 0;
    }
    const PaneStatus st = p.status();
    std::cout << "    " << hits << " rounds on the glass, " << through << " through, visibility " << st.visibility << "\n";
    CHECK(through == 0);
    CHECK(st.barrierIntact);
    CHECK(static_cast<int>(p.crushes().size()) >= hits);
    CHECK(st.visibility < (glstest::kOptimisedBuild ? 0.85 : 0.97));
}

// ---- Host coupling ------------------------------------------------------------------

TEST_CASE("bridge: a windscreen's inward normal points into the car") {
    const PaneMount m = vehicle::windscreen({1.2, 0, 1.0}, 1.4, 0.8, 62);
    CHECK(m.z.x < -0.4);  // rearward
    CHECK(m.z.z < 0);     // and down, it is raked
    CHECK_NEAR(dot(m.x, m.y), 0, 1e-12);
    const Vec3 p = m.toPane(m.toHost({0.3, 0.2, 0.001}));
    CHECK_NEAR(p.x, 0.3, 1e-12);
    CHECK_NEAR(p.y, 0.2, 1e-12);
}

TEST_CASE("bridge: a stone from ahead strikes the outer face of the windscreen") {
    const auto& lib = glstest::library();
    const PaneMount m = vehicle::windscreen({1.2, 0, 1.0}, 1.45, 0.85, 60);
    Pane pane(*lib.glazing("windscreen"), budgetForTier(1), 1);
    const Vec3 hit = m.toHost({0.7, 0.4, 0});
    const int k = strike(pane, m, *lib.impactor("stone"), hit, {-30, 0, 0});  // car at 30 m/s into a still stone
    CHECK(k == 0);
    CHECK(pane.impactors()[0].side == 1);
    pane.advance(0.02);
    CHECK(pane.impactors()[0].touched);
    CHECK_NEAR(pane.impactors()[0].hit.x, 0.7, 0.01);
}

TEST_CASE("bridge: side windows face out on both doors") {
    const PaneMount l = vehicle::sideWindow({0.3, 0.8, 1.0}, 0.75, 0.45, true, 15);
    const PaneMount r = vehicle::sideWindow({0.3, -0.8, 1.0}, 0.75, 0.45, false, 15);
    CHECK(l.z.y < -0.9);
    CHECK(r.z.y > 0.9);
}

TEST_CASE("bridge: a bird ahead of an aircraft hits the windshield's outer face") {
    const PaneMount m = aircraft::windshield({4.0, 0.4, -1.2}, 0.85, 0.6, 55, 20);
    CHECK(m.z.x < 0);  // aft, into the cockpit
    const Vec3 v = aircraft::relativeAirVelocity(128.6, deg(3), 0);
    CHECK(v.x < -128);
    CHECK(m.toPaneDirection(v).z > 0);  // travelling into the glass from outside
}
