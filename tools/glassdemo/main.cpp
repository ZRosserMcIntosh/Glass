// Glass Dynamics — headless demos, the validation report and a benchmark.
// SPDX-License-Identifier: MIT
//
//   glassdemo list
//   glassdemo run <glazing> <impactor> [speed m/s] [x y] [--tier N] [--seed S] [--svg out.svg] [--back]
//   glassdemo validate
//   glassdemo bench
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "gls/bridge.hpp"
#include "gls/library.hpp"
#include "gls/pane.hpp"

using namespace gls;

namespace {

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool loadLibrary(Library& lib) {
    const std::string dir = GLS_DATA_DIR;
    bool ok = true;
    for (const char* f : {"/materials.json", "/glazings.json", "/impactors.json"}) {
        for (const auto& e : lib.load(readFile(dir + f))) {
            std::fprintf(stderr, "%s: %s\n", f, e.c_str());
            ok = false;
        }
    }
    for (const auto& e : lib.resolve()) {
        std::fprintf(stderr, "%s\n", e.c_str());
        ok = false;
    }
    return ok;
}

struct Outcome {
    ImpactReport report;
    PaneStatus status;
    int fragments = 0, crackSegments = 0;
    double wallMs = 0, simTime = 0;
    int steps = 0;
};

double gViewX = -1, gViewY = -1, gViewHalf = -1;  // optional zoom for the SVG

Outcome runScenario(const Library& lib, const std::string& glazingId, const std::string& impactorId, double speed,
                    Vec2 at, int tier, std::uint64_t seed, bool back, const char* svg, bool verbose, double until = 1.5) {
    Outcome o;
    const GlazingDef* g = lib.glazing(glazingId);
    const ImpactorDef* d = lib.impactor(impactorId);
    if (!g || !d) {
        std::fprintf(stderr, "unknown glazing or impactor\n");
        return o;
    }
    Pane pane(*g, budgetForTier(tier), seed);
    if (at.x < 0) at = {0.5 * g->width, 0.5 * g->height};
    pane.fire(*d, at, speed > 0 ? speed : d->speed, 0, 0, back);
    const auto t0 = std::chrono::steady_clock::now();
    for (int frame = 0; frame < 100000 && pane.time() < until; ++frame) {
        o.steps += pane.advance(pane.time() + 1.0 / 60.0);
        if (pane.quiet() && pane.time() > 0.05) break;
    }
    o.wallMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    o.simTime = pane.time();
    o.report = pane.impactors()[0].report;
    o.status = pane.status();
    for (const auto& f : pane.fragments().fragments())
        if (!f.retired && !f.chip) ++o.fragments;
    for (const auto& G : pane.glass()) o.crackSegments += static_cast<int>(G.net.segments().size());
    if (verbose) {
        for (const auto& e : pane.events()) std::printf("  %s\n", e.c_str());
    }
    if (svg) {
        std::FILE* f = std::fopen(svg, "w");
        if (f) {
            const double W = g->width, H = g->height, s = 800 / std::max(W, H);
            if (gViewHalf > 0)
                std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='800' height='800' viewBox='%.1f %.1f %.1f %.1f'>\n",
                             (gViewX - gViewHalf) * s, (H - gViewY - gViewHalf) * s, 2 * gViewHalf * s, 2 * gViewHalf * s);
            else
                std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='%.0f' height='%.0f' viewBox='-10 -10 %.0f %.0f'>\n",
                             W * s + 20, H * s + 20, W * s + 20, H * s + 20);
            std::fprintf(f, "<rect x='0' y='0' width='%.1f' height='%.1f' fill='#dfe9ee' stroke='#333'/>\n", W * s, H * s);
            // Fragments where they were born: held grey, fallen pale red.
            for (const auto& fr : pane.fragments().fragments()) {
                if (fr.retired || fr.chip) continue;
                std::fprintf(f, "<polygon fill='%s' stroke='none' points='",
                             fr.state == FragmentState::Held ? "#c9d6dc" : "#f3d2cc");
                for (const auto& p : fr.outline)
                    std::fprintf(f, "%.1f,%.1f ", (fr.origin.x + p.x) * s, (H - fr.origin.y - p.y) * s);
                std::fprintf(f, "'/>\n");
            }
            const char* colours[] = {"#111", "#1d57c2", "#c2401d", "#2a8a3a", "#8a2a8a", "#8a6a1a"};
            int gi = 0;
            for (const auto& G : pane.glass()) {
                const double sw = gViewHalf > 0 ? 0.25 * gViewHalf / 0.1 : 1.0;
                std::fprintf(f, "<g stroke='%s' stroke-width='%.2f' stroke-linecap='round' opacity='0.85'>\n",
                             colours[gi % 6], (gi == 0 ? 1.0 : 0.7) * sw);
                for (const auto& sgm : G.net.segments())
                    std::fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f'/>\n", sgm.a.x * s, (H - sgm.a.y) * s,
                                 sgm.b.x * s, (H - sgm.b.y) * s);
                std::fprintf(f, "</g>\n");
                ++gi;
            }
            std::fprintf(f, "</svg>\n");
            std::fclose(f);
        }
    }
    return o;
}

void printOutcome(const char* label, const Outcome& o) {
    const auto& r = o.report;
    std::printf("%-44s peak %7.2f kN  stress %6.0f MPa  frags %5d  segs %6d  vis %.2f  %s\n", label,
                r.peakForce / 1e3, r.maxStress / 1e6, o.fragments, o.crackSegments, o.status.visibility,
                r.outcome.c_str());
}

int cmdList(const Library& lib) {
    std::printf("Glazings:\n");
    for (const auto& g : lib.glazings)
        std::printf("  %-22s %-62s %5.1f mm  %4.2f x %4.2f m\n", g.id.c_str(), g.name.c_str(), g.thickness() * 1e3,
                    g.width, g.height);
    std::printf("Impactors:\n");
    for (const auto& d : lib.impactors)
        std::printf("  %-16s %-28s %8.1f g  %6.1f m/s\n", d.id.c_str(), d.name.c_str(), d.mass * 1e3, d.speed);
    return 0;
}

int cmdRun(const Library& lib, int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: glassdemo run <glazing> <impactor> [speed] [x y] [--tier N] [--seed S] [--svg f] [--back]\n");
        return 2;
    }
    double speed = -1;
    Vec2 at{-1, -1};
    int tier = 3;
    std::uint64_t seed = 1;
    const char* svg = nullptr;
    bool back = false;
    int pos = 0;
    for (int i = 4; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--tier") && i + 1 < argc) tier = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) seed = std::strtoull(argv[++i], nullptr, 10);
        else if (!std::strcmp(argv[i], "--svg") && i + 1 < argc) svg = argv[++i];
        else if (!std::strcmp(argv[i], "--back")) back = true;
        else if (!std::strcmp(argv[i], "--view") && i + 3 < argc) {
            gViewX = std::atof(argv[++i]);
            gViewY = std::atof(argv[++i]);
            gViewHalf = std::atof(argv[++i]);
        }
        else if (pos == 0) { speed = std::atof(argv[i]); ++pos; }
        else if (pos == 1) { at.x = std::atof(argv[i]); ++pos; }
        else if (pos == 2) { at.y = std::atof(argv[i]); ++pos; }
    }
    const Outcome o = runScenario(lib, argv[2], argv[3], speed, at, tier, seed, back, svg, true);
    const auto& r = o.report;
    std::printf("\n%s -> %s at %.1f m/s (%s tier)\n", argv[3], argv[2], r.speedIn, budgetForTier(tier).name);
    std::printf("  outcome     %s\n", r.outcome.c_str());
    std::printf("  peak force  %.2f kN, contact %.0f us, peak stress at load %.0f MPa\n", r.peakForce / 1e3,
                r.contactTime * 1e6, r.maxStress / 1e6);
    std::printf("  energy      %.2f J in, %.2f J absorbed; out at %.1f m/s\n", r.energyIn, r.absorbed, r.speedOut);
    std::printf("  pane        %d crack segments, %d fragments, visibility %.2f, open %.4f m^2, bulge %.1f mm\n",
                o.crackSegments, o.fragments, o.status.visibility, o.status.openArea, o.status.maxBulge * 1e3);
    std::printf("  cost        %.0f ms wall for %.3f s simulated, %d steps\n", o.wallMs, o.simTime, o.steps);
    return 0;
}

int cmdValidate(const Library& lib) {
    struct Case {
        const char* label;
        const char* glazing;
        const char* impactor;
        double speed;
    };
    const Case cases[] = {
        {"227 g ball, 2 m drop on tempered 4 mm (R43)", "car_side_tempered", "steel_ball_227", 6.26},
        {"Window punch on tempered 4 mm", "car_side_tempered", "window_punch", 0},
        {"Window punch on windscreen", "windscreen", "window_punch", 0},
        {"Window punch on annealed 6 mm", "annealed_6", "window_punch", 0},
        {"2.26 kg ball, 4 m drop on windscreen (R43)", "windscreen", "steel_ball_2260", 8.86},
        {"Headform 40 km/h on windscreen", "windscreen", "headform", 11.1},
        {"Road stone 30 m/s on windscreen", "windscreen", "stone", 30},
        {"Body bag 48 in on annealed 6 mm (Z97.1)", "annealed_6", "body_bag", 4.89},
        {"Body bag 48 in on tempered 6 mm (Z97.1)", "tempered_6", "body_bag", 4.89},
        {"Baseball 40 m/s on annealed 4 mm", "annealed_4", "baseball", 40},
        {"BB 120 m/s on annealed 6 mm", "annealed_6", "bb", 120},
        {"Hail 50 mm on annealed 4 mm", "annealed_4", "hail_50", 33},
        {"2x4 missile on hurricane laminate (E1996)", "hurricane", "timber_2x4", 15.2},
        {"9 mm on UL 752 L1 laminate", "ul752_l1", "pistol_9mm", 0},
        {".44 Mag on UL 752 L3 GCP", "ul752_l3", "magnum_44", 0},
        {"7.62 on UL 752 L3 GCP", "ul752_l3", "rifle_762", 0},
        {"7.62 on UL 752 L8 GCP", "ul752_l8", "rifle_762", 0},
        {"9 mm on annealed 6 mm", "annealed_6", "pistol_9mm", 0},
        {"Bird 1.8 kg 250 kt on aircraft windshield", "aircraft_windshield", "bird_4lb", 128.6},
        {"Hammer 10 m/s on annealed 6 mm", "annealed_6", "hammer", 10},
        {"Brick 10 m/s on tempered 6 mm", "tempered_6", "brick", 10},
    };
    for (const auto& c : cases) {
        const Outcome o = runScenario(lib, c.glazing, c.impactor, c.speed, {-1, -1}, 3, 1, false, nullptr, false);
        printOutcome(c.label, o);
    }
    return 0;
}

int cmdBench(const Library& lib) {
    for (int tier = 0; tier < kTierCount; ++tier) {
        double total = 0;
        for (const char* g : {"annealed_6", "tempered_6", "windscreen", "ul752_l3"}) {
            const char* imp = std::strcmp(g, "ul752_l3") == 0 ? "magnum_44" : "steel_ball_2260";
            const Outcome o = runScenario(lib, g, imp, std::strcmp(g, "tempered_6") == 0 ? 20 : 0, {-1, -1}, tier, 1,
                                          false, nullptr, false, 0.3);
            total += o.wallMs;
            std::printf("%-7s %-12s %8.0f ms  %6d steps  %5d frags  %6d segs\n", budgetForTier(tier).name, g, o.wallMs,
                        o.steps, o.fragments, o.crackSegments);
        }
        std::printf("%-7s total %.0f ms\n", budgetForTier(tier).name, total);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Library lib;
    if (!loadLibrary(lib)) return 1;
    const std::string cmd = argc > 1 ? argv[1] : "list";
    if (cmd == "list") return cmdList(lib);
    if (cmd == "run") return cmdRun(lib, argc, argv);
    if (cmd == "validate") return cmdValidate(lib);
    if (cmd == "bench") return cmdBench(lib);
    std::fprintf(stderr, "commands: list, run, validate, bench\n");
    return 2;
}
