// Glass Dynamics — WebAssembly bindings for the browser demo.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over gls::Pane so a web page can run the real core. The page
// only renders and reads input. Buffers returned by pointer are views into
// the module's memory, valid until the next call; web/README.md has the
// layouts.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gls/library.hpp"
#include "gls/pane.hpp"

#define GLS_EXPORT extern "C" __attribute__((visibility("default")))

using namespace gls;

namespace {

struct Sim {
    Library lib;
    std::vector<std::string> errors;
    std::unique_ptr<Pane> pane;
    int glazing = -1;
    std::string text;
    std::vector<double> state, imps, plies, report;
    std::vector<float> cracks, frags, poses, field;
    std::vector<float> sprays, crushes;
};

Sim* g = nullptr;
Sim& sim() {
    if (!g) g = new Sim;
    return *g;
}

void putText(const std::string& s) {
    Sim& S = sim();
    S.text = s;
    S.text.push_back('\0');
}

void jsonString(std::string& o, const std::string& s) {
    o += '"';
    for (const char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
        else o += c;
    }
    o += '"';
}

void jsonNum(std::string& o, double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.6g", std::isfinite(v) ? v : 0.0);
    o += b;
}

const char* kindName(ImpactorKind k) {
    switch (k) {
        case ImpactorKind::Sphere: return "sphere";
        case ImpactorKind::Sharp: return "sharp";
        case ImpactorKind::Flat: return "flat";
        case ImpactorKind::Projectile: return "projectile";
        case ImpactorKind::SoftBody: return "soft";
    }
    return "sphere";
}

const char* treatmentName(Treatment t) {
    switch (t) {
        case Treatment::Annealed: return "annealed";
        case Treatment::HeatStrengthened: return "heat-strengthened";
        case Treatment::FullyTempered: return "fully tempered";
        case Treatment::Chemical: return "chemically strengthened";
    }
    return "annealed";
}

void provenance(std::string& o, const std::map<std::string, std::string>& p) {
    o += "{";
    bool first = true;
    for (const auto& [k, v] : p) {
        if (!first) o += ",";
        first = false;
        jsonString(o, k);
        o += ":";
        jsonString(o, v);
    }
    o += "}";
}

}  // namespace

// ---- Text exchange ---------------------------------------------------------------

// A buffer of at least n bytes for the page to write JSON into (or read text from).
GLS_EXPORT char* gls_text(int n) {
    Sim& S = sim();
    if (static_cast<int>(S.text.size()) < n + 1) S.text.resize(static_cast<std::size_t>(n) + 1);
    return S.text.data();
}

GLS_EXPORT int gls_text_length() { return static_cast<int>(std::strlen(sim().text.c_str())); }

// Loads the JSON of length n in the text buffer into the library; returns the
// number of errors so far.
GLS_EXPORT int gls_load(int n) {
    Sim& S = sim();
    for (auto& e : S.lib.load(std::string(S.text.data(), static_cast<std::size_t>(n)))) S.errors.push_back(e);
    return static_cast<int>(S.errors.size());
}

GLS_EXPORT int gls_resolve() {
    Sim& S = sim();
    for (auto& e : S.lib.resolve()) S.errors.push_back(e);
    return static_cast<int>(S.errors.size());
}

GLS_EXPORT int gls_error_text(int i) {
    Sim& S = sim();
    putText(i >= 0 && i < static_cast<int>(S.errors.size()) ? S.errors[i] : std::string());
    return gls_text_length();
}

// The catalogue the page builds its menus from (JSON in the text buffer).
GLS_EXPORT int gls_catalog() {
    Sim& S = sim();
    std::string o = "{\"glazings\":[";
    for (std::size_t i = 0; i < S.lib.glazings.size(); ++i) {
        const GlazingDef& d = S.lib.glazings[i];
        if (i) o += ",";
        o += "{\"id\":";
        jsonString(o, d.id);
        o += ",\"name\":";
        jsonString(o, d.name);
        o += ",\"category\":";
        jsonString(o, d.category);
        o += ",\"description\":";
        jsonString(o, d.description);
        o += ",\"standard\":";
        jsonString(o, d.standard);
        o += ",\"width\":"; jsonNum(o, d.width);
        o += ",\"height\":"; jsonNum(o, d.height);
        o += ",\"thickness\":"; jsonNum(o, d.thickness());
        o += ",\"pressure\":"; jsonNum(o, d.pressure);
        o += ",\"provenance\":";
        provenance(o, d.provenance);
        o += ",\"plies\":[";
        for (std::size_t k = 0; k < d.plies.size(); ++k) {
            const Ply& p = d.plies[k];
            if (k) o += ",";
            o += "{\"material\":";
            jsonString(o, p.mat ? p.mat->name : p.material);
            o += ",\"kind\":";
            jsonString(o, !p.mat ? "glass" : p.mat->kind == MaterialKind::Glass ? "glass" : p.mat->kind == MaterialKind::Interlayer ? "interlayer" : "polymer");
            o += ",\"thickness\":"; jsonNum(o, p.thickness);
            o += ",\"treatment\":";
            jsonString(o, treatmentName(p.treatment));
            o += ",\"surfaceCompression\":"; jsonNum(o, p.surfaceCompression);
            o += ",\"centralTension\":"; jsonNum(o, p.centralTension());
            o += "}";
        }
        o += "]}";
    }
    o += "],\"impactors\":[";
    for (std::size_t i = 0; i < S.lib.impactors.size(); ++i) {
        const ImpactorDef& d = S.lib.impactors[i];
        if (i) o += ",";
        o += "{\"id\":";
        jsonString(o, d.id);
        o += ",\"name\":";
        jsonString(o, d.name);
        o += ",\"category\":";
        jsonString(o, d.category);
        o += ",\"description\":";
        jsonString(o, d.description);
        o += ",\"kind\":";
        jsonString(o, kindName(d.kind));
        o += ",\"mass\":"; jsonNum(o, d.mass);
        o += ",\"radius\":"; jsonNum(o, d.radius);
        o += ",\"length\":"; jsonNum(o, d.length);
        o += ",\"speed\":"; jsonNum(o, d.speed);
        o += ",\"minSpeed\":"; jsonNum(o, d.minSpeed);
        o += ",\"maxSpeed\":"; jsonNum(o, d.maxSpeed);
        o += ",\"provenance\":";
        provenance(o, d.provenance);
        o += "}";
    }
    o += "]}";
    putText(o);
    return gls_text_length();
}

// ---- Pane --------------------------------------------------------------------------

GLS_EXPORT int gls_new_pane(int glazing, int tier, double seed) {
    Sim& S = sim();
    if (glazing < 0 || glazing >= static_cast<int>(S.lib.glazings.size())) return 0;
    S.glazing = glazing;
    S.pane = std::make_unique<Pane>(S.lib.glazings[glazing], budgetForTier(tier),
                                    static_cast<std::uint64_t>(std::max(0.0, seed)));
    return 1;
}

GLS_EXPORT void gls_set_floor(double y) {
    if (sim().pane) sim().pane->setFloor(y);
}

// Fires impactor `index` at pane point (x, y) m; returns its handle or -1.
GLS_EXPORT int gls_fire(int index, double x, double y, double speed, double yaw, double pitch, int fromBack,
                        double lead) {
    Sim& S = sim();
    if (!S.pane || index < 0 || index >= static_cast<int>(S.lib.impactors.size())) return -1;
    return S.pane->fire(S.lib.impactors[index], {x, y}, speed, yaw, pitch, fromBack != 0, lead);
}

// Whole internal steps until time >= t (s) or maxSteps; returns steps taken.
GLS_EXPORT int gls_advance(double t, int maxSteps) {
    Sim& S = sim();
    return S.pane ? S.pane->advance(t, maxSteps) : 0;
}

GLS_EXPORT double gls_time() { return sim().pane ? sim().pane->time() : 0; }
GLS_EXPORT double gls_last_step() { return sim().pane ? sim().pane->lastStep() : 0; }

// [0] time, [1] quiet, [2] glass plies, [3] impactors, [4] fragments (all
// records), [5] fragment revision, [6] cracked, [7] penetrated, [8] barrier
// intact, [9] open area m^2, [10] visibility, [11] max bulge m, [12] free
// fragments, [13] laminate thickness m, [14] width, [15] height,
// [16] fundamental Hz, [17] highest mode Hz, [18] plate modes x, [19] y,
// [20] crack cell m, [21] events, [22] last step s, [23] when the pieces were last stepped s
GLS_EXPORT double* gls_state() {
    Sim& S = sim();
    S.state.assign(32, 0);
    if (!S.pane) return S.state.data();
    const Pane& P = *S.pane;
    const PaneStatus st = P.status();
    double* s = S.state.data();
    s[0] = P.time();
    s[1] = P.quiet() ? 1 : 0;
    s[2] = static_cast<double>(P.glass().size());
    s[3] = static_cast<double>(P.impactors().size());
    s[4] = static_cast<double>(P.fragments().fragments().size());
    s[5] = static_cast<double>(P.fragments().revision);
    s[6] = st.cracked;
    s[7] = st.penetrated;
    s[8] = st.barrierIntact;
    s[9] = st.openArea;
    s[10] = st.visibility;
    s[11] = st.maxBulge;
    s[12] = st.fragments;
    s[13] = P.laminate().thickness;
    s[14] = P.glazing().width;
    s[15] = P.glazing().height;
    s[16] = P.plate().fundamentalHz();
    s[17] = P.plate().maxFrequencyHz();
    s[18] = P.plate().modesX();
    s[19] = P.plate().modesY();
    s[20] = P.glass().empty() ? 0 : P.glass()[0].net.cell();
    s[21] = static_cast<double>(P.events().size());
    s[22] = P.lastStep();
    s[23] = P.fragmentTime();
    s[24] = static_cast<double>(P.fragments().sprays().size());
    int held = 0;
    for (const auto& sh : P.sheets()) held += sh.remaining();
    s[25] = held;
    s[26] = static_cast<double>(P.crushes().size());
    return s;
}

// Per glass ply, 12 numbers: z0, z1, segments, cell, fragmented, dicing,
// central tension Pa, surface compression Pa, energy-balance dice size m,
// drawn dice spacing m, ply index, active tips.
GLS_EXPORT double* gls_plies() {
    Sim& S = sim();
    S.plies.clear();
    if (!S.pane) return S.plies.data();
    for (const auto& G : S.pane->glass()) {
        const double v[12] = {G.z0, G.z1, static_cast<double>(G.net.segments().size()), G.net.cell(),
                              G.fragmented ? 1.0 : 0.0, G.dicing ? 1.0 : 0.0, G.centralTension,
                              G.surfaceCompression, G.diceSize, G.diceSpacing, static_cast<double>(G.ply),
                              static_cast<double>(G.net.activeTips())};
        S.plies.insert(S.plies.end(), v, v + 12);
    }
    return S.plies.data();
}

// Per impactor, 20 numbers: pos xyz, vel xyz, axis xyz, phase, force N,
// squash m, contact radius m, radius m, length m, kind, side, hit x, hit y,
// pushing.
GLS_EXPORT double* gls_impactors() {
    Sim& S = sim();
    S.imps.clear();
    if (!S.pane) return S.imps.data();
    for (const auto& I : S.pane->impactors()) {
        const double v[20] = {I.pos.x, I.pos.y, I.pos.z, I.vel.x, I.vel.y, I.vel.z, I.axis.x, I.axis.y, I.axis.z,
                              static_cast<double>(I.phase), I.force, I.squash, I.contactRadius, I.def.radius,
                              I.def.length, static_cast<double>(I.def.kind), static_cast<double>(I.side), I.hit.x,
                              I.hit.y, I.pushing ? 1.0 : 0.0};
        S.imps.insert(S.imps.end(), v, v + 20);
    }
    return S.imps.data();
}

// Report of impactor k: 16 numbers (speed in/out, energy in/absorbed, peak
// force, contact start/time, max stress, depth, bulge, broke, penetrated,
// rebounded, embedded, diced, 0); the outcome sentence goes to the text buffer.
GLS_EXPORT double* gls_report(int k) {
    Sim& S = sim();
    S.report.assign(16, 0);
    if (!S.pane || k < 0 || k >= static_cast<int>(S.pane->impactors().size())) return S.report.data();
    const ImpactReport& r = S.pane->impactors()[k].report;
    const double v[16] = {r.speedIn, r.speedOut, r.energyIn, r.absorbed, r.peakForce, r.contactStart, r.contactTime,
                          r.maxStress, r.depth, r.bulge, r.broke ? 1.0 : 0.0, r.penetrated ? 1.0 : 0.0,
                          r.rebounded ? 1.0 : 0.0, r.embedded ? 1.0 : 0.0, r.diced ? 1.0 : 0.0, 0};
    std::copy(v, v + 16, S.report.begin());
    putText(r.outcome);
    return S.report.data();
}

// Event log lines from index `from`, newline separated, in the text buffer.
GLS_EXPORT int gls_events(int from) {
    Sim& S = sim();
    std::string o;
    if (S.pane)
        for (std::size_t i = static_cast<std::size_t>(std::max(0, from)); i < S.pane->events().size(); ++i)
            o += S.pane->events()[i] + "\n";
    putText(o);
    return gls_text_length();
}

// Crack segments of glass ply g from index `from`: [count, then per segment
// ax, ay, bx, by, ta, tb, kind] as float32. Times are seconds.
GLS_EXPORT float* gls_cracks(int gi, int from) {
    Sim& S = sim();
    S.cracks.assign(1, 0);
    if (!S.pane || gi < 0 || gi >= static_cast<int>(S.pane->glass().size())) return S.cracks.data();
    const auto& segs = S.pane->glass()[gi].net.segments();
    const std::size_t a = static_cast<std::size_t>(std::max(0, from));
    for (std::size_t i = a; i < segs.size(); ++i) {
        const auto& sg = segs[i];
        const float v[7] = {static_cast<float>(sg.a.x), static_cast<float>(sg.a.y), static_cast<float>(sg.b.x),
                            static_cast<float>(sg.b.y), sg.ta, sg.tb, static_cast<float>(sg.kind)};
        S.cracks.insert(S.cracks.end(), v, v + 7);
    }
    S.cracks[0] = static_cast<float>(segs.size() > a ? segs.size() - a : 0);
    return S.cracks.data();
}

// Fragment shapes from record `from`: [count, then per fragment: index, ply,
// chip, thickness, n, then n (x, y) pairs about the centroid] as float32.
GLS_EXPORT float* gls_fragment_shapes(int from) {
    Sim& S = sim();
    S.frags.assign(1, 0);
    if (!S.pane) return S.frags.data();
    const auto& fr = S.pane->fragments().fragments();
    const std::size_t a = static_cast<std::size_t>(std::max(0, from));
    for (std::size_t i = a; i < fr.size(); ++i) {
        const Fragment& f = fr[i];
        S.frags.push_back(static_cast<float>(i));
        S.frags.push_back(static_cast<float>(f.ply));
        S.frags.push_back(f.chip ? 1.f : 0.f);
        S.frags.push_back(static_cast<float>(f.thickness));
        S.frags.push_back(static_cast<float>(f.outline.size()));
        for (const auto& p : f.outline) {
            S.frags.push_back(static_cast<float>(p.x));
            S.frags.push_back(static_cast<float>(p.y));
        }
    }
    S.frags[0] = static_cast<float>(fr.size() > a ? fr.size() - a : 0);
    return S.frags.data();
}

// Poses of every fragment record: 12 float32 each: pos xyz, quat wxyz, vel
// xyz, state (0 held, 1 pending, 2 free, 3 resting, -1 retired).
GLS_EXPORT float* gls_fragment_poses() {
    Sim& S = sim();
    S.poses.clear();
    if (!S.pane) return S.poses.data();
    const auto& fr = S.pane->fragments().fragments();
    S.poses.resize(fr.size() * 12);
    for (std::size_t i = 0; i < fr.size(); ++i) {
        const Fragment& f = fr[i];
        float* p = &S.poses[i * 12];
        p[0] = static_cast<float>(f.pos.x); p[1] = static_cast<float>(f.pos.y); p[2] = static_cast<float>(f.pos.z);
        p[3] = static_cast<float>(f.rot.w); p[4] = static_cast<float>(f.rot.x); p[5] = static_cast<float>(f.rot.y);
        p[6] = static_cast<float>(f.rot.z);
        p[7] = static_cast<float>(f.vel.x); p[8] = static_cast<float>(f.vel.y); p[9] = static_cast<float>(f.vel.z);
        p[10] = f.retired ? -1.f : static_cast<float>(f.state);
        p[11] = static_cast<float>(f.bornAt);
    }
    return S.poses.data();
}

// Dice sprays from index `from`: pieces drawn coarser than the real dice that
// burst into them on landing. [count, then 11 float32 each: x, y, z, vx, vy,
// vz, dice count, dice size, thickness, time, ply].
GLS_EXPORT float* gls_sprays(int from) {
    Sim& S = sim();
    S.sprays.assign(1, 0);
    if (!S.pane) return S.sprays.data();
    const auto& sp = S.pane->fragments().sprays();
    const std::size_t a = static_cast<std::size_t>(std::max(0, from));
    for (std::size_t i = a; i < sp.size(); ++i) {
        const DiceSpray& d = sp[i];
        const float v[11] = {static_cast<float>(d.pos.x), static_cast<float>(d.pos.y), static_cast<float>(d.pos.z),
                             static_cast<float>(d.vel.x), static_cast<float>(d.vel.y), static_cast<float>(d.vel.z),
                             static_cast<float>(d.count), static_cast<float>(d.size), static_cast<float>(d.thickness),
                             static_cast<float>(d.time), static_cast<float>(d.ply)};
        S.sprays.insert(S.sprays.end(), v, v + 11);
    }
    S.sprays[0] = static_cast<float>(sp.size() > a ? sp.size() - a : 0);
    return S.sprays.data();
}

// Crushed discs from index `from`: comminuted glass, white to the eye.
// [count, then 6 float32 each: glass ply, x, y, radius, time, kind (0 contact
// damage or cone, 1 projectile)].
GLS_EXPORT float* gls_crushes(int from) {
    Sim& S = sim();
    S.crushes.assign(1, 0);
    if (!S.pane) return S.crushes.data();
    const auto& cr = S.pane->crushes();
    const std::size_t a = static_cast<std::size_t>(std::max(0, from));
    for (std::size_t i = a; i < cr.size(); ++i) {
        const CrushMark& c = cr[i];
        const float v[6] = {static_cast<float>(c.ply), static_cast<float>(c.c.x), static_cast<float>(c.c.y),
                            static_cast<float>(c.r), static_cast<float>(c.time), static_cast<float>(c.kind)};
        S.crushes.insert(S.crushes.end(), v, v + 6);
    }
    S.crushes[0] = static_cast<float>(cr.size() > a ? cr.size() - a : 0);
    return S.crushes.data();
}

// Deflection (m) and the front / back largest principal stress (Pa) of glass
// ply g on an nx x ny lattice spanning the pane: 3 nx ny float32.
GLS_EXPORT float* gls_field(int gi, int nx, int ny) {
    Sim& S = sim();
    nx = std::clamp(nx, 2, 256);
    ny = std::clamp(ny, 2, 256);
    S.field.assign(static_cast<std::size_t>(3) * nx * ny, 0.f);
    if (!S.pane) return S.field.data();
    std::vector<float> w, f, b;
    S.pane->sampleFields(gi, nx, ny, w, f, b);
    std::copy(w.begin(), w.end(), S.field.begin());
    std::copy(f.begin(), f.end(), S.field.begin() + nx * ny);
    std::copy(b.begin(), b.end(), S.field.begin() + 2 * nx * ny);
    return S.field.data();
}

// Fragments whose centroid lies in the size x size square at (x, y) on glass
// ply g (the ECE R43 / EN 12150 fragment count).
GLS_EXPORT int gls_count_fragments(int gi, double x, double y, double size) {
    return sim().pane ? sim().pane->fragmentsIn(gi, {x, y}, size) : 0;
}

// A fixed amount of representative work (a 24 x 24 mode plate stepped and
// sampled) for the page's device benchmark; returns a checksum.
GLS_EXPORT double gls_bench(int repeats) {
    ModalPlate p;
    p.configure(1.0, 1.0, 1364, 15, 0.22, 0.01, 24, 24);
    std::vector<Curvature> k(33 * 33);
    double sum = 0;
    for (int r = 0; r < repeats; ++r) {
        p.clearForces();
        p.addPatch(0.5, 0.5, 0.002, 1000);
        for (int i = 0; i < 20; ++i) p.step(1e-6);
        p.sampleGrid(33, 33, nullptr, nullptr, k.data());
        sum += k[16 * 33 + 16].kxx;
    }
    return sum;
}
