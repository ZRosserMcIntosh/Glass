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
#include "gls/rupert.hpp"

#define GLS_EXPORT extern "C" __attribute__((visibility("default")))

using namespace gls;

namespace {

constexpr int kSlots = 32;

struct Sim {
    Library lib;
    std::vector<std::string> errors;
    std::unique_ptr<Pane> pane;  // the selected slot's pane: every call below works on it
    int glazing = -1;
    int slot = 0;
    std::unique_ptr<Pane> parked[kSlots];  // the other slots' panes
    int parkedGlazing[kSlots] = {};
    std::string text;
    std::vector<double> state, imps, plies, report;
    std::vector<float> cracks, frags, poses, field;
    std::vector<float> sprays, crushes;
    std::vector<std::int32_t> labels;
    std::unique_ptr<RupertDrop> drop;  // a Prince Rupert's drop, apart from the panes
    std::vector<double> dropState;
    std::vector<float> dropFrags;
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
        o += ",\"pellets\":"; jsonNum(o, d.pellets);
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

// The same glazing cut to another size (a building's openings): width and
// height in m.
GLS_EXPORT int gls_new_pane_sized(int glazing, int tier, double seed, double width, double height) {
    Sim& S = sim();
    if (glazing < 0 || glazing >= static_cast<int>(S.lib.glazings.size())) return 0;
    if (!(width >= 0.02 && width <= 6 && height >= 0.02 && height <= 6)) return 0;
    GlazingDef d = S.lib.glazings[glazing];
    d.width = width;
    d.height = height;
    S.glazing = glazing;
    S.pane = std::make_unique<Pane>(d, budgetForTier(tier), static_cast<std::uint64_t>(std::max(0.0, seed)));
    return 1;
}

// Several panes side by side (a building's windows): `slot` 0..31 picks the
// pane every other call works on; each slot keeps its own pane and clock.
GLS_EXPORT int gls_select(int slot) {
    Sim& S = sim();
    if (slot < 0 || slot >= kSlots) return 0;
    if (slot == S.slot) return 1;
    std::swap(S.pane, S.parked[S.slot]);
    std::swap(S.glazing, S.parkedGlazing[S.slot]);
    S.slot = slot;
    std::swap(S.pane, S.parked[slot]);
    std::swap(S.glazing, S.parkedGlazing[slot]);
    if (!S.pane) S.glazing = -1;
    return 1;
}

// Drops the selected slot's pane.
GLS_EXPORT void gls_free_pane() {
    Sim& S = sim();
    S.pane.reset();
    S.glazing = -1;
}

// A blast wave on the pane (see gls::BlastLoad): arrival s, reflected peak Pa,
// positive phase s, Friedlander decay, gas pressure Pa, its rise s and venting
// time constant s.
GLS_EXPORT int gls_blast(double arrival, double peak, double duration, double decay, double gasPeak, double gasRise,
                         double gasTau) {
    Sim& S = sim();
    if (!S.pane) return 0;
    BlastLoad b;
    b.arrival = arrival;
    b.peak = peak;
    b.duration = std::max(duration, 1e-6);
    b.decay = decay;
    b.gasPeak = std::max(gasPeak, 0.0);
    b.gasRise = std::max(gasRise, 1e-6);
    b.gasTau = std::max(gasTau, 0.0);
    S.pane->blast(b);
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

// The same, the impactor given another mass (kg) and radius (m) when > 0:
// a piece of glass from another pane, say.
GLS_EXPORT int gls_fire_as(int index, double x, double y, double speed, double yaw, double pitch, int fromBack,
                           double lead, double mass, double radius) {
    Sim& S = sim();
    if (!S.pane || index < 0 || index >= static_cast<int>(S.lib.impactors.size())) return -1;
    ImpactorDef d = S.lib.impactors[index];
    if (mass > 0) d.mass = mass;
    if (radius > 0) {
        if (d.length > 0) d.length *= radius / d.radius;
        d.radius = radius;
    }
    return S.pane->fire(d, {x, y}, speed, yaw, pitch, fromBack != 0, lead);
}

// Takes fragment record i out of the scene (it has become something else's
// impactor). Returns 1 if it was there.
GLS_EXPORT int gls_drop_fragment(int i) {
    Sim& S = sim();
    if (!S.pane) return 0;
    auto& fr = S.pane->fragments().fragments();
    if (i < 0 || i >= static_cast<int>(fr.size()) || fr[i].retired) return 0;
    fr[i].retired = true;
    fr[i].clump = -1;
    return 1;
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
// ... [27] blast pressure Pa, [28] a laminate pulled out of its frame whole
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
    s[27] = P.blastPressure(P.time());
    s[28] = P.pulledOut() ? 1 : 0;
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

// A steady tone at the pane: frequency (Hz), level (peak at the glass: Pa for
// sound, N for a shaker), ramp time constant (s); shaker != 0 pushes at (x, y)
// instead of loading the whole face. Level 0 fades it out. Retuning keeps the phase.
GLS_EXPORT int gls_tone(double frequency, double level, double ramp, int shaker, double x, double y) {
    Sim& S = sim();
    if (!S.pane) return 0;
    ToneLoad t;
    t.frequency = frequency;
    t.level = level;
    t.ramp = ramp;
    t.shaker = shaker != 0;
    t.at = {x, y};
    S.pane->tone(t);
    return 1;
}

// The tone's level now (it follows the target), or 0.
GLS_EXPORT double gls_tone_level() { return sim().pane ? sim().pane->toneLevel() : 0; }

// How near a tone has brought the glass to breaking since the last call:
// [share of its strength, that flaw's tension (Pa), its strength (Pa)].
GLS_EXPORT double* gls_stress_strength() {
    Sim& S = sim();
    S.report.assign(3, 0);
    if (!S.pane) return S.report.data();
    S.report[0] = S.pane->toneMeter(S.report[1], S.report[2]);
    return S.report.data();
}

// How far the pane swings under the tone now, m.
GLS_EXPORT double gls_tone_swing() { return sim().pane ? sim().pane->toneSwing() : 0; }

// Natural frequency (Hz) of plate mode (m, n), 1-based.
GLS_EXPORT double gls_mode_hz(int m, int n) {
    Sim& S = sim();
    if (!S.pane || m < 1 || n < 1 || m > S.pane->plate().modesX() || n > S.pane->plate().modesY()) return 0;
    return S.pane->plate().modeHz(m, n);
}

// A nickel-sulfide stone of `diameterMm` at pane point (x, y), years after
// tempering. Returns 1 if it broke the pane; what happened goes to the text buffer.
GLS_EXPORT int gls_nickel_sulfide(double x, double y, double diameterMm) {
    Sim& S = sim();
    if (!S.pane) return 0;
    const bool broke = S.pane->nickelSulfide({x, y}, diameterMm * 1e-3);
    putText(S.pane->events().empty() ? std::string() : S.pane->events().back());
    return broke ? 1 : 0;
}

// ---- Prince Rupert's drop ----
// A new drop: head radius, length, tip radius (mm), surface compression and
// core tension (MPa), seed. Glass is soda-lime.
GLS_EXPORT int gls_drop_new(double headMm, double lengthMm, double tipMm, double compMPa, double coreMPa, double seed) {
    Sim& S = sim();
    const Material* m = S.lib.material("soda_lime_float");
    if (!m) return 0;
    DropShape d;
    d.headRadius = headMm * 1e-3;
    d.length = lengthMm * 1e-3;
    d.tipRadius = tipMm * 1e-3;
    d.surfaceCompression = compMPa * 1e6;
    d.coreTension = coreMPa * 1e6;
    S.drop = std::make_unique<RupertDrop>(d, *m, static_cast<std::uint64_t>(seed));
    return 1;
}

// Snap the tail x m from the head.
GLS_EXPORT int gls_drop_snip(double x) {
    Sim& S = sim();
    if (!S.drop) return 0;
    S.drop->snip(x);
    putText(S.drop->outcome());
    return 1;
}

// Library impactor `index` hits the head at `speed` (m/s; <= 0: its own).
// Returns 1 if the drop broke; the outcome goes to the text buffer.
GLS_EXPORT int gls_drop_strike(int index, double speed) {
    Sim& S = sim();
    if (!S.drop || index < 0 || index >= static_cast<int>(S.lib.impactors.size())) return 0;
    const ImpactorDef& d = S.lib.impactors[index];
    DropStrike st;
    st.mass = d.mass;
    st.radius = d.radius;
    st.speed = speed > 0 ? speed : d.speed;
    st.density = d.density;
    st.yield = d.yield;
    st.youngs = d.youngs;
    st.poisson = d.poisson;
    st.rigid = d.kind != ImpactorKind::Projectile;
    const bool broke = S.drop->strike(st);
    putText(S.drop->outcome());
    return broke ? 1 : 0;
}

GLS_EXPORT void gls_drop_advance(double t) {
    if (sim().drop) sim().drop->advance(t);
}

// [time, broken from, broken to (m along the axis), broken, snipped at,
//  front speed, fragments, contact pressure, drive, needed, by force,
//  skin depth, skin mean compression, fragment size, length, started at]
GLS_EXPORT double* gls_drop_state() {
    Sim& S = sim();
    S.dropState.assign(16, 0);
    if (!S.drop) return S.dropState.data();
    const RupertDrop& d = *S.drop;
    const double v[16] = {d.time(), d.brokenFrom(), d.brokenTo(), d.broken() ? 1.0 : 0.0, d.snippedAt(), d.frontSpeed(),
                          static_cast<double>(d.fragments().size()), d.contactPressure(), d.drive(), d.needed(),
                          d.byForce() ? 1.0 : 0.0, d.skinDepth(), d.skinCompression(), d.fragmentSize(),
                          d.length(), d.startedAt()};
    std::copy(v, v + 16, S.dropState.begin());
    return S.dropState.data();
}

// Fragments from index `from`: [count, then 9 float32 each: born, x, y, z,
// vx, vy, vz, size, lead (0/1)], drop frame (x along the axis, y up).
GLS_EXPORT float* gls_drop_fragments(int from) {
    Sim& S = sim();
    S.dropFrags.assign(1, 0);
    if (!S.drop) return S.dropFrags.data();
    const auto& fr = S.drop->fragments();
    for (std::size_t i = static_cast<std::size_t>(std::max(0, from)); i < fr.size(); ++i) {
        const DropFragment& f = fr[i];
        const float v[9] = {static_cast<float>(f.born), static_cast<float>(f.pos.x), static_cast<float>(f.pos.y),
                            static_cast<float>(f.pos.z), static_cast<float>(f.vel.x), static_cast<float>(f.vel.y),
                            static_cast<float>(f.vel.z), static_cast<float>(f.size), f.lead ? 1.f : 0.f};
        S.dropFrags.insert(S.dropFrags.end(), v, v + 9);
    }
    S.dropFrags[0] = static_cast<float>((S.dropFrags.size() - 1) / 9);
    return S.dropFrags.data();
}

GLS_EXPORT double gls_drop_radius(double x) { return sim().drop ? sim().drop->radius(x) : 0; }
GLS_EXPORT double gls_drop_residual(double u) { return sim().drop ? sim().drop->residual(u) : 0; }

// The laminate sheet fragment i is bonded into (a clump index), or -1.
GLS_EXPORT int gls_fragment_sheet(int i) {
    Sim& S = sim();
    if (!S.pane) return -1;
    auto& fs = S.pane->fragments();
    if (i < 0 || i >= static_cast<int>(fs.fragments().size())) return -1;
    const int c = fs.fragments()[i].clump;
    return c >= 0 && fs.clumps()[c].bonded ? c : -1;
}

// The sheet impactor k is wrapped in (a clump index), or -1.
GLS_EXPORT int gls_carried_sheet(int k) {
    Sim& S = sim();
    if (!S.pane || k < 0 || k >= static_cast<int>(S.pane->impactors().size())) return -1;
    return S.pane->impactors()[k].carriedBy;
}

// Mass (kg) of sheet c, glass and what it carries (its interlayer and any object wrapped in it).
GLS_EXPORT double gls_sheet_mass(int c) {
    Sim& S = sim();
    if (!S.pane) return 0;
    auto& cl = S.pane->fragments().clumps();
    return c >= 0 && c < static_cast<int>(cl.size()) ? cl[c].mass + cl[c].extraMass : 0;
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
// chip, thickness, n, then n (x, y) pairs about the centroid, then the number
// of holes and for each its point count and (x, y) pairs] as float32.
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
        S.frags.push_back(static_cast<float>(f.holes.size()));
        for (const auto& h : f.holes) {
            S.frags.push_back(static_cast<float>(h.size()));
            for (const auto& p : h) {
                S.frags.push_back(static_cast<float>(p.x));
                S.frags.push_back(static_cast<float>(p.y));
            }
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
// came apart into them (in the air, or on landing). [count, then 15 float32
// each: x, y, z, vx, vy, vz, dice count, dice size, thickness, time, ply,
// fragment index (its last pose is still in gls_fragment_poses), wx, wy, wz].
GLS_EXPORT float* gls_sprays(int from) {
    Sim& S = sim();
    S.sprays.assign(1, 0);
    if (!S.pane) return S.sprays.data();
    const auto& sp = S.pane->fragments().sprays();
    const std::size_t a = static_cast<std::size_t>(std::max(0, from));
    for (std::size_t i = a; i < sp.size(); ++i) {
        const DiceSpray& d = sp[i];
        const float v[15] = {static_cast<float>(d.pos.x), static_cast<float>(d.pos.y), static_cast<float>(d.pos.z),
                             static_cast<float>(d.vel.x), static_cast<float>(d.vel.y), static_cast<float>(d.vel.z),
                             static_cast<float>(d.count), static_cast<float>(d.size), static_cast<float>(d.thickness),
                             static_cast<float>(d.time), static_cast<float>(d.ply), static_cast<float>(d.fragment),
                             static_cast<float>(d.omega.x), static_cast<float>(d.omega.y), static_cast<float>(d.omega.z)};
        S.sprays.insert(S.sprays.end(), v, v + 15);
    }
    S.sprays[0] = static_cast<float>(sp.size() > a ? sp.size() - a : 0);
    return S.sprays.data();
}

// Which piece each crack-grid cell of glazing ply `ply` belongs to: [nx, ny,
// then nx * ny fragment indices (-1: none)] as int32, rows from the bottom.
GLS_EXPORT std::int32_t* gls_fragment_map(int ply) {
    Sim& S = sim();
    S.labels.assign(2, 0);
    if (!S.pane) return S.labels.data();
    const auto& maps = S.pane->fragments().maps;
    if (ply < 0 || ply >= static_cast<int>(maps.size())) return S.labels.data();
    const auto& m = maps[ply];
    S.labels[0] = m.nx;
    S.labels[1] = m.ny;
    S.labels.insert(S.labels.end(), m.label.begin(), m.label.end());
    return S.labels.data();
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
