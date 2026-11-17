// Glass Dynamics — minimal dependency-free test harness (same style as
// OpenRoad Dynamics' ord_test.hpp and Aviation Dynamics' avd_test.hpp).
// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace glstest {

struct Case {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}
inline int& failures() {
    static int f = 0;
    return f;
}
struct Registrar {
    Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

// Long scenarios run at full length in optimised builds and shortened in
// Debug/sanitizer builds (same invariants, fewer simulated seconds).
inline constexpr bool kOptimisedBuild =
#ifdef NDEBUG
    true;
#else
    false;
#endif

}  // namespace glstest

#define GLS_CAT2(a, b) a##b
#define GLS_CAT(a, b) GLS_CAT2(a, b)
#define TEST_CASE(name)                                                                                 \
    static void GLS_CAT(gls_test_fn_, __LINE__)();                                                      \
    static glstest::Registrar GLS_CAT(gls_test_reg_, __LINE__)(name, &GLS_CAT(gls_test_fn_, __LINE__)); \
    static void GLS_CAT(gls_test_fn_, __LINE__)()

#define CHECK(cond)                                                                        \
    do {                                                                                   \
        if (!(cond)) {                                                                     \
            ++glstest::failures();                                                         \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #cond << "\n"; \
        }                                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                  \
    do {                                                                                                       \
        const double gls_a_ = (a), gls_b_ = (b), gls_t_ = (tol);                                               \
        if (!(std::abs(gls_a_ - gls_b_) <= gls_t_)) {                                                          \
            ++glstest::failures();                                                                             \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b "| <= " #tol << "  (" \
                      << gls_a_ << " vs " << gls_b_ << ")\n";                                                  \
        }                                                                                                      \
    } while (0)

#define CHECK_RANGE(v, lo, hi)                                                                                     \
    do {                                                                                                           \
        const double gls_v_ = (v);                                                                                 \
        if (!(gls_v_ >= (lo) && gls_v_ <= (hi))) {                                                                 \
            ++glstest::failures();                                                                                 \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #v " = " << gls_v_ << " not in [" << (lo) \
                      << ", " << (hi) << "]\n";                                                                    \
        }                                                                                                          \
    } while (0)

// ---- Shared fixtures ----
#include <fstream>
#include <sstream>

#include "gls/library.hpp"
#include "gls/pane.hpp"

namespace glstest {

inline const gls::Library& library() {
    static gls::Library lib = [] {
        gls::Library l;
        for (const char* f : {"/materials.json", "/glazings.json", "/impactors.json"}) {
            std::ifstream in(std::string(GLS_DATA_DIR) + f, std::ios::binary);
            std::stringstream ss;
            ss << in.rdbuf();
            for (const auto& e : l.load(ss.str())) std::cerr << "  library: " << e << "\n";
        }
        for (const auto& e : l.resolve()) std::cerr << "  library: " << e << "\n";
        return l;
    }();
    return lib;
}

// Fires `impactor` at the pane centre (or `at`) and runs until quiet or `until` seconds.
struct Shot {
    std::unique_ptr<gls::Pane> pane;
    gls::ImpactReport report;
    int segments = 0, fragments = 0;
};
inline Shot shoot(const char* glazing, const char* impactor, double speed = -1, int tier = 2, std::uint64_t seed = 1,
                  bool fromBack = false, gls::Vec2 at = {-1, -1}, double until = 0.6) {
    const auto& lib = library();
    const gls::GlazingDef* g = lib.glazing(glazing);
    const gls::ImpactorDef* d = lib.impactor(impactor);
    Shot s;
    if (!g || !d) {
        std::cerr << "  unknown " << glazing << " / " << impactor << "\n";
        ++failures();
        return s;
    }
    s.pane = std::make_unique<gls::Pane>(*g, gls::budgetForTier(tier), seed);
    if (at.x < 0) at = {0.5 * g->width, 0.5 * g->height};
    s.pane->fire(*d, at, speed > 0 ? speed : d->speed, 0, 0, fromBack, 0.05);
    for (int f = 0; f < 4000 && s.pane->time() < until; ++f) {
        s.pane->advance(s.pane->time() + 1.0 / 120);
        if (s.pane->quiet() && s.pane->time() > 0.02) break;
    }
    s.report = s.pane->impactors()[0].report;
    for (const auto& G : s.pane->glass()) s.segments += static_cast<int>(G.net.segments().size());
    for (const auto& f : s.pane->fragments().fragments())
        if (!f.retired && !f.chip) ++s.fragments;
    return s;
}

}  // namespace glstest
