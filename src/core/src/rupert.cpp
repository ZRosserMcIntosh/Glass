// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT

#include "gls/rupert.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace gls {

namespace {
// Same energy balance as tempered dicing (pane.cpp): the share of the stored
// strain energy that makes new surface.
constexpr Real kDiceEnergyFraction = 0.5;
// Share of the stored energy a fragment leaves with as speed. Estimated: high-
// speed footage shows the powder drifting out at a few metres a second.
constexpr Real kKineticShare = 0.05;
// Roesler's cone crack: c = (chi F / K_Ic)^(2/3) (contact.cpp's constant).
constexpr Real kRoesler = 0.0136;
constexpr int kMaxFragments = 6000;
constexpr int kSplash = 140; // drops of a projectile that splashes off the head

std::string fmt(const char* f, double a = 0, double b = 0, double c = 0, double d = 0) {
    char buf[320];
    std::snprintf(buf, sizeof buf, f, a, b, c, d);
    return buf;
}
}  // namespace

RupertDrop::RupertDrop(const DropShape& shape, const Material& glass, std::uint64_t seed)
    : s_(shape), g_(glass), rng_(seed) {
    // sigma(u) = s_t - (s_t + s_c) u^k balances (integral of sigma u du = 0) for k = 2 s_c / s_t.
    k_ = 2 * s_.surfaceCompression / std::max(s_.coreTension, 1.0);
    speed_ = g_.crackSpeed;
    snipX_ = s_.length;
    // The compressed skin at the head: its depth and mean compression.
    depth_ = compressionDepth() * s_.headRadius;
    const Real u0 = 1 - compressionDepth();
    Real sum = 0;
    constexpr int n = 64;
    for (int i = 0; i < n; ++i) sum -= residual(u0 + (1 - u0) * (i + 0.5) / n);
    meanC_ = sum / n;
}

Real RupertDrop::radius(Real x) const {
    const Real R = s_.headRadius;
    if (x <= 0 || x >= s_.length) return 0;
    if (x <= R) return std::sqrt(std::max(0.0, R * R - sq(R - x)));
    // Behind the head the tail thins away smoothly to its tip.
    return s_.tipRadius + (R - s_.tipRadius) * std::exp(-std::pow((x - R) / s_.neck, 1.3));
}

Real RupertDrop::residual(Real u) const {
    u = clampr(u, 0.0, 1.0);
    return s_.coreTension - (s_.coreTension + s_.surfaceCompression) * std::pow(u, k_);
}

Real RupertDrop::compressionDepth() const {
    return 1 - std::pow(s_.coreTension / (s_.coreTension + s_.surfaceCompression), 1 / k_);
}

Real RupertDrop::fragmentSize() const {
    const Real ct = s_.coreTension;
    return 10 * sq(g_.toughness) / ((1 - g_.poisson) * kDiceEnergyFraction * ct * ct);
}

Real RupertDrop::brokenFrom() const {
    if (!broken()) return 1;
    return headward_ ? std::max(0.0, origin_ - speed_ * std::max(0.0, time_ - startedAt_)) : origin_;
}

Real RupertDrop::brokenTo() const {
    if (!broken()) return 0;
    return tailward_ ? std::min(snipX_, origin_ + speed_ * std::max(0.0, time_ - startedAt_)) : origin_;
}

void RupertDrop::snip(Real x) {
    if (broken()) return;
    x = clampr(x, s_.headRadius * 1.5, s_.length * 0.98);
    snipX_ = x;
    // The tail is thin: snapping it opens a crack straight into the core.
    startFront(x, true, false, time_);
    outcome_ = fmt("Snapped the tail %.0f mm from the head: the crack reached the core in tension, and a front of forking "
                   "cracks ran back at %.0f m/s; the head burst %.0f us later",
                   1e3 * x, speed_, 1e6 * x / speed_);
}

bool RupertDrop::strike(const DropStrike& st) {
    if (broken()) return false;
    const Real nu = g_.poisson;
    // Hertz for an elastic impact on the head's curved front.
    const Real Rs = 1 / (1 / st.radius + 1 / s_.headRadius);
    const Real Es = 1 / ((1 - sq(st.poisson)) / st.youngs + (1 - sq(nu)) / g_.youngs);
    const Real dm = std::pow(15 * st.mass * st.speed * st.speed / (16 * Es * std::sqrt(Rs)), 0.4);
    const Real F = 4.0 / 3.0 * Es * std::sqrt(Rs) * std::pow(dm, 1.5);
    const Real a = std::sqrt(Rs * dm);
    const Real p0 = 3 * F / (2 * kPi * a * a);
    // A soft projectile flows instead once its face is pushed past its flow
    // stress: the pressure is then the stagnation pressure of its metal
    // (Tate, on a target that does not give), spread over its face.
    const Real flow = st.yield + 0.5 * st.density * st.speed * st.speed;
    const bool flows = !st.rigid && p0 > flow;
    const Real K = g_.toughness, skin = 1.12 * std::sqrt(kPi * depth_);
    if (flows) {
        // Its face presses evenly: the surface just outside it is pulled by
        // (1 - 2 nu) / 2 p (Love). A crack there must open against the skin's
        // surface compression all the way down: K = 1.12 (sigma - s_c) sqrt(pi d).
        byForce_ = false;
        contactP_ = flow;
        drive_ = 0.5 * (1 - 2 * nu) * flow;
        needed_ = s_.surfaceCompression + K / skin;
    } else {
        // Hertz: the cone crack's own length grows with the force (Roesler);
        // the skin's compression closes it. It reaches the core once
        // chi F / d^1.5 > K_Ic + 1.12 s_mean sqrt(pi d).
        byForce_ = true;
        contactP_ = p0;
        drive_ = F;
        needed_ = std::pow(depth_, 1.5) * (K + meanC_ * skin) / kRoesler;
    }
    const bool fails = drive_ > needed_;
    const Real L = st.mass / (st.density * kPi * st.radius * st.radius);  // the projectile's length
    if (flows && !fails) {
        // It splashes off the head, sideways and a little back, over the time its length takes to arrive.
        // (If the drop gives way, the bullet goes on through the powder instead.)
        const Real each = std::cbrt(st.mass / st.density / kSplash);
        for (int i = 0; i < kSplash; ++i) {
            const Real ang = rng_.uniform(0, 2 * kPi), back = rng_.uniform(0.0, 0.45);
            const Real v = st.speed * rng_.uniform(0.25, 0.85);
            DropFragment f;
            f.lead = true;
            f.born = time_ + rng_.uniform(0, L / st.speed);
            f.pos = {-rng_.uniform(0, 0.3) * st.radius, std::cos(ang) * st.radius * 0.8, std::sin(ang) * st.radius * 0.8};
            f.vel = Vec3{-std::sin(back), std::cos(back) * std::cos(ang), std::cos(back) * std::sin(ang)} * v;
            f.size = each * rng_.uniform(0.5, 1.4);
            frags_.push_back(f);
        }
    }
    if (fails) {
        startFront(0, false, true, time_);
        outcome_ = byForce_
            ? fmt("A %.1f kN blow drove a cone crack through the %.2f mm of compressed skin (it needed %.1f kN): it reached the "
                  "core and the whole drop burst",
                  drive_ / 1e3, 1e3 * depth_, needed_ / 1e3)
            : fmt("%.1f GPa on its face pulled the glass beside it with %.0f MPa, more than the %.0f MPa a crack needs to get "
                  "through the compressed skin: it reached the core and the whole drop burst",
                  contactP_ / 1e9, drive_ / 1e6, needed_ / 1e6);
    } else if (flows) {
        outcome_ = fmt("The bullet splashed off the head: %.2f GPa on its face pulled the glass beside it with only %.0f MPa, "
                       "against the %.0f MPa a crack needs to get through the compressed skin. Not a mark on the drop",
                       contactP_ / 1e9, drive_ / 1e6, needed_ / 1e6);
    } else {
        outcome_ = fmt("Bounced off the head: a %.1f kN blow, where a cone crack needs %.1f kN to get through the %.2f mm of "
                       "compressed skin",
                       drive_ / 1e3, needed_ / 1e3, 1e3 * depth_);
    }
    return fails;
}

void RupertDrop::startFront(Real x0, bool towardHead, bool towardTail, Real when) {
    startedAt_ = when;
    origin_ = x0;
    headward_ = towardHead;
    tailward_ = towardTail;
    spray(x0, towardHead, towardTail, when);
}

// Every fragment the front will make, with the time it gets there. The core's
// stored energy sets their size; the stress where each one was, its speed.
void RupertDrop::spray(Real x0, bool towardHead, bool towardTail, Real when) {
    const Real a = fragmentSize();
    const Real lo = towardHead ? 0.0 : x0, hi = towardTail ? snipX_ : x0;
    // Count first, to keep within the budget.
    Real volume = 0;
    for (Real x = lo + 0.5 * a; x < hi; x += a) volume += kPi * sq(radius(x)) * a;
    const Real perVolume = std::min(5 / (a * a * a), (kMaxFragments - static_cast<Real>(frags_.size())) / std::max(volume, 1e-12));
    for (Real x = lo + 0.5 * a; x < hi; x += a) {
        const Real r = radius(x);
        if (r <= 0) continue;
        Real n = kPi * r * r * a * perVolume;
        int count = static_cast<int>(n) + (rng_.uniform() < n - std::floor(n) ? 1 : 0);
        count = std::max(count, 1);
        for (int i = 0; i < count; ++i) {
            const Real u = std::sqrt(rng_.uniform()), th = rng_.uniform(0, 2 * kPi);
            const Real sig = residual(u);
            const Real U = sig * sig / (2 * g_.youngs);
            const Real v = std::sqrt(2 * kKineticShare * U / g_.density) * rng_.uniform(0.6, 1.3);
            const Vec3 radial{0, std::cos(th), std::sin(th)};
            DropFragment f;
            f.born = when + std::fabs(x - x0) / speed_;
            f.pos = {x + rng_.uniform(-0.5, 0.5) * a, u * r * radial.y, u * r * radial.z};
            f.vel = radial * (v * (0.3 + 0.7 * u)) + Vec3{rng_.uniform(-0.3, 0.3), rng_.uniform(-0.3, 0.3), rng_.uniform(-0.3, 0.3)} * v;
            f.size = a * std::exp(rng_.uniform(-0.9, 0.4)) / 1.7;
            frags_.push_back(f);
        }
        if (static_cast<int>(frags_.size()) >= kMaxFragments) break;
    }
    std::stable_sort(frags_.begin(), frags_.end(), [](const DropFragment& p, const DropFragment& q) { return p.born < q.born; });
}

}  // namespace gls
