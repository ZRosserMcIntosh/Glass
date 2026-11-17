// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/contact.hpp"

#include <algorithm>
#include <cmath>

namespace gls {

Real contactModulus(Real E1, Real nu1, Real E2, Real nu2) {
    return 1.0 / ((1 - nu1 * nu1) / E1 + (1 - nu2 * nu2) / E2);
}

// ---- Sphere ---------------------------------------------------------------------

void SphereContact::configure(Real R, Real Es, Real py, Real e, Real v0) {
    R_ = R; Es_ = Es; py_ = py; e_ = e; v0_ = std::max(0.01, v0);
    dmax_ = dp_ = fmax_ = a_ = 0;
    yielded_ = false;
    // Indentation at which the mean Hertz pressure reaches the flow pressure.
    dy_ = py_ > 0 ? R_ * sq(3 * kPi * py_ / (4 * Es_)) : 1e9;
}

Real SphereContact::force(Real d, Real rate) {
    const Real k = 4.0 / 3.0 * Es_ * std::sqrt(R_);
    Real F = 0;
    if (d >= dmax_) {
        if (d <= dy_) {
            F = k * std::pow(std::max(d, 0.0), 1.5);
            a_ = std::sqrt(R_ * std::max(d, 0.0));
        } else {
            // Thornton: past first yield the contact flattens at the flow
            // pressure; force grows linearly with indentation.
            const Real Fy = k * std::pow(dy_, 1.5);
            F = Fy + kPi * py_ * R_ * (d - dy_);
            a_ = std::sqrt(F / (kPi * py_));
            yielded_ = true;
        }
        dmax_ = d;
        fmax_ = std::max(fmax_, F);
        if (yielded_) dp_ = std::max(0.0, dmax_ - 3 * fmax_ / (4 * Es_ * std::max(a_, 1e-9)));
    } else if (d > dp_) {
        // Elastic unloading from the peak about the flattened profile.
        F = fmax_ * std::pow((d - dp_) / std::max(dmax_ - dp_, 1e-12), 1.5);
        a_ = std::sqrt(std::max(0.0, F) / std::max(1e-9, kPi * (py_ > 0 ? py_ : Es_)));
        if (!yielded_) a_ = std::sqrt(R_ * d);
    } else {
        a_ = 0;
        return 0;
    }
    if (e_ < 1) F *= 1 + 8 * (1 - e_) / (5 * e_) * rate / v0_;  // Flores et al. 2011, valid down to e ~ 0.3
    return std::max(0.0, F);
}

// ---- Sharp tip ------------------------------------------------------------------

void SharpContact::configure(Real halfAngle, Real H, Real tip) {
    alpha_ = kPi * sq(std::tan(halfAngle));
    H_ = H;
    tip_ = tip;
    dmax_ = fmax_ = a_ = 0;
}

Real SharpContact::force(Real d, Real) {
    if (d <= 0) { a_ = 0; return 0; }
    const Real ta = std::sqrt(alpha_ / kPi);
    if (d >= dmax_) {
        dmax_ = d;
        const Real F = alpha_ * H_ * d * d;
        fmax_ = std::max(fmax_, F);
        a_ = tip_ + d * ta;
        return F;
    }
    // Glass recovers about 40 % of a sharp indentation elastically.
    const Real dp = 0.6 * dmax_;
    if (d <= dp) { a_ = 0; return 0; }
    a_ = tip_ + d * ta;
    return fmax_ * sq((d - dp) / (dmax_ - dp));
}

// ---- Flat punch ------------------------------------------------------------------

void FlatContact::configure(Real a, Real Es, Real crush) {
    a_ = a;
    k_ = 2 * Es * a;
    cap_ = crush > 0 ? crush * kPi * a * a : 1e30;
    dmax_ = dp_ = fmax_ = 0;
}

Real FlatContact::force(Real d, Real) {
    if (d <= 0) return 0;
    if (d >= dmax_) {
        dmax_ = d;
        const Real F = std::min(k_ * (d - dp_), cap_);
        if (F >= cap_) dp_ = d - cap_ / k_;  // crushed: the end shortens
        fmax_ = std::max(fmax_, F);
        return F;
    }
    return std::max(0.0, k_ * (d - dp_));
}

// ---- Soft body ----------------------------------------------------------------------

Real SoftBodyLoad::hugoniotPressure() const { return density * normalSpeed * (c0 + slope * normalSpeed); }
Real SoftBodyLoad::steadyForce() const { return density * sq(normalSpeed) * kPi * radius * radius; }
Real SoftBodyLoad::duration() const { return length / std::max(normalSpeed, 1e-3); }

Real SoftBodyLoad::force(Real t) const {
    if (t <= 0 || normalSpeed <= 0) return 0;
    const Real v = normalSpeed;
    const Real T = duration(), tail = radius / v;
    if (t > T + tail) return 0;
    // The hemispherical nose flattens: contact area grows until the full section.
    const Real s = v * t;
    const Real area = s < radius ? kPi * (2 * radius * s - s * s) : kPi * radius * radius;
    // Hugoniot shock decays as release waves arrive from the free edge.
    const Real tau = radius / c0;
    const Real Ps = density * v * v;
    const Real P = Ps + (hugoniotPressure() - Ps) * std::exp(-t / tau);
    Real F = area * P;
    if (t > T) F *= std::max(0.0, 1 - (t - T) / tail);
    return F;
}

Real SoftBodyLoad::patchRadius(Real t) const {
    const Real s = normalSpeed * std::max(t, 0.0);
    const Real a = s < radius ? std::sqrt(std::max(0.0, 2 * radius * s - s * s)) : radius;
    return std::max(1.15 * a, 1e-4);
}

// ---- Fracture mechanics helpers ----------------------------------------------------

Real medianCrackDepth(Real F, Real E, Real H, Real K) {
    const Real chi = 0.016 * std::sqrt(E / H);
    return std::pow(std::max(0.0, chi * F / K), 2.0 / 3.0);
}

Real coneCrackLength(Real F, Real K) { return std::pow(std::max(0.0, 0.0136 * F / K), 2.0 / 3.0); }

Real hertzEdgeTension(Real F, Real a, Real nu) {
    if (a <= 0) return 0;
    return 0.5 * (1 - 2 * nu) * 3 * F / (2 * kPi * a * a);
}

// ---- Membrane --------------------------------------------------------------------------

void Membrane::configure(std::vector<MembranePly> plies, Real Rc, Real Rm, Real stiff, Real conc) {
    plies_ = std::move(plies);
    Rc_ = std::max(Rc, 1e-3);
    Rm_ = std::max(Rm, 1.5 * Rc_);
    stiff_ = stiff;
    conc_ = conc;
    rimStrain_ = 0;
    tabulate();
}

void Membrane::tabulate() {
    // Tension against strain, dense where the plies yield.
    constexpr int K = 400;
    tabE_.resize(K + 1);
    tabN_.resize(K + 1);
    for (int i = 0; i <= K; ++i) {
        const Real e = 10.0 * sq(static_cast<Real>(i) / K);
        tabE_[i] = e;
        tabN_[i] = tension(e);
    }
}

bool Membrane::torn() const {
    for (const auto& p : plies_)
        if (!p.torn) return false;
    return true;
}

Real Membrane::strength() const {
    Real N = 0;
    for (const auto& p : plies_)
        if (!p.torn) N += p.thickness * p.tensile;
    return N;
}

Real Membrane::tension(Real e) const {
    Real N = 0;
    for (const auto& p : plies_) {
        if (p.torn) continue;
        const Real ey = p.flow > 0 ? p.flow / p.youngs : 1e9;
        Real s;
        if (e < ey) {
            s = p.youngs * e;
        } else {
            const Real hard = std::max((p.tensile - p.flow) / std::max(1e-3, p.failureStrain - ey), 1e-3 * p.youngs);
            s = p.flow + hard * (e - ey);
        }
        N += p.thickness * s;
    }
    return N * stiff_;
}

Real Membrane::strainAt(Real N) const {
    if (tabN_.empty() || N <= 0) return 0;
    if (N >= tabN_.back()) return tabE_.back();
    const auto it = std::upper_bound(tabN_.begin(), tabN_.end(), N);
    const std::size_t i = static_cast<std::size_t>(it - tabN_.begin());
    const Real n0 = tabN_[i - 1], n1 = tabN_[i];
    const Real u = n1 > n0 ? (N - n0) / (n1 - n0) : 0;
    return tabE_[i - 1] + u * (tabE_[i] - tabE_[i - 1]);
}

Real Membrane::force(Real delta) {
    if (delta <= 0 || torn()) { rimStrain_ = rimTension_ = 0; return 0; }
    const Real span = Rm_ - Rc_;
    const Real len = std::sqrt(span * span + delta * delta);
    // Stretched generator length for rim tension Nc (tension ~ Nc Rc / r),
    // integrated on log-spaced radii.
    auto stretched = [&](Real Nc) {
        constexpr int K = 24;
        Real sum = 0;
        const Real lr = std::log(Rm_ / Rc_);
        for (int i = 0; i < K; ++i) {
            const Real r0 = Rc_ * std::exp(lr * i / K), r1 = Rc_ * std::exp(lr * (i + 1) / K);
            const Real rm = 0.5 * (r0 + r1);
            sum += (r1 - r0) * (1 + strainAt(Nc * Rc_ / rm));
        }
        return sum;
    };
    Real lo = 0, hi = tension(10.0);
    for (int i = 0; i < 40; ++i) {
        const Real mid = 0.5 * (lo + hi);
        (stretched(mid) < len ? lo : hi) = mid;
    }
    const Real Nc = 0.5 * (lo + hi);
    rimTension_ = Nc;
    rimStrain_ = strainAt(Nc) * conc_;
    bool tore = false;
    for (auto& p : plies_)
        if (!p.torn && rimStrain_ * p.crackStrain >= p.failureStrain) { p.torn = true; tore = true; }
    if (tore) tabulate();
    return 2 * kPi * Rc_ * Nc * (delta / len);
}

Real Membrane::shape(Real r, Real delta) const {
    if (r <= Rc_) return delta;
    if (r >= Rm_) return 0;
    return delta * std::log(Rm_ / r) / std::log(Rm_ / Rc_);
}

// ---- Ballistic ---------------------------------------------------------------------------

void Ballistic::configure(const std::vector<BallisticLayer>& layers, const ImpactorDef& p, Real speed,
                          Real mushroomFactor, bool membraneBacking) {
    layers_ = layers;
    membrane_ = membraneBacking;
    handedOff_ = false;
    events_.clear();
    mp_ = p.mass;
    ap0_ = ap_ = p.radius;
    L0_ = p.length > 0 ? p.length : 2 * p.radius;
    rhop_ = p.density;
    Yp_ = p.yield > 0 ? p.yield : 1e9;
    v0_ = v_ = speed;
    // Soft (lead) noses flatten against glass, hard cores do not (Taylor).
    mushroom_ = clampr(mushroomFactor * std::sqrt(1 + rhop_ * v0_ * v0_ / (2 * Yp_)), 1.0, 2.5);
    depth_ = mcol_ = glassDepth_ = absorbed_ = bulge_ = backingTravel_ = 0;
    r_ = ap_;
    layer_ = -1;
    stopped_ = exited_ = false;
}

// Energy per unit volume of interlayer torn along the shot line, and of a
// polymer backing dished and plugged, as fractions of sigma_t * eps_f
// (calibrated with the comminution energy against UL 752 representative
// laminates: tests/test_ballistic.cpp).
static constexpr Real kInterlayerEnergy = 0.01;
static constexpr Real kBackingEnergy = 0.05;
// The crushed cone stops spreading at about this many projectile radii
// (0.03 m for the 9 mm the constants were calibrated with); it scales with the
// projectile, so a .50 or a cannonball crushes a cone its own size.
static constexpr Real kConeRadiusMax = 0.03;
static constexpr Real kConeRadiusRatio = kConeRadiusMax / 0.0045;
// Energy to drive through rubble an earlier hit left, as a fraction of
// crushing fresh glass (estimated: confined rubble still has to be displaced
// and re-crushed; armoured glass loses most of its stopping power where it has
// already been hit).
static constexpr Real kRubbleFraction = 0.15;

Real Ballistic::step(Real dt, Real time) {
    if (done() || dt <= 0) return 0;
    const Real vBefore = v_;
    Real E = 0.5 * mp_ * v_ * v_;
    Real rem = v_ * dt;
    const Real total = layers_.empty() ? 0 : layers_.back().z1;
    Real elapsed = 0;
    while (rem > 1e-12 && !done()) {
        int idx = -1;
        for (int i = 0; i < static_cast<int>(layers_.size()); ++i)
            if (depth_ >= layers_[i].z0 - 1e-12 && depth_ < layers_[i].z1 - 1e-12) { idx = i; break; }
        if (idx < 0) {
            if (membrane_) handedOff_ = true;  // the polymer between the crushed plies takes it
            else exited_ = depth_ >= total;
            break;
        }
        const BallisticLayer& L = layers_[idx];
        const Real h = L.z1 - L.z0;
        if (membrane_ && L.kind != MaterialKind::Glass) {
            bool glassAhead = false;
            for (std::size_t i = idx + 1; i < layers_.size(); ++i) glassAhead = glassAhead || layers_[i].kind == MaterialKind::Glass;
            if (!glassAhead) { handedOff_ = true; break; }
            // Polymer between glass plies stretches with the rest of it later.
            const Real ds = std::min(rem, L.z1 - depth_);
            depth_ += ds;
            rem -= ds;
            elapsed += ds / std::max(v_, 1e-6);
            continue;
        }
        if (idx != layer_) {
            layer_ = idx;
            if (L.kind == MaterialKind::Glass) events_.push_back({L.ply, time + elapsed, std::max(r_, ap_), E, v_});
        }
        const Real ds = std::min(rem, L.z1 - depth_);
        Real dE = 0;
        if (L.kind == MaterialKind::Glass) {
            // Crushed cone: its radius grows through the ply at the cone angle,
            // starting from the damage the plies in front already spread.
            const Real s0 = depth_ - L.z0;
            r_ = std::min(std::max(kConeRadiusMax, kConeRadiusRatio * ap0_), std::max(r_, ap_) + ds * std::tan(L.mat->coneAngle));
            // Glass an earlier hit already crushed only has to be pushed aside.
            const Real fresh = r_ * r_ - sq(std::min(r_, L.rubble));
            dE = L.mat->comminution * kPi * (fresh + kRubbleFraction * (r_ * r_ - fresh)) * ds;
            // The plug of glass straight ahead takes a share of the momentum.
            const Real plug = L.mat->density * kPi * ap_ * ap_ * h;
            E *= std::pow(mp_ / (mp_ + plug), ds / h);
            ap_ += (ap0_ * mushroom_ - ap_) * std::min(1.0, ds / (0.5 * L0_));
            glassDepth_ += ds;
            (void)s0;
        } else if (L.kind == MaterialKind::Interlayer) {
            dE = kInterlayerEnergy * 0.5 * L.mat->tensile * L.mat->failureStrain * kPi * r_ * r_ * ds;
        } else {
            dE = kBackingEnergy * L.mat->tensile * L.mat->failureStrain * kPi * r_ * r_ * ds;
            bulge_ = std::max(bulge_, 2 * (depth_ + ds - L.z0));
        }
        if (dE >= E) {
            depth_ += ds * E / dE;
            absorbed_ += E;
            E = 0;
            v_ = 0;
            stopped_ = true;
            break;
        }
        E -= dE;
        absorbed_ += dE;
        elapsed += ds / std::max(v_, 1e-6);
        depth_ += ds;
        v_ = std::sqrt(2 * E / mp_);
        rem -= ds;
        if (depth_ >= total - 1e-12) {
            if (membrane_) handedOff_ = true;
            else exited_ = true;
            break;
        }
    }
    absorbed_ = 0.5 * mp_ * v0_ * v0_ - 0.5 * mp_ * v_ * v_;
    return mp_ * (vBefore - v_) / dt;
}

}  // namespace gls
