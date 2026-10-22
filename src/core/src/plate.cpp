// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/plate.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace gls {

// ---- Laminate ------------------------------------------------------------------

Laminate makeLaminate(const GlazingDef& g) {
    Laminate L;
    Real z = 0, mass = 0;
    for (int i = 0; i < static_cast<int>(g.plies.size()); ++i) {
        const Ply& p = g.plies[i];
        LaminateLayer layer;
        layer.ply = i;
        layer.z0 = z;
        layer.z1 = z + p.thickness;
        layer.plateModulus = p.mat ? p.mat->plateModulus() : 72e9 / (1 - 0.22 * 0.22);
        layer.poisson = p.mat ? p.mat->poisson : 0.22;
        layer.kind = p.mat ? p.mat->kind : MaterialKind::Glass;
        mass += p.thickness * (p.mat ? p.mat->density : 2500);
        z = layer.z1;
        L.layers.push_back(layer);
    }
    L.thickness = z;
    L.massPerArea = mass;

    // Monolithic limit: transformed section about the stiffness-weighted axis.
    Real ea = 0, eaz = 0;
    for (const auto& l : L.layers) {
        const Real h = l.z1 - l.z0;
        ea += l.plateModulus * h;
        eaz += l.plateModulus * h * l.mid();
    }
    L.neutralAxis = ea > 0 ? eaz / ea : z / 2;
    for (const auto& l : L.layers) {
        const Real h = l.z1 - l.z0;
        L.Dlayered += l.plateModulus * h * h * h / 12;
        L.Dmono += l.plateModulus * (h * h * h / 12 + h * sq(l.mid() - L.neutralAxis));
    }

    // Shear transfer through each interlayer between two stiff plies
    // (Bennison & Wölfel effective-thickness coefficient, ASTM E1300), averaged.
    const Real span = std::min(g.width, g.height);
    Real gsum = 0;
    int gcount = 0;
    for (int i = 1; i + 1 < static_cast<int>(g.plies.size()); ++i) {
        const Ply& v = g.plies[i];
        if (!v.mat || v.mat->kind != MaterialKind::Interlayer) continue;
        const Ply& p1 = g.plies[i - 1];
        const Ply& p2 = g.plies[i + 1];
        if (!p1.mat || !p2.mat || p1.mat->kind == MaterialKind::Interlayer || p2.mat->kind == MaterialKind::Interlayer)
            continue;
        const Real h1 = p1.thickness, h2 = p2.thickness, hv = v.thickness;
        const Real hs = 0.5 * (h1 + h2) + hv;
        const Real hs1 = hs * h1 / (h1 + h2), hs2 = hs * h2 / (h1 + h2);
        const Real Is = h1 * hs2 * hs2 + h2 * hs1 * hs1;
        const Real E = 0.5 * (p1.mat->youngs + p2.mat->youngs);
        const Real G = v.mat->shearModulus();
        gsum += 1.0 / (1.0 + 9.6 * E * Is * hv / (G * hs * hs * span * span));
        ++gcount;
    }
    bool hasInterlayer = false;
    for (const auto& l : L.layers) hasInterlayer = hasInterlayer || l.kind == MaterialKind::Interlayer;
    L.gamma = gcount ? gsum / gcount : (hasInterlayer ? 0.0 : 1.0);
    L.D = L.Dlayered + L.gamma * (L.Dmono - L.Dlayered);
    for (const auto& l : L.layers)
        if (l.kind == MaterialKind::Glass) { L.poisson = l.poisson; break; }
    return L;
}

// ---- Stress helpers ----------------------------------------------------------------

void PlaneStress::principal(Real& s1, Real& s2, Real& angle1) const {
    const Real c = 0.5 * (sxx + syy);
    const Real r = std::sqrt(sq(0.5 * (sxx - syy)) + sxy * sxy);
    s1 = c + r;
    s2 = c - r;
    angle1 = 0.5 * std::atan2(2 * sxy, sxx - syy);
}

Real PlaneStress::normalAcross(Vec2 d) const {
    // Normal to the crack line: n = perp(d); traction n.sigma.n.
    const Vec2 n = perp(d);
    return n.x * n.x * sxx + n.y * n.y * syy + 2 * n.x * n.y * sxy;
}

Real equivalentRadius(Real r0, Real h) {
    if (r0 >= 1.724 * h) return r0;
    return std::sqrt(1.6 * r0 * r0 + h * h) - 0.675 * h;
}

Real besselJ1(Real x) {
    const Real ax = std::fabs(x);
    if (ax < 8.0) {
        const Real y = x * x;
        const Real a1 = x * (72362614232.0 + y * (-7895059235.0 + y * (242396853.1 + y * (-2972611.439 + y * (15704.48260 + y * (-30.16036606))))));
        const Real a2 = 144725228442.0 + y * (2300535178.0 + y * (18583304.74 + y * (99447.43394 + y * (376.9991397 + y * 1.0))));
        return a1 / a2;
    }
    const Real z = 8.0 / ax, y = z * z, xx = ax - 2.356194491;
    const Real a1 = 1.0 + y * (0.183105e-2 + y * (-0.3516396496e-4 + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
    const Real a2 = 0.04687499995 + y * (-0.2002690873e-3 + y * (0.8449199096e-5 + y * (-0.88228987e-6 + y * 0.105787412e-6)));
    const Real ans = std::sqrt(0.636619772 / ax) * (std::cos(xx) * a1 - z * std::sin(xx) * a2);
    return x < 0 ? -ans : ans;
}

// ---- Modal plate -------------------------------------------------------------------

// The filtered series spreads a point load over about this many mode
// half-wavelengths (calibrated against a 700 x 700 mode reference,
// tests/test_plate.cpp).
static constexpr Real kResolutionFactor = 1.0;
// Membrane action counts once the stretching modes' swings add up to this
// share of the thickness: below it, it changes stiffness and stress by under
// 1 % (the converged solution against the linear one, docs/validation.md).
static constexpr Real kStretchGate = 0.25;
// Legendre degree of the in-plane displacement (peak stress converged to
// 0.1 % against degree 20 for 36 modes up to p a^4 / (E h^4) = 1000).
static constexpr int kStretchDegree = 16;
// The membrane force comes as kicks on a schedule of its own (multiple time
// stepping): each interval resolves the stiffening it adds, omega_nl dt <=
// 0.5, and stays clear of the stretching modes' own half-period, omega dt <= 2.
static constexpr Real kStretchKick = 0.5;
static constexpr Real kStretchSplit = 2.0;

void ModalPlate::configure(Real a, Real b, Real D, Real rhoH, Real nu, Real zeta, int M, int N) {
    a_ = a; b_ = b; D_ = D; rhoH_ = rhoH; nu_ = nu; zeta_ = zeta;
    M_ = std::max(1, M);
    N_ = std::max(1, N);
    const std::size_t n = static_cast<std::size_t>(M_) * N_;
    q_.assign(n, 0);
    v_.assign(n, 0);
    f_.assign(n, 0);
    omega_.resize(n);
    modalMass_ = rhoH_ * a_ * b_ / 4;
    const Real c = std::sqrt(D_ / rhoH_);
    for (int m = 1; m <= M_; ++m)
        for (int k = 1; k <= N_; ++k)
            omega_[(m - 1) * N_ + (k - 1)] = kPi * kPi * (sq(m / a_) + sq(k / b_)) * c;
    // Lanczos sigma factors suppress Gibbs ringing in the curvature.
    sigX_.resize(M_ + 1);
    sigY_.resize(N_ + 1);
    for (int m = 0; m <= M_; ++m) {
        const Real t = kPi * m / (M_ + 1);
        sigX_[m] = m == 0 ? 1 : std::sin(t) / t;
    }
    for (int k = 0; k <= N_; ++k) {
        const Real t = kPi * k / (N_ + 1);
        sigY_[k] = k == 0 ? 1 : std::sin(t) / t;
    }
    resolution_ = kResolutionFactor * std::max(a_ / M_, b_ / N_);
    cacheDt_ = -1;
    halfDt_ = -1;
    sb_.reset();
    sIdx_.clear();
    sFlag_.assign(n, 0);
    sEnergy_ = 0;
    ++rev_;
    sx_.resize(M_ + 1); cx_.resize(M_ + 1); sy_.resize(N_ + 1); cy_.resize(N_ + 1);
}

void ModalPlate::reset() {
    ++rev_;
    sClock_ = -1;
    sEnergy_ = 0;
    std::fill(q_.begin(), q_.end(), 0.0);
    std::fill(v_.begin(), v_.end(), 0.0);
    std::fill(f_.begin(), f_.end(), 0.0);
}

void ModalPlate::clearForces() { std::fill(f_.begin(), f_.end(), 0.0); }

void ModalPlate::addPatch(Real x, Real y, Real radius, Real force) {
    if (force == 0) return;
    basis(x, y, sx_, cx_, sy_, cy_);
    // The disc factors only change with the radius; reuse them within 2 %.
    if (patchR_ < 0 || std::fabs(radius - patchR_) > 0.02 * patchR_) {
        patchR_ = radius;
        patch_.resize(q_.size());
        for (int m = 1; m <= M_; ++m)
            for (int k = 1; k <= N_; ++k) {
                const Real kr = kPi * std::sqrt(sq(m / a_) + sq(k / b_)) * radius;
                patch_[(m - 1) * N_ + (k - 1)] = kr < 1e-6 ? 1.0 : 2 * besselJ1(kr) / kr;
            }
    }
    const Real s = force / modalMass_;
    for (int m = 1; m <= M_; ++m) {
        const Real sm = s * sx_[m];
        Real* f = &f_[(m - 1) * N_];
        const Real* pf = &patch_[(m - 1) * N_];
        for (int k = 1; k <= N_; ++k) f[k - 1] += sm * sy_[k] * pf[k - 1];
    }
}

void ModalPlate::addPressure(Real p) {
    if (p == 0) return;
    for (int m = 1; m <= M_; m += 2)
        for (int k = 1; k <= N_; k += 2)
            f_[(m - 1) * N_ + (k - 1)] += p * 4 * a_ * b_ / (kPi * kPi * m * k) / modalMass_;
}

void ModalPlate::settleStatic() {
    ++rev_;
    sClock_ = -1;
    for (std::size_t i = 0; i < q_.size(); ++i) {
        q_[i] = f_[i] / sq(omega_[i]);
        v_[i] = 0;
    }
    if (stretching()) settleStretched();
}

namespace {
// Exact step of each damped mode for a force held over dt.
void exactStep(const std::vector<Real>& omega, Real zeta, Real dt, std::vector<Real>& c11, std::vector<Real>& c12,
               std::vector<Real>& c21, std::vector<Real>& c22, std::vector<Real>& g1, std::vector<Real>& g2) {
    const std::size_t n = omega.size();
    c11.resize(n); c12.resize(n); c21.resize(n); c22.resize(n); g1.resize(n); g2.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Real w = omega[i];
        const Real sig = zeta * w;
        const Real wd = w * std::sqrt(std::max(1e-9, 1 - zeta * zeta));
        const Real E = std::exp(-sig * dt), C = std::cos(wd * dt), S = std::sin(wd * dt);
        c11[i] = E * (C + sig * S / wd);
        c12[i] = E * S / wd;
        c21[i] = -E * w * w * S / wd;
        c22[i] = E * (C - sig * S / wd);
        g1[i] = (1 - c11[i]) / (w * w);
        g2[i] = c12[i];
    }
}
}  // namespace

void ModalPlate::coefficients(Real dt) {
    if (dt == cacheDt_) return;
    cacheDt_ = dt;
    exactStep(omega_, zeta_, dt, c11_, c12_, c21_, c22_, g1_, g2_);
}

void ModalPlate::step(Real dt) {
    if (dt <= 0) return;
    coefficients(dt);
    ++rev_;
    const bool nl = stretching();
    for (std::size_t i = 0; i < q_.size(); ++i) {
        if (nl && sFlag_[i]) continue;
        const Real q = q_[i], v = v_[i], f = f_[i];
        q_[i] = c11_[i] * q + c12_[i] * v + g1_[i] * f;
        v_[i] = c21_[i] * q + c22_[i] * v + g2_[i] * f;
    }
    if (!nl) {
        sEnergy_ = 0;
        sClock_ = -1;
        return;
    }
    // Stretching: the modes that stretch drift exactly between kicks of the
    // membrane force, each kick standing for the half intervals either side
    // of it (Strang's split; the kick is the gradient of the membrane
    // energy, so the split keeps its energy).
    if (sClock_ < 0) sClock_ = sCarry_ = 0.5 * kickInterval();
    Real left = dt;
    while (sClock_ <= left) {
        drift(sClock_);
        left -= sClock_;
        stretchForce(sF_);
        const Real next = kickInterval();
        const Real weight = sCarry_ + 0.5 * next;
        for (std::size_t k = 0; k < sIdx_.size(); ++k) v_[sIdx_[k]] -= weight * sF_[k] / modalMass_;
        sClock_ = next;
        sCarry_ = 0.5 * next;
    }
    drift(left);
    sClock_ -= left;
}

void ModalPlate::drift(Real t) {
    if (t <= 0) return;
    if (t != halfDt_) {
        halfDt_ = t;
        std::vector<Real> om(sIdx_.size());
        for (std::size_t k = 0; k < sIdx_.size(); ++k) om[k] = omega_[sIdx_[k]];
        exactStep(om, zeta_, t, hc11_, hc12_, hc21_, hc22_, hg1_, hg2_);
    }
    for (std::size_t k = 0; k < sIdx_.size(); ++k) {
        const int i = sIdx_[k];
        const Real q = q_[i], v = v_[i], f = f_[i];
        q_[i] = hc11_[k] * q + hc12_[k] * v + hg1_[k] * f;
        v_[i] = hc21_[k] * q + hc22_[k] * v + hg2_[k] * f;
    }
}

// ---- Membrane action -------------------------------------------------------------

void ModalPlate::configureStretch(Real A, Real nu, Real h, int modes) {
    sClock_ = -1;
    sb_.reset();
    sIdx_.clear();
    sFlag_.assign(q_.size(), 0);
    sEnergy_ = 0;
    halfDt_ = -1;
    ++rev_;
    if (A <= 0 || h <= 0 || modes <= 0) return;
    // The lowest modes stretch the pane; higher ones ride on it (their
    // amplitudes are far below the thickness, and with them in the sum the
    // stretch is noise). A box of about `modes` of them, as many half-waves
    // per metre each way: the corners, where a stretched pane is most
    // stressed, need high orders both ways at once.
    const int bx = std::clamp(static_cast<int>(std::lround(std::sqrt(modes * a_ / b_))), 1, M_);
    const int by = std::clamp(static_cast<int>(std::lround(bx * b_ / a_)), 1, N_);
    sBoxX_ = bx;
    sBoxY_ = by;
    std::vector<int> order;
    for (int m = 1; m <= bx; ++m)
        for (int n = 1; n <= by; ++n) order.push_back((m - 1) * N_ + (n - 1));
    std::vector<int> mm, nn;
    sKappaMax_ = sOmegaMax_ = 0;
    for (int i : order) {
        const int m = i / N_ + 1, n = i % N_ + 1;
        mm.push_back(m);
        nn.push_back(n);
        sIdx_.push_back(i);
        sFlag_[i] = 1;
        sKappaMax_ = std::max(sKappaMax_, kPi * kPi * (sq(m / a_) + sq(n / b_)));
        sOmegaMax_ = std::max(sOmegaMax_, omega_[i]);
    }
    sb_ = stretchBasis(b_ / a_, nu, mm, nn, kStretchDegree);
    sA_ = A;
    sThick_ = h;
    sScale_ = A / (a_ * a_);
    const int K = static_cast<int>(sIdx_.size());
    sPair_.assign(static_cast<std::size_t>(K) * K, 0);
    for (int j = 0, r = 0; j < K; ++j)
        for (int k = j; k < K; ++k, ++r) sPair_[j * K + k] = sPair_[k * K + j] = r;
    sP_.resize(sb_->pairs);
    sY_.resize(sb_->pairs);
    sF_.resize(K);
}

bool ModalPlate::stretching() const {
    if (!sb_) return false;
    // Each mode's swing, not where it is in its cycle.
    Real amp = 0;
    for (int i : sIdx_) amp += std::sqrt(sq(q_[i]) + sq(v_[i] / omega_[i]));
    return amp >= kStretchGate * sThick_;
}

void ModalPlate::stretchForce(std::vector<Real>& f, std::vector<Real>* H) const {
    const int K = static_cast<int>(sIdx_.size());
    for (int j = 0, r = 0; j < K; ++j)
        for (int k = j; k < K; ++k, ++r) sP_[r] = q_[sIdx_[j]] * q_[sIdx_[k]];
    Real U = 0;
    for (const auto& bl : sb_->blocks) {
        const int np = static_cast<int>(bl.pairs.size());
        sBp_.resize(np);
        for (int a = 0; a < np; ++a) sBp_[a] = sP_[bl.pairs[a]];
        // y = G p as a sum of G's columns (G is symmetric): every y_a adds
        // its terms in the same order, so the loop vectorises and stays
        // deterministic.
        sBy_.assign(np, 0.0);
        Real* y = sBy_.data();
        for (int b = 0; b < np; ++b) {
            const Real pv = sBp_[b];
            if (pv == 0) continue;
            const Real* g = &bl.G[static_cast<std::size_t>(b) * np];
            for (int a = 0; a < np; ++a) y[a] += g[a] * pv;
        }
        for (int a = 0; a < np; ++a) {
            sY_[bl.pairs[a]] = y[a];
            U += sBp_[a] * y[a];
        }
    }
    sEnergy_ = 0.5 * sScale_ * U;
    // dU/dq_i = sum_r y_r dp_r/dq_i: p_jk = q_j q_k.
    f.assign(K, 0.0);
    for (int i = 0; i < K; ++i) {
        Real s = 0;
        for (int k = 0; k < K; ++k) s += sY_[sPair_[i * K + k]] * q_[sIdx_[k]];
        f[i] = sScale_ * (s + sY_[sPair_[i * K + i]] * q_[sIdx_[i]]);
    }
    if (!H) return;
    // d2U/dq_i dq_l = D^T G D + (y_il, 2 y_ii on the diagonal), D = dp/dq.
    auto d = [&](int i, int k) { return k == i ? 2 * q_[sIdx_[i]] : q_[sIdx_[k]]; };
    // Each pair's place in its class block.
    std::vector<int> cls(sb_->pairs), pos(sb_->pairs);
    for (int c = 0; c < static_cast<int>(sb_->blocks.size()); ++c)
        for (int a = 0; a < static_cast<int>(sb_->blocks[c].pairs.size()); ++a) cls[sb_->blocks[c].pairs[a]] = c, pos[sb_->blocks[c].pairs[a]] = a;
    std::vector<Real> GD(static_cast<std::size_t>(sb_->pairs) * K, 0.0);
    for (int l = 0; l < K; ++l)
        for (int k = 0; k < K; ++k) {
            const Real dl = d(l, k);
            const int r = sPair_[l * K + k];
            const auto& bl = sb_->blocks[cls[r]];
            const int np = static_cast<int>(bl.pairs.size());
            const Real* g = &bl.G[static_cast<std::size_t>(pos[r]) * np];
            for (int b = 0; b < np; ++b) GD[static_cast<std::size_t>(bl.pairs[b]) * K + l] += g[b] * dl;
        }
    H->assign(static_cast<std::size_t>(K) * K, 0.0);
    for (int i = 0; i < K; ++i)
        for (int l = 0; l < K; ++l) {
            Real s = 0;
            for (int k = 0; k < K; ++k) s += d(i, k) * GD[static_cast<std::size_t>(sPair_[i * K + k]) * K + l];
            s += (i == l ? 2 : 1) * sY_[sPair_[i * K + l]];
            (*H)[static_cast<std::size_t>(i) * K + l] = sScale_ * s;
        }
}

// Static equilibrium of the stretching modes under the held loads, by
// Newton's method with the load raised in steps (the others are linear).
void ModalPlate::settleStretched() {
    const int K = static_cast<int>(sIdx_.size());
    std::vector<Real> target(K), q0(K), F, H, J, r(K), dq(K);
    for (int k = 0; k < K; ++k) target[k] = f_[sIdx_[k]] * modalMass_, q0[k] = 0;
    for (int i : sIdx_) q_[i] = 0;
    auto residual = [&](Real lam, std::vector<Real>& out, std::vector<Real>* Hm) {
        stretchForce(F, Hm);
        Real n2 = 0;
        for (int k = 0; k < K; ++k) {
            const int i = sIdx_[k];
            out[k] = modalMass_ * sq(omega_[i]) * q_[i] + F[k] - lam * target[k];
            n2 += out[k] * out[k];
        }
        return std::sqrt(n2);
    };
    constexpr int kLoadSteps = 8;
    for (int s = 1; s <= kLoadSteps; ++s) {
        const Real lam = static_cast<Real>(s) / kLoadSteps;
        for (int it = 0; it < 60; ++it) {
            const Real r0 = residual(lam, r, &H);
            J = H;
            for (int k = 0; k < K; ++k) J[static_cast<std::size_t>(k) * K + k] += modalMass_ * sq(omega_[sIdx_[k]]);
            // Solve J dq = -r (Gaussian elimination, partial pivoting).
            for (int k = 0; k < K; ++k) dq[k] = -r[k];
            for (int c = 0; c < K; ++c) {
                int piv = c;
                for (int rr = c + 1; rr < K; ++rr)
                    if (std::fabs(J[static_cast<std::size_t>(rr) * K + c]) > std::fabs(J[static_cast<std::size_t>(piv) * K + c])) piv = rr;
                if (piv != c) {
                    for (int cc = 0; cc < K; ++cc) std::swap(J[static_cast<std::size_t>(c) * K + cc], J[static_cast<std::size_t>(piv) * K + cc]);
                    std::swap(dq[c], dq[piv]);
                }
                const Real pv = J[static_cast<std::size_t>(c) * K + c];
                for (int rr = c + 1; rr < K; ++rr) {
                    const Real l = J[static_cast<std::size_t>(rr) * K + c] / pv;
                    if (l == 0) continue;
                    for (int cc = c; cc < K; ++cc) J[static_cast<std::size_t>(rr) * K + cc] -= l * J[static_cast<std::size_t>(c) * K + cc];
                    dq[rr] -= l * dq[c];
                }
            }
            for (int c = K - 1; c >= 0; --c) {
                Real v = dq[c];
                for (int cc = c + 1; cc < K; ++cc) v -= J[static_cast<std::size_t>(c) * K + cc] * dq[cc];
                dq[c] = v / J[static_cast<std::size_t>(c) * K + c];
            }
            // Damped: halve until the residual falls.
            for (int k = 0; k < K; ++k) q0[k] = q_[sIdx_[k]];
            Real step = 1, r1 = r0;
            for (int ls = 0; ls < 20; ++ls, step *= 0.5) {
                for (int k = 0; k < K; ++k) q_[sIdx_[k]] = q0[k] + step * dq[k];
                r1 = residual(lam, r, nullptr);
                if (r1 < r0) break;
            }
            Real big = 0;
            for (int k = 0; k < K; ++k) big = std::max(big, std::fabs(step * dq[k]));
            if (big < 1e-12 * sThick_ || r1 >= r0) break;
        }
    }
    for (int i : sIdx_) v_[i] = 0;
    stretchForce(F);  // energy of the settled state
    ++rev_;
}

Real ModalPlate::stretchStep() const { return stretching() ? kickInterval() : 1e9; }

Real ModalPlate::kickInterval() const {
    Real q2 = 0;
    for (int i : sIdx_) q2 += sq(q_[i]);
    // The membrane's own stiffness: of the stretch itself (a quartic energy
    // has U'' = 12 U / q^2) and of the tension it puts on the highest mode.
    const Real U = sEnergy_;
    const Real tension = std::sqrt(2 * sA_ * U / (a_ * b_));
    const Real k = std::max(12 * U / std::max(q2, 1e-30), tension * sKappaMax_ * a_ * b_ / 4);
    const Real wnl = std::sqrt(k / modalMass_);
    return std::min(wnl > 0 ? kStretchKick / wnl : 1e9, kStretchSplit / sOmegaMax_);
}

void ModalPlate::inPlane() const {
    if (sURev_ == rev_ && !sU_.empty()) return;
    sURev_ = rev_;
    const int K = static_cast<int>(sIdx_.size());
    for (int j = 0, r = 0; j < K; ++j)
        for (int k = j; k < K; ++k, ++r) sP_[r] = q_[sIdx_[j]] * q_[sIdx_[k]] / (a_ * a_);
    sU_.assign(sb_->dofs, 0.0);
    for (const auto& bl : sb_->blocks) {
        const int np = static_cast<int>(bl.pairs.size()), nb = static_cast<int>(bl.dofs.size());
        // u = -X p, column by column (X is held pairs x dofs).
        sBy_.assign(nb, 0.0);
        Real* u = sBy_.data();
        for (int b = 0; b < np; ++b) {
            const Real pv = sP_[bl.pairs[b]];
            if (pv == 0) continue;
            const Real* x = &bl.Xt[static_cast<std::size_t>(b) * nb];
            for (int d = 0; d < nb; ++d) u[d] -= x[d] * pv;
        }
        for (int d = 0; d < nb; ++d) sU_[bl.dofs[d]] = u[d];
    }
}

PlaneStrain ModalPlate::membraneStrain(Real x, Real y) const {
    PlaneStrain e;
    if (!stretching()) return e;
    inPlane();
    // The bent shape's own share: w,x^2 / 2, w,y^2 / 2, w,x w,y.
    Real wx = 0, wy = 0;
    for (std::size_t k = 0; k < sIdx_.size(); ++k) {
        const Real m = sb_->mm[k], n = sb_->nn[k], q = q_[sIdx_[k]];
        const Real kx = m * kPi / a_, ky = n * kPi / b_;
        wx += q * kx * std::cos(kx * x) * std::sin(ky * y);
        wy += q * ky * std::sin(kx * x) * std::cos(ky * y);
    }
    e.exx = 0.5 * wx * wx;
    e.eyy = 0.5 * wy * wy;
    e.gxy = wx * wy;
    // The in-plane displacement's share.
    const int D = sb_->degree + 1;
    sL_.resize(D); sdL_.resize(D); sLy_.resize(D); sdLy_.resize(D);
    stretchLegendre(sb_->degree, clampr(x / a_, 0, 1), sL_.data(), sdL_.data());
    stretchLegendre(sb_->degree, clampr(y / b_, 0, 1), sLy_.data(), sdLy_.data());
    const Real ib = 1 / sb_->beta;
    for (int d = 0; d < sb_->dofs; ++d) {
        const int i = sb_->dofI[d], j = sb_->dofJ[d];
        const Real u = sU_[d];
        if (!sb_->dofV[d]) {
            e.exx += u * sdL_[i] * sLy_[j];
            e.gxy += u * sL_[i] * sdLy_[j] * ib;
        } else {
            e.eyy += u * sL_[i] * sdLy_[j] * ib;
            e.gxy += u * sdL_[i] * sLy_[j];
        }
    }
    return e;
}

void ModalPlate::basis(Real x, Real y, std::vector<Real>& sx, std::vector<Real>& cx, std::vector<Real>& sy,
                       std::vector<Real>& cy) const {
    // sin(m t), cos(m t) by the angle-addition recurrence.
    auto fill = [](Real t, int n, std::vector<Real>& s, std::vector<Real>& c) {
        const Real s1 = std::sin(t), c1 = std::cos(t);
        s[0] = 0; c[0] = 1;
        for (int m = 1; m <= n; ++m) {
            s[m] = s[m - 1] * c1 + c[m - 1] * s1;
            c[m] = c[m - 1] * c1 - s[m - 1] * s1;
        }
    };
    fill(kPi * x / a_, M_, sx, cx);
    fill(kPi * y / b_, N_, sy, cy);
}

Real ModalPlate::deflection(Real x, Real y) const {
    basis(x, y, sx_, cx_, sy_, cy_);
    Real w = 0;
    for (int m = 1; m <= M_; ++m) {
        Real inner = 0;
        const Real* q = &q_[(m - 1) * N_];
        for (int k = 1; k <= N_; ++k) inner += q[k - 1] * sy_[k];
        w += sx_[m] * inner;
    }
    return w;
}

Real ModalPlate::velocity(Real x, Real y) const {
    basis(x, y, sx_, cx_, sy_, cy_);
    Real w = 0;
    for (int m = 1; m <= M_; ++m) {
        Real inner = 0;
        const Real* v = &v_[(m - 1) * N_];
        for (int k = 1; k <= N_; ++k) inner += v[k - 1] * sy_[k];
        w += sx_[m] * inner;
    }
    return w;
}

Vec2 ModalPlate::velocityGradient(Real x, Real y) const {
    basis(x, y, sx_, cx_, sy_, cy_);
    Real gx = 0, gy = 0;
    for (int m = 1; m <= M_; ++m) {
        Real is = 0, ic = 0;
        const Real* v = &v_[(m - 1) * N_];
        for (int k = 1; k <= N_; ++k) {
            is += v[k - 1] * sy_[k];
            ic += v[k - 1] * (k * kPi / b_) * cy_[k];
        }
        gx += (m * kPi / a_) * cx_[m] * is;
        gy += sx_[m] * ic;
    }
    return {gx, gy};
}

Curvature ModalPlate::curvature(Real x, Real y) const {
    basis(x, y, sx_, cx_, sy_, cy_);
    Real wxx = 0, wyy = 0, wxy = 0;
    for (int m = 1; m <= M_; ++m) {
        Real a1 = 0, a2 = 0, a3 = 0;
        const Real* q = &q_[(m - 1) * N_];
        for (int k = 1; k <= N_; ++k) {
            const Real qs = q[k - 1] * sigY_[k];
            const Real kn = k * kPi / b_;
            a1 += qs * sy_[k];
            a2 += qs * kn * kn * sy_[k];
            a3 += qs * kn * cy_[k];
        }
        const Real km = m * kPi / a_;
        const Real sm = sigX_[m];
        wxx -= km * km * sm * sx_[m] * a1;
        wyy -= sm * sx_[m] * a2;
        wxy += km * sm * cx_[m] * a3;
    }
    return {-wxx, -wyy, -wxy};
}

Real ModalPlate::curvatureBound() const {
    Real K = 0;
    for (int m = 1; m <= M_; ++m) {
        const Real km = m * kPi / a_;
        for (int k = 1; k <= N_; ++k) {
            const Real kn = k * kPi / b_;
            K += std::fabs(q_[(m - 1) * N_ + k - 1] * sigX_[m] * sigY_[k]) * (km * km + kn * kn);
        }
    }
    return K;
}

void ModalPlate::sampleGrid(int nx, int ny, Real* w, Real* wdot, Curvature* k, PlaneStrain* e) const {
    if (nx < 2 || ny < 2) return;
    if (e) {
        if (!stretching()) std::fill(e, e + static_cast<std::size_t>(nx) * ny, PlaneStrain{});
        else {
            inPlane();
            // Separable: Legendre tables along each side of the lattice.
            const int D = sb_->degree + 1;
            std::vector<Real> Lx(static_cast<std::size_t>(nx) * D), dLx(Lx.size()), Ly(static_cast<std::size_t>(ny) * D), dLy(Ly.size());
            for (int i = 0; i < nx; ++i) stretchLegendre(sb_->degree, static_cast<Real>(i) / (nx - 1), &Lx[static_cast<std::size_t>(i) * D], &dLx[static_cast<std::size_t>(i) * D]);
            for (int j = 0; j < ny; ++j) stretchLegendre(sb_->degree, static_cast<Real>(j) / (ny - 1), &Ly[static_cast<std::size_t>(j) * D], &dLy[static_cast<std::size_t>(j) * D]);
            std::vector<Real> Uu(static_cast<std::size_t>(D) * D, 0.0), Uv(Uu.size(), 0.0);
            for (int d = 0; d < sb_->dofs; ++d) (sb_->dofV[d] ? Uv : Uu)[static_cast<std::size_t>(sb_->dofI[d]) * D + sb_->dofJ[d]] = sU_[d];
            // T[i][j] = sum over y degree of U[i][.] times the y table at row j.
            std::vector<Real> Tu0(static_cast<std::size_t>(D) * ny), Tu1(Tu0.size()), Tv0(Tu0.size()), Tv1(Tu0.size());
            for (int i = 0; i < D; ++i)
                for (int j = 0; j < ny; ++j) {
                    Real a0 = 0, a1 = 0, b0 = 0, b1 = 0;
                    for (int l = 0; l < D; ++l) {
                        const Real ly = Ly[static_cast<std::size_t>(j) * D + l], dly = dLy[static_cast<std::size_t>(j) * D + l];
                        a0 += Uu[static_cast<std::size_t>(i) * D + l] * ly;
                        a1 += Uu[static_cast<std::size_t>(i) * D + l] * dly;
                        b0 += Uv[static_cast<std::size_t>(i) * D + l] * ly;
                        b1 += Uv[static_cast<std::size_t>(i) * D + l] * dly;
                    }
                    const std::size_t o = static_cast<std::size_t>(i) * ny + j;
                    Tu0[o] = a0, Tu1[o] = a1, Tv0[o] = b0, Tv1[o] = b1;
                }
            const Real ib = 1 / sb_->beta;
            // Slopes of the stretching modes, separably: per x wavenumber m,
            // Sx[m][j] = sum_n q_mn (m pi / a) sin(n pi y_j / b) and
            // Sy[m][j] = sum_n q_mn (n pi / b) cos(n pi y_j / b).
            int Ms = 0;
            for (int m : sb_->mm) Ms = std::max(Ms, m);
            std::vector<Real> Sx(static_cast<std::size_t>(Ms + 1) * ny, 0.0), Sy(Sx.size(), 0.0);
            for (std::size_t c = 0; c < sIdx_.size(); ++c) {
                const int m = sb_->mm[c], n = sb_->nn[c];
                const Real q = q_[sIdx_[c]], kx = m * kPi / a_, ky = n * kPi / b_;
                for (int j = 0; j < ny; ++j) {
                    const Real t = n * kPi * j / (ny - 1);
                    Sx[static_cast<std::size_t>(m) * ny + j] += q * kx * std::sin(t);
                    Sy[static_cast<std::size_t>(m) * ny + j] += q * ky * std::cos(t);
                }
            }
            std::vector<Real> cxm(static_cast<std::size_t>(nx) * (Ms + 1)), sxm(cxm.size());
            for (int i = 0; i < nx; ++i)
                for (int m = 0; m <= Ms; ++m) {
                    const Real t = m * kPi * i / (nx - 1);
                    cxm[static_cast<std::size_t>(i) * (Ms + 1) + m] = std::cos(t);
                    sxm[static_cast<std::size_t>(i) * (Ms + 1) + m] = std::sin(t);
                }
            for (int j = 0; j < ny; ++j)
                for (int i = 0; i < nx; ++i) {
                    Real wx = 0, wy = 0;
                    for (int m = 1; m <= Ms; ++m) {
                        wx += cxm[static_cast<std::size_t>(i) * (Ms + 1) + m] * Sx[static_cast<std::size_t>(m) * ny + j];
                        wy += sxm[static_cast<std::size_t>(i) * (Ms + 1) + m] * Sy[static_cast<std::size_t>(m) * ny + j];
                    }
                    PlaneStrain s{0.5 * wx * wx, 0.5 * wy * wy, wx * wy};
                    for (int l = 0; l < D; ++l) {
                        const Real lx = Lx[static_cast<std::size_t>(i) * D + l], dlx = dLx[static_cast<std::size_t>(i) * D + l];
                        const std::size_t o = static_cast<std::size_t>(l) * ny + j;
                        s.exx += dlx * Tu0[o];
                        s.eyy += lx * Tv1[o] * ib;
                        s.gxy += lx * Tu1[o] * ib + dlx * Tv0[o];
                    }
                    e[static_cast<std::size_t>(j) * nx + i] = s;
                }
        }
    }
    // Separable evaluation: T[m][j] = sum_n q[m][n] * (row basis at y_j), then
    // a sum over m per lattice point. Basis tables are kept between calls.
    if (nx != gNx_ || ny != gNy_ || gSX_.size() != static_cast<std::size_t>(nx) * (M_ + 1) ||
        gSY_.size() != static_cast<std::size_t>(ny) * (N_ + 1)) {
        gNx_ = nx;
        gNy_ = ny;
        gSX_.resize(static_cast<std::size_t>(nx) * (M_ + 1));
        gCX_.resize(gSX_.size());
        gSY_.resize(static_cast<std::size_t>(ny) * (N_ + 1));
        gCY_.resize(gSY_.size());
        for (int i = 0; i < nx; ++i) {
            const Real t = kPi * i / (nx - 1);
            for (int m = 0; m <= M_; ++m) {
                gSX_[i * (M_ + 1) + m] = std::sin(m * t);
                gCX_[i * (M_ + 1) + m] = std::cos(m * t);
            }
        }
        for (int j = 0; j < ny; ++j) {
            const Real t = kPi * j / (ny - 1);
            for (int n = 0; n <= N_; ++n) {
                gSY_[j * (N_ + 1) + n] = std::sin(n * t);
                gCY_[j * (N_ + 1) + n] = std::cos(n * t);
            }
        }
    }
    const std::size_t T = static_cast<std::size_t>(M_) * ny;
    for (auto& t : gT_) t.resize(T);
    Real *Tw = gT_[0].data(), *Tv = gT_[1].data(), *T1 = gT_[2].data(), *T2 = gT_[3].data(), *T3 = gT_[4].data();
    for (int m = 1; m <= M_; ++m) {
        const Real* q = &q_[(m - 1) * N_];
        const Real* v = &v_[(m - 1) * N_];
        for (int j = 0; j < ny; ++j) {
            const Real* sy = &gSY_[j * (N_ + 1)];
            const Real* cy = &gCY_[j * (N_ + 1)];
            Real aw = 0, av = 0, a1 = 0, a2 = 0, a3 = 0;
            for (int n = 1; n <= N_; ++n) {
                if (w) aw += q[n - 1] * sy[n];
                if (wdot) av += v[n - 1] * sy[n];
                if (k) {
                    const Real qs = q[n - 1] * sigY_[n];
                    const Real kn = n * kPi / b_;
                    a1 += qs * sy[n];
                    a2 += qs * kn * kn * sy[n];
                    a3 += qs * kn * cy[n];
                }
            }
            const std::size_t idx = static_cast<std::size_t>(m - 1) * ny + j;
            Tw[idx] = aw;
            Tv[idx] = av;
            T1[idx] = a1;
            T2[idx] = a2;
            T3[idx] = a3;
        }
    }
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const Real* sx = &gSX_[i * (M_ + 1)];
            const Real* cx = &gCX_[i * (M_ + 1)];
            Real ww = 0, vv = 0, wxx = 0, wyy = 0, wxy = 0;
            for (int m = 1; m <= M_; ++m) {
                const std::size_t idx = static_cast<std::size_t>(m - 1) * ny + j;
                if (w) ww += sx[m] * Tw[idx];
                if (wdot) vv += sx[m] * Tv[idx];
                if (k) {
                    const Real km = m * kPi / a_;
                    const Real sm = sigX_[m] * sx[m];
                    wxx -= km * km * sm * T1[idx];
                    wyy -= sm * T2[idx];
                    wxy += km * sigX_[m] * cx[m] * T3[idx];
                }
            }
            const std::size_t o = static_cast<std::size_t>(j) * nx + i;
            if (w) w[o] = ww;
            if (wdot) wdot[o] = vv;
            if (k) k[o] = {-wxx, -wyy, -wxy};
        }
    }
}

Real ModalPlate::nearField(Real force, Real r, Real re, Real loadTime) const {
    const Real rr = std::max(r, re);
    const Real reach = std::min(resolution_, std::sqrt(std::max(loadTime, 0.0) * std::sqrt(D_ / rhoH_)));
    if (rr >= reach || force == 0) return 0;
    return force / (4 * kPi * D_) * std::log(reach / rr);
}

Real ModalPlate::energy() const {
    Real e = 0;
    for (std::size_t i = 0; i < q_.size(); ++i) e += 0.5 * modalMass_ * (v_[i] * v_[i] + sq(omega_[i] * q_[i]));
    return e + sEnergy_;
}

Real ModalPlate::maxFrequencyHz() const { return omega_.empty() ? 0 : omega_.back() / (2 * kPi); }
Real ModalPlate::fundamentalHz() const { return omega_.empty() ? 0 : omega_.front() / (2 * kPi); }

}  // namespace gls
