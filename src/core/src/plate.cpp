// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/plate.hpp"

#include <cmath>

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
    sx_.resize(M_ + 1); cx_.resize(M_ + 1); sy_.resize(N_ + 1); cy_.resize(N_ + 1);
}

void ModalPlate::reset() {
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
    for (std::size_t i = 0; i < q_.size(); ++i) {
        q_[i] = f_[i] / sq(omega_[i]);
        v_[i] = 0;
    }
}

void ModalPlate::coefficients(Real dt) {
    if (dt == cacheDt_) return;
    cacheDt_ = dt;
    const std::size_t n = q_.size();
    c11_.resize(n); c12_.resize(n); c21_.resize(n); c22_.resize(n); g1_.resize(n); g2_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Real w = omega_[i];
        const Real sig = zeta_ * w;
        const Real wd = w * std::sqrt(std::max(1e-9, 1 - zeta_ * zeta_));
        const Real E = std::exp(-sig * dt), C = std::cos(wd * dt), S = std::sin(wd * dt);
        c11_[i] = E * (C + sig * S / wd);
        c12_[i] = E * S / wd;
        c21_[i] = -E * w * w * S / wd;
        c22_[i] = E * (C - sig * S / wd);
        g1_[i] = (1 - c11_[i]) / (w * w);
        g2_[i] = c12_[i];
    }
}

void ModalPlate::step(Real dt) {
    if (dt <= 0) return;
    coefficients(dt);
    for (std::size_t i = 0; i < q_.size(); ++i) {
        const Real q = q_[i], v = v_[i], f = f_[i];
        q_[i] = c11_[i] * q + c12_[i] * v + g1_[i] * f;
        v_[i] = c21_[i] * q + c22_[i] * v + g2_[i] * f;
    }
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

void ModalPlate::sampleGrid(int nx, int ny, Real* w, Real* wdot, Curvature* k) const {
    if (nx < 2 || ny < 2) return;
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
    return e;
}

Real ModalPlate::maxFrequencyHz() const { return omega_.empty() ? 0 : omega_.back() / (2 * kPi); }
Real ModalPlate::fundamentalHz() const { return omega_.empty() ? 0 : omega_.front() / (2 * kPi); }

}  // namespace gls
