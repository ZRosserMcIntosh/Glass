// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Membrane action of the plate (von Kármán). The mid-surface strain is
//   e = (u,x + w,x²/2,  v,y + w,y²/2,  u,y + v,x + w,x w,y)
// with the in-plane displacement (u, v) in equilibrium for the bent shape
// (in-plane waves cross a pane in a fraction of a millisecond). The edges are
// free in the plane of the pane (N_n = N_nt = 0: glass sliding in its
// gasket, as in the analysis behind ASTM E1300). (u, v) is a tensor
// Legendre series; minimising the membrane energy over it leaves, for w a sum
// of stretching modes q_k psi_k, an energy quartic in q:
//   U = 1/2 p^T G p,   p_jk = q_j q_k (j <= k).
// Everything separates into one-dimensional integrals of Legendre
// polynomials and sines, so the tables cost milliseconds; they depend only on
// the aspect ratio, Poisson's ratio and the mode set, and are shared.
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#if !(defined(__wasi__) && !defined(_REENTRANT))
#include <mutex>
#define GLS_STRETCH_LOCK 1
#endif

#include "gls/plate.hpp"

namespace gls {

namespace {

// Gauss-Legendre nodes and weights on [0, 1].
void gauss(int n, std::vector<Real>& x, std::vector<Real>& w) {
    x.resize(n);
    w.resize(n);
    for (int i = 0; i < n; ++i) {
        Real z = std::cos(kPi * (i + 0.75) / (n + 0.5)), dp = 1;
        for (int it = 0; it < 100; ++it) {
            Real p0 = 1, p1 = z;
            for (int k = 2; k <= n; ++k) {
                const Real p2 = ((2 * k - 1) * z * p1 - (k - 1) * p0) / k;
                p0 = p1;
                p1 = p2;
            }
            dp = n * (z * p1 - p0) / (z * z - 1);
            const Real dz = p1 / dp;
            z -= dz;
            if (std::fabs(dz) < 1e-15) break;
        }
        x[i] = 0.5 * (1 - z);
        w[i] = 1 / ((1 - z * z) * dp * dp);
    }
}

// Normalised shifted Legendre polynomials on [0, 1] and their t-derivatives.
void legendre(int deg, Real t, Real* L, Real* dL) {
    const Real s = 2 * t - 1;
    Real p0 = 1, p1 = s, d0 = 0, d1 = 1;
    for (int k = 0; k <= deg; ++k) {
        Real p, d;
        if (k == 0) p = 1, d = 0;
        else if (k == 1) p = s, d = 1;
        else {
            p = ((2 * k - 1) * s * p1 - (k - 1) * p0) / k;
            d = d0 + (2 * k - 1) * p1;
            p0 = p1, p1 = p, d0 = d1, d1 = d;
        }
        const Real norm = std::sqrt(2.0 * k + 1);
        L[k] = norm * p;
        dL[k] = norm * 2 * d;
    }
}

constexpr int kSin = 0, kCos = 1;

// One separable piece of a pair's strain: c g1(x) g2(x) h1(y) h2(y), sines
// and cosines of the modes' half-wavenumbers (x on [0, 1], y on [0, 1] of b).
struct Term {
    Real c = 0;
    int gx1, mx1, gx2, mx2, gy1, ny1, gy2, ny2;
};

}  // namespace

std::shared_ptr<const StretchBasis> stretchBasis(Real beta, Real nu, const std::vector<int>& mm,
                                                  const std::vector<int>& nn, int degree) {
    using Key = std::tuple<Real, Real, std::vector<int>, std::vector<int>, int>;
    static std::map<Key, std::shared_ptr<const StretchBasis>> cache;
#ifdef GLS_STRETCH_LOCK
    static std::mutex lock;  // hosts may build panes on several threads (not on single-threaded wasm)
#endif
    const Key key{beta, nu, mm, nn, degree};
    {
#ifdef GLS_STRETCH_LOCK
        std::lock_guard<std::mutex> g(lock);
#endif
        auto it = cache.find(key);
        if (it != cache.end()) return it->second;
    }

    auto B = std::make_shared<StretchBasis>();
    B->beta = beta;
    B->nu = nu;
    B->degree = degree;
    B->mm = mm;
    B->nn = nn;
    const int K = static_cast<int>(mm.size());
    const int P = K * (K + 1) / 2;
    B->pairs = P;
    int Mx = 1, Ny = 1;
    for (int k = 0; k < K; ++k) Mx = std::max(Mx, mm[k]), Ny = std::max(Ny, nn[k]);
    const int D = degree + 1;

    // ---- One-dimensional tables ----
    const int nq = std::max(48, 4 * std::max(Mx, Ny) + 2 * D + 8);
    std::vector<Real> qx, qw;
    gauss(nq, qx, qw);
    const int maxM = std::max(Mx, Ny);
    // trig[g][m][q]: sin / cos(m pi t) at the nodes, m = 1..maxM.
    std::vector<Real> trig(static_cast<std::size_t>(2) * (maxM + 1) * nq);
    auto tr = [&](int g, int m, int q) -> Real& { return trig[(static_cast<std::size_t>(g) * (maxM + 1) + m) * nq + q]; };
    for (int m = 0; m <= maxM; ++m)
        for (int q = 0; q < nq; ++q) tr(kSin, m, q) = std::sin(m * kPi * qx[q]), tr(kCos, m, q) = std::cos(m * kPi * qx[q]);
    std::vector<Real> Lq(static_cast<std::size_t>(nq) * D), dLq(Lq.size());
    for (int q = 0; q < nq; ++q) legendre(degree, qx[q], &Lq[static_cast<std::size_t>(q) * D], &dLq[static_cast<std::size_t>(q) * D]);
    auto Lv = [&](int d, int q, int i) { return (d ? dLq : Lq)[static_cast<std::size_t>(q) * D + i]; };

    // TL[d][g1][g2][i][m1][m2] = int_0^1 L_i^(d) g1_m1 g2_m2 dt.
    const int S = maxM + 1;
    std::vector<Real> TL(static_cast<std::size_t>(2) * 4 * D * S * S, 0.0);
    auto tl = [&](int d, int g1, int g2, int i, int m1, int m2) -> Real& {
        return TL[((((static_cast<std::size_t>(d) * 4 + g1 * 2 + g2) * D + i) * S + m1) * S + m2)];
    };
    for (int d = 0; d < 2; ++d)
        for (int g1 = 0; g1 < 2; ++g1)
            for (int g2 = 0; g2 < 2; ++g2)
                for (int m1 = 1; m1 <= maxM; ++m1)
                    for (int m2 = 1; m2 <= maxM; ++m2)
                        for (int q = 0; q < nq; ++q) {
                            const Real f = qw[q] * tr(g1, m1, q) * tr(g2, m2, q);
                            for (int i = 0; i < D; ++i) tl(d, g1, g2, i, m1, m2) += f * Lv(d, q, i);
                        }
    // J4: int_0^1 of four trig factors; codes c = g * S + m.
    const int C = 2 * S;
    std::vector<Real> J4(static_cast<std::size_t>(C) * C * C * C, 0.0);
    {
        std::vector<Real> pair(static_cast<std::size_t>(C) * C * nq, 0.0);
        for (int a = 0; a < C; ++a)
            for (int b = 0; b < C; ++b)
                if (a % S && b % S)
                    for (int q = 0; q < nq; ++q)
                        pair[(static_cast<std::size_t>(a) * C + b) * nq + q] = tr(a / S, a % S, q) * tr(b / S, b % S, q);
        for (int a = 0; a < C; ++a)
            for (int b = a; b < C; ++b) {
                if (!(a % S) || !(b % S)) continue;
                const Real* pab = &pair[(static_cast<std::size_t>(a) * C + b) * nq];
                for (int c = 0; c < C; ++c)
                    for (int d = c; d < C; ++d) {
                        if (!(c % S) || !(d % S)) continue;
                        const Real* pcd = &pair[(static_cast<std::size_t>(c) * C + d) * nq];
                        Real s = 0;
                        for (int q = 0; q < nq; ++q) s += qw[q] * pab[q] * pcd[q];
                        for (int x1 : {a, b})
                            for (int x2 : {a, b}) {
                                if ((x1 == x2) != (a == b)) continue;
                                for (int x3 : {c, d})
                                    for (int x4 : {c, d}) {
                                        if ((x3 == x4) != (c == d)) continue;
                                        J4[((static_cast<std::size_t>(x1) * C + x2) * C + x3) * C + x4] = s;
                                    }
                            }
                    }
            }
    }
    auto j4 = [&](int g1, int m1, int g2, int m2, int g3, int m3, int g4, int m4) {
        return J4[((static_cast<std::size_t>(g1 * S + m1) * C + (g2 * S + m2)) * C + (g3 * S + m3)) * C + (g4 * S + m4)];
    };
    // Legendre Gram matrices: M0 = int L L, M1[i][k] = int L_i' L_k, M2 = int L' L'.
    std::vector<Real> M0(static_cast<std::size_t>(D) * D, 0.0), M1(M0.size(), 0.0), M2(M0.size(), 0.0);
    for (int q = 0; q < nq; ++q)
        for (int i = 0; i < D; ++i)
            for (int k = 0; k < D; ++k) {
                M0[i * D + k] += qw[q] * Lv(0, q, i) * Lv(0, q, k);
                M1[i * D + k] += qw[q] * Lv(1, q, i) * Lv(0, q, k);
                M2[i * D + k] += qw[q] * Lv(1, q, i) * Lv(1, q, k);
            }

    // ---- Pair strains as separable terms (coordinates in units of a) ----
    // On y in [0, beta] with t = y / beta: dy = beta dt, d/dy = (1/beta) d/dt.
    std::vector<Term> ex(P), ey(P), g1(P), g2(P);
    for (int a = 0, r = 0; a < K; ++a)
        for (int b = a; b < K; ++b, ++r) {
            const Real f = a == b ? 0.5 : 1.0;  // (2 - delta) / 2
            const Real kxa = mm[a] * kPi, kxb = mm[b] * kPi, kya = nn[a] * kPi / beta, kyb = nn[b] * kPi / beta;
            ex[r] = {f * kxa * kxb, kCos, mm[a], kCos, mm[b], kSin, nn[a], kSin, nn[b]};
            ey[r] = {f * kya * kyb, kSin, mm[a], kSin, mm[b], kCos, nn[a], kCos, nn[b]};
            g1[r] = {f * kxa * kyb, kCos, mm[a], kSin, mm[b], kSin, nn[a], kCos, nn[b]};
            g2[r] = {f * kya * kxb, kSin, mm[a], kCos, mm[b], kCos, nn[a], kSin, nn[b]};
        }
    auto prod = [&](const Term& t, const Term& u) {
        return t.c * u.c * j4(t.gx1, t.mx1, t.gx2, t.mx2, u.gx1, u.mx1, u.gx2, u.mx2) * beta *
               j4(t.gy1, t.ny1, t.gy2, t.ny2, u.gy1, u.ny1, u.gy2, u.ny2);
    };
    // int of a term against L_i^(dx)(x) L_j^(dy)(y), derivatives in units of a.
    auto proj = [&](const Term& t, int i, int dx, int j, int dy) {
        return t.c * tl(dx, t.gx1, t.gx2, i, t.mx1, t.mx2) * tl(dy, t.gy1, t.gy2, j, t.ny1, t.ny2) * (dy ? 1.0 : beta);
    };

    // ---- Symmetry classes ----
    // About the pane's centre lines a pair's strain is even or odd: sin(m pi t)
    // and cos(m pi t) flip as (-1)^(m+1) and (-1)^m under t -> 1 - t, and
    // L_i as (-1)^i. Class (px, py) = parity of (m_a + m_b, n_a + n_b); the
    // membrane energy couples only pairs of one class, and each class has its
    // own in-plane dofs (u odd in x and of parity py in y when px = 0, ...).
    std::vector<int> pairClass(P);
    for (int a = 0, r = 0; a < K; ++a)
        for (int b = a; b < K; ++b, ++r) pairClass[r] = ((mm[a] + mm[b]) & 1) * 2 + ((nn[a] + nn[b]) & 1);
    // In-plane dofs (rigid motions removed: u00, v00, v10).
    for (int comp = 0; comp < 2; ++comp)
        for (int j = 0; j < D; ++j)
            for (int i = 0; i < D; ++i) {
                if (i == 0 && j == 0) continue;
                if (comp == 1 && i == 1 && j == 0) continue;
                B->dofI.push_back(i);
                B->dofJ.push_back(j);
                B->dofV.push_back(static_cast<char>(comp));
            }
    const int nd = static_cast<int>(B->dofI.size());
    B->dofs = nd;
    auto dofClass = [&](int d) {
        const int i = B->dofI[d], j = B->dofJ[d];
        return B->dofV[d] ? (i & 1) * 2 + ((j + 1) & 1) : ((i + 1) & 1) * 2 + (j & 1);
    };
    const Real sh = 0.5 * (1 - nu);
    auto stiffness = [&](int r, int c) {
        const int i = B->dofI[r], j = B->dofJ[r], k = B->dofI[c], l = B->dofJ[c];
        const Real m0x = M0[i * D + k], m2x = M2[i * D + k], m0y = beta * M0[j * D + l], m2y = M2[j * D + l] / beta;
        if (!B->dofV[r] && !B->dofV[c]) return m2x * m0y + sh * m0x * m2y;
        if (B->dofV[r] && B->dofV[c]) return m0x * m2y + sh * m2x * m0y;
        // u dof against v dof: nu u,x v,y + sh u,y v,x.
        int ui = i, uj = j, vi = k, vj = l;
        if (B->dofV[r]) std::swap(ui, vi), std::swap(uj, vj);
        return nu * M1[ui * D + vi] * M1[vj * D + uj] + sh * M1[vi * D + ui] * M1[uj * D + vj];
    };
    // Load of pair r on dof d: int B_d^T C e0_r.
    auto load = [&](int d, int r) {
        const int i = B->dofI[d], j = B->dofJ[d];
        if (!B->dofV[d])  // u: phi,x (ex + nu ey) + phi,y sh g
            return proj(ex[r], i, 1, j, 0) + nu * proj(ey[r], i, 1, j, 0) + sh * (proj(g1[r], i, 0, j, 1) + proj(g2[r], i, 0, j, 1));
        // v: phi,y (ey + nu ex) + phi,x sh g
        return proj(ey[r], i, 0, j, 1) + nu * proj(ex[r], i, 0, j, 1) + sh * (proj(g1[r], i, 1, j, 0) + proj(g2[r], i, 1, j, 0));
    };
    for (int c = 0; c < 4; ++c) {
        StretchBasis::Block bl;
        for (int r = 0; r < P; ++r)
            if (pairClass[r] == c) bl.pairs.push_back(r);
        for (int d = 0; d < nd; ++d)
            if (dofClass(d) == c) bl.dofs.push_back(d);
        const int np = static_cast<int>(bl.pairs.size()), nb = static_cast<int>(bl.dofs.size());
        if (!np) continue;
        // Cholesky of this class's in-plane stiffness.
        std::vector<Real> L(static_cast<std::size_t>(nb) * nb);
        for (int r = 0; r < nb; ++r)
            for (int q = 0; q < nb; ++q) L[static_cast<std::size_t>(r) * nb + q] = stiffness(bl.dofs[r], bl.dofs[q]);
        for (int k = 0; k < nb; ++k) {
            Real* rk = &L[static_cast<std::size_t>(k) * nb];
            Real sum = rk[k];
            for (int t = 0; t < k; ++t) sum -= rk[t] * rk[t];
            const Real dk = std::sqrt(std::max(sum, 1e-300));
            rk[k] = dk;
            for (int r = k + 1; r < nb; ++r) {
                Real* rr = &L[static_cast<std::size_t>(r) * nb];
                Real v = rr[k];
                for (int t = 0; t < k; ++t) v -= rr[t] * rk[t];
                rr[k] = v / dk;
            }
        }
        // R (dofs x pairs), then X = K^-1 R column by column.
        std::vector<Real> R(static_cast<std::size_t>(nb) * np), col(nb);
        for (int d = 0; d < nb; ++d)
            for (int r = 0; r < np; ++r) R[static_cast<std::size_t>(d) * np + r] = load(bl.dofs[d], bl.pairs[r]);
        bl.X.assign(R.size(), 0.0);
        for (int r = 0; r < np; ++r) {
            for (int d = 0; d < nb; ++d) col[d] = R[static_cast<std::size_t>(d) * np + r];
            for (int d = 0; d < nb; ++d) {
                Real v = col[d];
                for (int t = 0; t < d; ++t) v -= L[static_cast<std::size_t>(d) * nb + t] * col[t];
                col[d] = v / L[static_cast<std::size_t>(d) * nb + d];
            }
            for (int d = nb - 1; d >= 0; --d) {
                Real v = col[d];
                for (int t = d + 1; t < nb; ++t) v -= L[static_cast<std::size_t>(t) * nb + d] * col[t];
                col[d] = v / L[static_cast<std::size_t>(d) * nb + d];
            }
            for (int d = 0; d < nb; ++d) bl.X[static_cast<std::size_t>(d) * np + r] = col[d];
        }
        // G = int e0 C e0 - R^T X.
        bl.G.assign(static_cast<std::size_t>(np) * np, 0.0);
        for (int a = 0; a < np; ++a)
            for (int b = a; b < np; ++b) {
                const int r = bl.pairs[a], q = bl.pairs[b];
                Real v = prod(ex[r], ex[q]) + prod(ey[r], ey[q]) + nu * (prod(ex[r], ey[q]) + prod(ey[r], ex[q])) +
                         sh * (prod(g1[r], g1[q]) + prod(g1[r], g2[q]) + prod(g2[r], g1[q]) + prod(g2[r], g2[q]));
                for (int d = 0; d < nb; ++d) v -= R[static_cast<std::size_t>(d) * np + a] * bl.X[static_cast<std::size_t>(d) * np + b];
                bl.G[static_cast<std::size_t>(a) * np + b] = bl.G[static_cast<std::size_t>(b) * np + a] = v;
            }
        bl.Xt.resize(bl.X.size());
        for (int d = 0; d < nb; ++d)
            for (int r = 0; r < np; ++r) bl.Xt[static_cast<std::size_t>(r) * nb + d] = bl.X[static_cast<std::size_t>(d) * np + r];
        B->blocks.push_back(std::move(bl));
    }

#ifdef GLS_STRETCH_LOCK
    std::lock_guard<std::mutex> g(lock);
#endif
    return cache.emplace(key, B).first->second;
}

void stretchLegendre(int degree, Real t, Real* L, Real* dL) { legendre(degree, t, L, dL); }

}  // namespace gls
