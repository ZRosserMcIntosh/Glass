// Glass Dynamics — an independent reference for the large-deflection plate.
// SPDX-License-Identifier: MIT
//
// Solves von Kármán's equations for a uniformly loaded, simply supported
// rectangular plate by finite differences on the Airy stress function,
// the same kind of analysis as Vallabhan & Wang's (Texas Tech, 1981) behind
// the ASTM E1300 charts. It shares no code with the engine's plate:
//
//   D ∇⁴w = p + F,yy w,xx + F,xx w,yy − 2 F,xy w,xy
//     ∇⁴F = E h (w,xy² − w,xx w,yy)          N_x = F,yy, N_y = F,xx, N_xy = −F,xy
//
// Simply supported: w = 0 and w,nn = 0 at the edges. In plane, the edges are
// free (N_n = N_nt = 0, a pane sliding in its gasket): F = 0 and F,n = 0.
// A quarter of the plate on a uniform grid (central differences, ghost nodes
// reflected about the edges and the symmetry lines), Picard iteration with
// under-relaxation and banded LU, then Richardson extrapolation over two
// grids.
//
//   vonkarman [b/a] [nu] [N]       Q = p a^4/(E h^4) against the centre
//                                  deflection w/h (and ASTM E1300's), the
//                                  largest principal stress sigma a^2/(E h^2)
//                                  and where it is, a the shorter (x) side
//   vonkarman --study b/a nu Q     the same at one load on four grids
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

struct Band {
    int n = 0, bw = 0;
    std::vector<double> a;  // row r, column c at r * (2 bw + 1) + (c - r + bw)
    void init(int n_, int bw_) {
        n = n_;
        bw = bw_;
        a.assign(static_cast<std::size_t>(n) * (2 * bw + 1), 0.0);
    }
    double& at(int r, int c) { return a[static_cast<std::size_t>(r) * (2 * bw + 1) + (c - r + bw)]; }
    // In-place LU without pivoting (the operators here are diagonally strong).
    void factor() {
        for (int k = 0; k < n; ++k) {
            const double piv = at(k, k);
            const int rmax = std::min(n - 1, k + bw);
            for (int r = k + 1; r <= rmax; ++r) {
                double& l = at(r, k);
                if (l == 0) continue;
                l /= piv;
                for (int c = k + 1; c <= rmax; ++c) at(r, c) -= l * at(k, c);
            }
        }
    }
    void solve(std::vector<double>& x) {
        for (int r = 0; r < n; ++r)
            for (int c = std::max(0, r - bw); c < r; ++c) x[r] -= at(r, c) * x[c];
        for (int r = n - 1; r >= 0; --r) {
            for (int c = r + 1; c <= std::min(n - 1, r + bw); ++c) x[r] -= at(r, c) * x[c];
            x[r] /= at(r, r);
        }
    }
};

struct Result {
    double Q = 0, wc = 0, smax = 0, sx = 0, sy = 0, scentre = 0;
    bool converged = false;
};

class Solver {
public:
    Solver(double beta, double nu, int N) : beta_(beta), nu_(nu), N_(N) {
        Ny_ = std::max(2, static_cast<int>(std::lround(N * beta)));
        hx_ = 0.5 / N_;
        hy_ = 0.5 * beta_ / Ny_;
        n_ = N_ * Ny_;
        w_.assign(n_, 0.0);
        F_.assign(n_, 0.0);
        // The stress function's operator does not change: factor it once.
        bf_.init(n_, 2 * N_);
        for (int j = 1; j <= Ny_; ++j)
            for (int i = 1; i <= N_; ++i) addBiharmonic(bf_, idx(i, j), i, j, false, 1.0);
        bf_.factor();
    }

    // Static solution at load Q, starting from the last one.
    Result solve(double Q) {
        Result r;
        r.Q = Q;
        const double Dh = 1.0 / (12 * (1 - nu_ * nu_));
        double relax = 0.6;
        double last = 1e30;
        for (int it = 0; it < 4000; ++it) {
            // F from the current w.
            std::vector<double> rhs(n_);
            for (int j = 1; j <= Ny_; ++j)
                for (int i = 1; i <= N_; ++i) {
                    const double wxx = dxx(w_, i, j, true), wyy = dyy(w_, i, j, true), wxy = dxy(w_, i, j, true);
                    rhs[idx(i, j)] = wxy * wxy - wxx * wyy;
                }
            bf_.solve(rhs);
            F_ = rhs;
            // w from the bending equation with the membrane forces held.
            Band bw;
            bw.init(n_, 2 * N_);
            std::vector<double> q(n_, Q);
            for (int j = 1; j <= Ny_; ++j)
                for (int i = 1; i <= N_; ++i) {
                    const int r0 = idx(i, j);
                    addBiharmonic(bw, r0, i, j, true, Dh);
                    const double Fyy = dyy(F_, i, j, false), Fxx = dxx(F_, i, j, false), Fxy = dxy(F_, i, j, false);
                    // − (Fyy w,xx + Fxx w,yy − 2 Fxy w,xy)
                    addTerm(bw, r0, i + 1, j, true, -Fyy / (hx_ * hx_));
                    addTerm(bw, r0, i - 1, j, true, -Fyy / (hx_ * hx_));
                    addTerm(bw, r0, i, j, true, 2 * Fyy / (hx_ * hx_) + 2 * Fxx / (hy_ * hy_));
                    addTerm(bw, r0, i, j + 1, true, -Fxx / (hy_ * hy_));
                    addTerm(bw, r0, i, j - 1, true, -Fxx / (hy_ * hy_));
                    const double c = 2 * Fxy / (4 * hx_ * hy_);
                    addTerm(bw, r0, i + 1, j + 1, true, c);
                    addTerm(bw, r0, i - 1, j - 1, true, c);
                    addTerm(bw, r0, i + 1, j - 1, true, -c);
                    addTerm(bw, r0, i - 1, j + 1, true, -c);
                }
            bw.factor();
            bw.solve(q);
            double change = 0, size = 0;
            for (int k = 0; k < n_; ++k) {
                change = std::max(change, std::fabs(q[k] - w_[k]));
                size = std::max(size, std::fabs(q[k]));
            }
            if (change > last * 1.05) relax = std::max(0.05, relax * 0.7);
            last = change;
            for (int k = 0; k < n_; ++k) w_[k] += relax * (q[k] - w_[k]);
            if (change < 1e-11 * std::max(size, 1e-12)) {
                r.converged = true;
                break;
            }
        }
        r.wc = w_[idx(N_, Ny_)];
        // Largest principal stress on either face, every node including the edges.
        r.smax = -1e30;
        for (int j = 0; j <= Ny_; ++j)
            for (int i = 0; i <= N_; ++i) {
                const double s = principal(i, j);
                if (s > r.smax) {
                    r.smax = s;
                    r.sx = i * hx_;
                    r.sy = j * hy_ / beta_;
                }
            }
        r.scentre = principal(N_, Ny_);
        return r;
    }

private:
    int idx(int i, int j) const { return (j - 1) * N_ + (i - 1); }
    // Unknown behind grid node (i, j) and its sign; -1 when the value is zero (an edge).
    int map(int i, int j, bool isW, double& sign) const {
        sign = 1;
        if (i < 0) { i = -i; if (isW) sign = -sign; }
        if (j < 0) { j = -j; if (isW) sign = -sign; }
        if (i > N_) i = 2 * N_ - i;
        if (j > Ny_) j = 2 * Ny_ - j;
        if (i == 0 || j == 0) return -1;
        return idx(i, j);
    }
    double val(const std::vector<double>& u, int i, int j, bool isW) const {
        double s;
        const int k = map(i, j, isW, s);
        return k < 0 ? 0.0 : s * u[k];
    }
    double dxx(const std::vector<double>& u, int i, int j, bool isW) const {
        return (val(u, i + 1, j, isW) - 2 * val(u, i, j, isW) + val(u, i - 1, j, isW)) / (hx_ * hx_);
    }
    double dyy(const std::vector<double>& u, int i, int j, bool isW) const {
        return (val(u, i, j + 1, isW) - 2 * val(u, i, j, isW) + val(u, i, j - 1, isW)) / (hy_ * hy_);
    }
    double dxy(const std::vector<double>& u, int i, int j, bool isW) const {
        return (val(u, i + 1, j + 1, isW) - val(u, i + 1, j - 1, isW) - val(u, i - 1, j + 1, isW) +
                val(u, i - 1, j - 1, isW)) / (4 * hx_ * hy_);
    }
    void addTerm(Band& b, int row, int i, int j, bool isW, double c) const {
        double s;
        const int k = map(i, j, isW, s);
        if (k >= 0) b.at(row, k) += s * c;
    }
    void addBiharmonic(Band& b, int row, int i, int j, bool isW, double scale) const {
        const double x4 = scale / std::pow(hx_, 4), y4 = scale / std::pow(hy_, 4), xy = 2 * scale / (hx_ * hx_ * hy_ * hy_);
        addTerm(b, row, i + 2, j, isW, x4);
        addTerm(b, row, i - 2, j, isW, x4);
        addTerm(b, row, i + 1, j, isW, -4 * x4 - 2 * xy);
        addTerm(b, row, i - 1, j, isW, -4 * x4 - 2 * xy);
        addTerm(b, row, i, j + 2, isW, y4);
        addTerm(b, row, i, j - 2, isW, y4);
        addTerm(b, row, i, j + 1, isW, -4 * y4 - 2 * xy);
        addTerm(b, row, i, j - 1, isW, -4 * y4 - 2 * xy);
        addTerm(b, row, i, j, isW, 6 * x4 + 6 * y4 + 4 * xy);
        for (int di : {-1, 1})
            for (int dj : {-1, 1}) addTerm(b, row, i + di, j + dj, isW, xy);
    }
    // Largest principal stress of the two faces at a node, sigma a^2 / (E h^2).
    double principal(int i, int j) const {
        const double Fxx = dxx(F_, i, j, false), Fyy = dyy(F_, i, j, false), Fxy = dxy(F_, i, j, false);
        const double wxx = dxx(w_, i, j, true), wyy = dyy(w_, i, j, true), wxy = dxy(w_, i, j, true);
        const double k = 1 / (2 * (1 - nu_ * nu_));
        double best = -1e30;
        for (int face : {-1, 1}) {
            const double sxx = Fyy - face * k * (wxx + nu_ * wyy);
            const double syy = Fxx - face * k * (wyy + nu_ * wxx);
            const double sxy = -Fxy - face * k * (1 - nu_) * wxy;
            const double c = 0.5 * (sxx + syy), r = std::sqrt(0.25 * (sxx - syy) * (sxx - syy) + sxy * sxy);
            best = std::max(best, c + r);
        }
        return best;
    }

    double beta_, nu_;
    int N_, Ny_ = 0, n_ = 0;
    double hx_ = 0, hy_ = 0;
    std::vector<double> w_, F_;
    Band bf_;
};

// ASTM E1300's centre deflection of four-sided simply supported glass (its fit
// to the charts' nonlinear analysis): w = t exp(r0 + r1 x + r2 x^2),
// x = ln(ln(q (a b)^2 / (E t^4))), AR = long / short side.
double e1300(double Q, double beta) {
    const double AR = std::max(beta, 1 / beta);
    const double qab = Q * beta * beta;  // q (a b)^2 / (E t^4) with a = 1, b = beta
    const double r0 = 0.553 - 3.83 * AR + 1.11 * AR * AR - 0.0969 * AR * AR * AR;
    const double r1 = -2.29 + 5.83 * AR - 2.17 * AR * AR + 0.2067 * AR * AR * AR;
    const double r2 = 1.485 - 1.908 * AR + 0.815 * AR * AR - 0.0822 * AR * AR * AR;
    if (qab <= std::exp(1.0)) return 0;
    const double x = std::log(std::log(qab));
    return std::exp(r0 + r1 * x + r2 * x * x);
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    if (argc > 1 && std::string(argv[1]) == "--study") {
        // Grid convergence at one load: vonkarman --study b/a nu Q
        const double beta = argc > 2 ? std::atof(argv[2]) : 1.0, nu = argc > 3 ? std::atof(argv[3]) : 0.22;
        const double Q = argc > 4 ? std::atof(argv[4]) : 400;
        for (int n : {8, 16, 32, 64}) {
            Solver s(beta, nu, n);
            for (int k = 1; k < 8; ++k) s.solve(Q * k / 8);
            const Result r = s.solve(Q);
            std::printf("N %3d: w/h %.4f  s_max %.3f at x/a %.4f y/b %.4f  s_centre %.3f%s\n", n, r.wc, r.smax, r.sx, r.sy,
                        r.scentre, r.converged ? "" : "  (not converged)");
        }
        return 0;
    }
    const double beta = argc > 1 ? std::atof(argv[1]) : 1.0;
    const double nu = argc > 2 ? std::atof(argv[2]) : 0.22;
    const int N = argc > 3 ? std::atoi(argv[3]) : 32;
    const std::vector<double> loads = {1, 10, 20, 40, 60, 80, 100, 150, 200, 250, 300, 400, 500, 700, 1000};
    Solver coarse(beta, nu, N), fine(beta, nu, 2 * N);
    std::printf("# b/a = %.3f, nu = %.3f, grids %d and %d per half side, Richardson-extrapolated\n", beta, nu, N, 2 * N);
    std::printf("#      Q    w/h  E1300   s_max  at x/a  y/b   s_centre   (s = sigma a^2/(E h^2), a = the x side)\n");
    for (double Q : loads) {
        const Result c = coarse.solve(Q), f = fine.solve(Q);
        auto rich = [](double a, double b) { return b + (b - a) / 3; };
        std::printf("%8.1f %6.3f %6.3f %7.3f  %5.3f %5.3f   %7.3f%s\n", Q, rich(c.wc, f.wc), e1300(Q, beta), rich(c.smax, f.smax),
                    f.sx, f.sy, rich(c.scentre, f.scentre), c.converged && f.converged ? "" : "  (not converged)");
    }
    return 0;
}
