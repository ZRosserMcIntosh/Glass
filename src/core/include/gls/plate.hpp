// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Bending of the pane: a laminate reduced to one Kirchhoff plate, solved by
// modal superposition (Navier modes of a simply supported rectangle). Each
// mode is advanced exactly for a force held over the step, so the step can be
// anything from 50 ns during a bullet strike to milliseconds while the pane
// rings down. Stresses come back per ply through the laminate's shear
// coupling (Bennison-Wölfel effective thickness).
//
// Bent further than about its own thickness, a pane stretches in its plane
// (membrane action, von Kármán): the lowest modes carry a quartic membrane
// energy that stiffens them and adds an in-plane stress (stretch.cpp).
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "gls/library.hpp"
#include "gls/math.hpp"

namespace gls {

struct LaminateLayer {
    int ply = 0;
    Real z0 = 0, z1 = 0;  // depth of the faces from the front face, m
    Real plateModulus = 0;  // E / (1 - nu^2)
    Real poisson = 0.22;
    MaterialKind kind = MaterialKind::Glass;
    Real mid() const { return 0.5 * (z0 + z1); }
};

struct Laminate {
    std::vector<LaminateLayer> layers;
    Real thickness = 0;     // m
    Real massPerArea = 0;   // kg/m^2
    Real Dmono = 0;         // N*m, fully bonded
    Real Dlayered = 0;      // N*m, plies sliding freely
    Real gamma = 1;         // shear transfer coefficient (0 layered .. 1 monolithic)
    Real D = 0;             // effective bending stiffness, N*m
    Real neutralAxis = 0;   // depth of the monolithic neutral axis, m
    Real poisson = 0.22;

    // Bending stress lever arm (m) at depth z inside layer `l`: the plate
    // stress there is plateModulus * lever * curvature.
    Real lever(int l, Real z) const {
        const LaminateLayer& L = layers[l];
        return (z - L.mid()) + gamma * (L.mid() - neutralAxis);
    }
};

Laminate makeLaminate(const GlazingDef& g);

// Plate curvature (kappa = -w,ij), 1/m.
struct Curvature {
    Real kxx = 0, kyy = 0, kxy = 0;
};

// In-plane stress on one face of a ply, Pa (tension positive).
struct PlaneStress {
    Real sxx = 0, syy = 0, sxy = 0;
    // Principal stresses and the direction (radians) of sigma1.
    void principal(Real& s1, Real& s2, Real& angle1) const;
    Real normalAcross(Vec2 crackDir) const;  // traction normal to a crack running along crackDir
};

// Mid-surface strain from membrane action (engineering shear), tension positive.
struct PlaneStrain {
    Real exx = 0, eyy = 0, gxy = 0;
};

// Membrane action of a simply supported a x b pane whose edges slide freely
// in its plane, on the unit pane (x in [0, 1], y in [0, beta], beta = b / a)
// with unit in-plane stiffness. For deflection sum q_k psi_k (psi_k the
// stretching modes) the membrane energy is 1/2 p^T G p with p_jk = q_j q_k
// (j <= k, in units of a), and the in-plane displacement is -X p in a tensor
// Legendre series of the given degree. Both split into the four symmetry
// classes about the pane's centre lines, which do not couple.
struct StretchBasis {
    Real beta = 1, nu = 0.22;
    int degree = 0, pairs = 0, dofs = 0;
    std::vector<int> mm, nn;                // the stretching modes
    std::vector<int> dofI, dofJ;            // Legendre degrees in x and y of each in-plane dof
    std::vector<char> dofV;                 // 0: u, 1: v
    struct Block {
        std::vector<int> pairs, dofs;       // members of one symmetry class
        std::vector<Real> G;                // pairs x pairs
        std::vector<Real> X;                // dofs x pairs
        std::vector<Real> Xt;               // the same, pairs x dofs
    };
    std::vector<Block> blocks;
};
// Shared between panes of the same shape.
std::shared_ptr<const StretchBasis> stretchBasis(Real beta, Real nu, const std::vector<int>& mm,
                                                  const std::vector<int>& nn, int degree);
// Normalised Legendre polynomials on [0, 1] up to `degree`, and their derivatives.
void stretchLegendre(int degree, Real t, Real* L, Real* dL);

class ModalPlate {
public:
    // a x b pane, effective bending stiffness D, mass per area, Poisson ratio,
    // modal damping ratio and the number of modes along x and y.
    void configure(Real a, Real b, Real D, Real massPerArea, Real poisson, Real damping, int modesX, int modesY);
    void reset();

    // Loads held over the next step(s).
    void clearForces();
    void addPatch(Real x, Real y, Real radius, Real force);  // uniform disc load, + toward the back face
    void addPressure(Real p);                                // uniform pressure
    void settleStatic();  // jump to static equilibrium under the current loads

    void step(Real dt);

    // Membrane action: in-plane stiffness A = sum E h / (1 - nu^2) (N/m) of
    // the layup, its Poisson ratio, its thickness (stretching counts from a
    // quarter of it) and about how many of the lowest modes stretch (a box
    // in m and n). Without it the plate is linear.
    void configureStretch(Real membraneStiffness, Real poisson, Real thickness, int modes);
    bool stretches() const { return static_cast<bool>(sb_); }
    bool stretching() const;  // bent far enough now for its stretch to count
    // Mid-surface strain at a point (zero while not stretching).
    PlaneStrain membraneStrain(Real x, Real y) const;
    PlaneStrain membraneStrain(Vec2 p) const { return membraneStrain(p.x, p.y); }
    Real membraneEnergy() const { return sEnergy_; }  // J, as of the last kick
    int stretchModesX() const { return sBoxX_; }        // the box of stretching modes, m <= this
    int stretchModesY() const { return sBoxY_; }      // and n <= this
    // The interval between membrane kicks for the plate as it is now (no
    // limit while not stretching).
    Real stretchStep() const;

    Real deflection(Real x, Real y) const;
    Real velocity(Real x, Real y) const;
    Curvature curvature(Real x, Real y) const;
    Real deflection(Vec2 p) const { return deflection(p.x, p.y); }
    Real velocity(Vec2 p) const { return velocity(p.x, p.y); }
    Curvature curvature(Vec2 p) const { return curvature(p.x, p.y); }
    // Slopes of the velocity field (rad/s) for fragment spin.
    Vec2 velocityGradient(Real x, Real y) const;

    // Fields on an nx x ny lattice spanning the pane edge to edge (row major,
    // x fastest). Any output pointer may be null.
    void sampleGrid(int nx, int ny, Real* w, Real* wdot, Curvature* k, PlaneStrain* e = nullptr) const;

    // Near-field curvature (1/m, isotropic) to add within the modal
    // resolution radius of a concentrated load F: the truncated, filtered
    // series cannot hold the logarithmic peak under the load.
    // `loadTime` limits it to the zone a flexural disturbance covers since the
    // load began, r ~ sqrt(t sqrt(D / rho h)): a short pulse cannot bend the
    // plate quasi-statically out to the resolution radius.
    Real nearField(Real force, Real r, Real equivalentRadius, Real loadTime = 1e9) const;
    Real resolutionRadius() const { return resolution_; }

    Real energy() const;  // kinetic + strain, J
    // No curvature component exceeds this anywhere on the plate (1/m): every
    // mode at its full amplitude, |sin|, |cos| <= 1.
    Real curvatureBound() const;
    Real maxFrequencyHz() const;
    Real fundamentalHz() const;
    Real modeHz(int m, int n) const { return omega_[static_cast<std::size_t>(m - 1) * N_ + (n - 1)] / (2 * kPi); }
    int modesX() const { return M_; }
    int modesY() const { return N_; }
    Real width() const { return a_; }
    Real height() const { return b_; }
    Real stiffness() const { return D_; }

private:
    void basis(Real x, Real y, std::vector<Real>& sx, std::vector<Real>& cx, std::vector<Real>& sy,
               std::vector<Real>& cy) const;
    void coefficients(Real dt);
    // Membrane force on each stretching mode (N, modal) for the current q;
    // also refreshes the energy. Hessian (N/m, K x K) if asked.
    void stretchForce(std::vector<Real>& f, std::vector<Real>* hessian = nullptr) const;
    void settleStretched();
    void drift(Real t);  // the stretching modes alone, exactly, over t
    Real kickInterval() const;  // between membrane kicks, for the plate as it is
    void inPlane() const;  // the in-plane displacement for the current q (cached)

    Real a_ = 1, b_ = 1, D_ = 1, rhoH_ = 1, nu_ = 0.22, zeta_ = 0.01, resolution_ = 0.05;
    int M_ = 0, N_ = 0;
    std::vector<Real> q_, v_, f_, omega_, sigX_, sigY_;
    Real modalMass_ = 1;
    // Exact-step coefficients for the last dt.
    Real cacheDt_ = -1;
    std::vector<Real> c11_, c12_, c21_, c22_, g1_, g2_;
    mutable std::vector<Real> sx_, cx_, sy_, cy_;
    // sampleGrid's basis tables and scratch, kept for the last lattice size.
    std::vector<Real> patch_;  // disc-load factors 2 J1(kr)/(kr) for patchR_
    Real patchR_ = -1;
    mutable int gNx_ = 0, gNy_ = 0;
    mutable std::vector<Real> gSX_, gCX_, gSY_, gCY_, gT_[5];

    // Membrane action.
    std::shared_ptr<const StretchBasis> sb_;
    std::vector<int> sIdx_;          // q_ index of each stretching mode
    std::vector<char> sFlag_;        // per mode: stretches
    Real sA_ = 0, sThick_ = 0, sScale_ = 0;  // A, h, A / a^2
    Real sKappaMax_ = 0, sOmegaMax_ = 0;     // of the stretching modes
    int sBoxX_ = 0, sBoxY_ = 0;
    mutable Real sEnergy_ = 0;
    mutable std::vector<Real> sP_, sY_, sF_, sBp_, sBy_;
    std::vector<int> sPair_;         // pair index of stretching modes (j, k), K x K
    std::vector<Real> hc11_, hc12_, hc21_, hc22_, hg1_, hg2_;  // drift coefficients
    Real halfDt_ = -1;               // the drift they are for
    Real sClock_ = -1, sCarry_ = 0;  // s to the next membrane kick; the share of time the next kick stands for
    std::uint64_t rev_ = 0;          // bumped whenever q changes
    mutable std::uint64_t sURev_ = ~0ull;
    mutable std::vector<Real> sU_;   // in-plane displacement (dofs), units of a
    mutable std::vector<Real> sL_, sdL_, sLy_, sdLy_;
};

// Westergaard's equivalent load radius for thick-plate contact.
Real equivalentRadius(Real contactRadius, Real thickness);

// Bessel J1 (rational approximation, |error| < 1e-8).
Real besselJ1(Real x);

}  // namespace gls
