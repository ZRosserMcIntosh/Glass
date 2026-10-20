// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Bending of the pane: a laminate reduced to one Kirchhoff plate, solved by
// modal superposition (Navier modes of a simply supported rectangle). Each
// mode is advanced exactly for a force held over the step, so the step can be
// anything from 50 ns during a bullet strike to milliseconds while the pane
// rings down. Stresses come back per ply through the laminate's shear
// coupling (Bennison-Wölfel effective thickness).
#pragma once

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
    void sampleGrid(int nx, int ny, Real* w, Real* wdot, Curvature* k) const;

    // Near-field curvature (1/m, isotropic) to add within the modal
    // resolution radius of a concentrated load F: the truncated, filtered
    // series cannot hold the logarithmic peak under the load.
    // `loadTime` limits it to the zone a flexural disturbance covers since the
    // load began, r ~ sqrt(t sqrt(D / rho h)): a short pulse cannot bend the
    // plate quasi-statically out to the resolution radius.
    Real nearField(Real force, Real r, Real equivalentRadius, Real loadTime = 1e9) const;
    Real resolutionRadius() const { return resolution_; }

    Real energy() const;  // kinetic + strain, J
    Real maxFrequencyHz() const;
    Real fundamentalHz() const;
    int modesX() const { return M_; }
    int modesY() const { return N_; }
    Real width() const { return a_; }
    Real height() const { return b_; }
    Real stiffness() const { return D_; }

private:
    void basis(Real x, Real y, std::vector<Real>& sx, std::vector<Real>& cx, std::vector<Real>& sy,
               std::vector<Real>& cy) const;
    void coefficients(Real dt);

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
};

// Westergaard's equivalent load radius for thick-plate contact.
Real equivalentRadius(Real contactRadius, Real thickness);

// Bessel J1 (rational approximation, |error| < 1e-8).
Real besselJ1(Real x);

}  // namespace gls
