// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Contact and penetration mechanics, each from its published model:
//   - spheres: Hertz, Thornton's elastic-plastic limit, Hunt-Crossley damping
//   - sharp tips: cone indentation at the glass hardness; Lawn-Evans-Marshall
//     median cracks; Roesler cone cracks
//   - soft bodies (bird substitute): Wilbeck's shock and steady-flow pressures
//   - post-breakage laminates: an axisymmetric interlayer membrane
//   - bullets: a layered cone-and-column model after Florence
#pragma once

#include <vector>

#include "gls/library.hpp"
#include "gls/math.hpp"

namespace gls {

Real contactModulus(Real E1, Real nu1, Real E2, Real nu2);

// Sphere (or rounded nose) on a flat: elastic Hertz, flattening at the
// impactor's flow pressure (Thornton), viscoelastic loss (Hunt-Crossley).
class SphereContact {
public:
    void configure(Real radius, Real Estar, Real flowPressure, Real restitution, Real approachSpeed);
    // Force (N, >= 0) at indentation delta (m) and rate (m/s, + closing).
    Real force(Real delta, Real rate);
    Real contactRadius() const { return a_; }
    Real peakForce() const { return fmax_; }
    bool yielded() const { return yielded_; }

private:
    Real R_ = 0.01, Es_ = 1e10, py_ = 0, e_ = 1, v0_ = 1;
    Real dmax_ = 0, dp_ = 0, fmax_ = 0, a_ = 0, dy_ = 1e9;
    bool yielded_ = false;
};

// Sharp cone tip into glass: plastic indentation at the hardness,
// P = pi tan^2(psi) H d^2, with elastic recovery on unloading.
class SharpContact {
public:
    void configure(Real halfAngle, Real hardness, Real tipRadius);
    Real force(Real delta, Real rate);
    Real contactRadius() const { return a_; }
    Real peakForce() const { return fmax_; }

private:
    Real alpha_ = 1, H_ = 5e9, tip_ = 0, dmax_ = 0, fmax_ = 0, a_ = 0;
};

// Flat-ended punch (a timber end, a brick face): elastic Boussinesq
// stiffness 2 E* a, capped where the impactor crushes.
class FlatContact {
public:
    void configure(Real radius, Real Estar, Real crush);
    Real force(Real delta, Real rate);
    Real contactRadius() const { return a_; }
    Real peakForce() const { return fmax_; }

private:
    Real a_ = 0.03, k_ = 1e8, cap_ = 1e9, dmax_ = 0, dp_ = 0, fmax_ = 0;
};

// Wilbeck soft-body impact (birds, gelatine): a brief Hugoniot shock at the
// first contact, then steady flow at the momentum flux, for the time the
// body takes to flow onto the surface.
struct SoftBodyLoad {
    Real density = 950, c0 = 1483, slope = 2.0;  // water-like Hugoniot, porosity reduces density
    Real radius = 0.057, length = 0.228;         // m
    Real normalSpeed = 100;                       // m/s
    Real hugoniotPressure() const;
    Real steadyForce() const;
    Real duration() const;                        // s, squash-up time L / v
    Real force(Real t) const;                     // N at time t since contact
    Real patchRadius(Real t) const;
};

// Lawn-Evans-Marshall median/radial crack length for a sharp contact load.
Real medianCrackDepth(Real force, Real E, Real H, Real toughness);
// Roesler cone crack length for a blunt contact load past initiation.
Real coneCrackLength(Real force, Real toughness);
// Peak radial tension at the edge of a Hertz contact (initiates the cone).
Real hertzEdgeTension(Real force, Real contactRadius, Real poisson);

// Post-breakage laminate: the interlayer (and any polymer plies) as an
// axisymmetric membrane pulled by the impactor's rim. Tension falls as 1/r
// from the rim, so strain concentrates there and that is where it tears.
struct MembranePly {
    Real thickness = 0, youngs = 0, flow = 0, tensile = 0, failureStrain = 0;
    bool torn = false;
};

class Membrane {
public:
    void configure(std::vector<MembranePly> plies, Real rimRadius, Real outerRadius, Real fragmentStiffening,
                   Real strainConcentration);
    // Out-of-plane force (N) at deflection delta (m) of the rim; updates tearing.
    Real force(Real delta);
    bool torn() const;
    // The impactor wraps the membrane as it goes deeper: the rim grows.
    void setRim(Real r) { Rc_ = clampr(r, 1e-3, 0.95 * Rm_); }
    Real rimStrain() const { return rimStrain_; }
    Real outerRadius() const { return Rm_; }
    Real rimRadius() const { return Rc_; }
    // Membrane deflection (m) at radius r from the centre for rim deflection delta.
    Real shape(Real r, Real delta) const;

private:
    Real tension(Real strain) const;  // N/m summed over intact plies
    Real strainAt(Real N) const;      // inverse (monotone), from a table
    void tabulate();
    std::vector<MembranePly> plies_;
    std::vector<Real> tabE_, tabN_;
    Real Rc_ = 0.02, Rm_ = 0.2, stiff_ = 1, conc_ = 1, rimStrain_ = 0;
};

// Layered ballistic penetration. The projectile crushes a cone in each glass
// ply (half-angle ~68 deg), shares momentum with the comminuted column it
// drives ahead, pays the comminution energy, and mushrooms if soft. Interlayers
// add mass and tearing energy; a polymer backing absorbs the column as a
// membrane until it perforates. Returns forces for the plate model as it goes.
struct BallisticLayer {
    int ply = 0;
    Real z0 = 0, z1 = 0;
    MaterialKind kind = MaterialKind::Glass;
    const Material* mat = nullptr;
};

class Ballistic {
public:
    struct Event {
        int ply = 0;
        Real time = 0;
        Real coneRadius = 0;   // m, crushed disc on entry to the ply
        Real energy = 0;       // J, kinetic energy on entry
        Real speed = 0;
    };

    void configure(const std::vector<BallisticLayer>& layers, const ImpactorDef& p, Real speed, Real mushroomFactor);
    // Advances dt; returns the force on the laminate (N, + into the pane).
    Real step(Real dt, Real time);
    bool done() const { return stopped_ || exited_; }
    bool stopped() const { return stopped_; }
    bool exited() const { return exited_; }
    Real speed() const { return v_; }
    Real depth() const { return depth_; }
    Real columnRadius() const { return r_; }
    Real projectileRadius() const { return ap_; }
    Real projectileMass() const { return mp_; }
    Real columnMass() const { return mcol_; }
    Real bulge() const { return bulge_; }
    Real absorbed() const { return absorbed_; }
    int layer() const { return layer_; }
    std::vector<Event>& events() { return events_; }

private:
    std::vector<BallisticLayer> layers_;
    std::vector<Event> events_;
    Real mp_ = 0.008, ap0_ = 0.0045, ap_ = 0.0045, L0_ = 0.015, rhop_ = 11000, Yp_ = 1e8, v0_ = 360;
    Real v_ = 0, depth_ = 0, mcol_ = 0, r_ = 0, glassDepth_ = 0, absorbed_ = 0, bulge_ = 0;
    Real mushroom_ = 1, backingTravel_ = 0;
    int layer_ = -1;
    bool stopped_ = false, exited_ = false;
};

}  // namespace gls
