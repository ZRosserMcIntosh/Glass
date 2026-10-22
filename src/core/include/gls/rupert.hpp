// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// A Prince Rupert's drop: molten glass dripped into cold water. The outside
// freezes first and the inside, shrinking as it cools later, pulls it into
// very high compression while the core is left in tension. The head shrugs
// off a hammer or a lead bullet, because nothing at its surface can open a
// crack against that compression; but break the thin tail and a crack
// reaches the core, whose stored energy drives a front of forking cracks
// back to the head at the speed of a running crack, and the whole drop bursts
// into powder.
//
// A body of revolution along its axis x (head front at x = 0, the tail out
// along +x), its residual axial stress a function of r / R(x) only. A break
// runs as a front at the glass's terminal crack speed; every fragment's birth
// time and launch state is decided when the front starts, from the stored
// energy where it was, so stepping is only moving the clock. Fragments then
// fly ballistically (gravity, no drag) to a floor, which the host draws.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gls/library.hpp"
#include "gls/math.hpp"
#include "gls/rng.hpp"

namespace gls {

struct DropShape {
    Real headRadius = 5e-3;    // m
    Real length = 75e-3;       // m, head front to tail tip
    Real tipRadius = 0.35e-3;  // m
    Real neck = 7e-3;          // m, how fast the tail thins behind the head
    Real surfaceCompression = 500e6;  // Pa, at the head's surface
    Real coreTension = 100e6;         // Pa, on the axis
};

// Something hitting the head square on, along -x into its front.
struct DropStrike {
    Real mass = 0.008, radius = 0.0045, speed = 380;  // kg, m, m/s
    Real density = 10800, yield = 250e6;              // Pa; the projectile's flow stress
    Real youngs = 16e9, poisson = 0.44;               // the projectile's elastic constants
    bool rigid = false;  // a hammer or steel ball: elastic Hertz contact, it does not flow
};

struct DropFragment {
    Vec3 pos, vel;       // at birth, drop frame (y up)
    Real size = 0;       // m
    Real born = 0;       // s
    bool lead = false;   // a splash of the projectile, not glass
};

class RupertDrop {
public:
    RupertDrop(const DropShape& shape, const Material& glass, std::uint64_t seed = 1);

    Real radius(Real x) const;          // m, at distance x from the head's front
    // Residual axial stress (Pa, tension +) at radial share u = r / R in [0, 1].
    Real residual(Real u) const;
    Real compressionDepth() const;      // share of the radius in compression at the surface
    Real fragmentSize() const;          // m, from the core's stored energy

    // Snaps the tail at x (pliers): the piece beyond falls away, and a front
    // runs from x to the head.
    void snip(Real x);
    // Something hits the head. Returns whether the drop broke.
    bool strike(const DropStrike& s);

    void advance(Real t) { time_ = std::max(time_, t); }
    Real time() const { return time_; }

    // The broken stretch [lo, hi] of the axis now (lo > hi: none yet).
    Real brokenFrom() const;
    Real brokenTo() const;
    bool broken() const { return startedAt_ >= 0; }
    Real startedAt() const { return startedAt_; }  // s, when it began to break (-1: whole)
    bool whole() const { return startedAt_ < 0; }
    Real snippedAt() const { return snipX_; }   // tail cut here (length if not)
    Real length() const { return s_.length; }
    Real headRadius() const { return s_.headRadius; }
    Real frontSpeed() const { return speed_; }
    const std::vector<DropFragment>& fragments() const { return frags_; }

    // The last strike: its contact pressure (Pa), and what drove a crack into
    // the skin against what it needed to get through: a soft projectile that
    // flows, the tension at the edge of its face (Pa); a hard one, its peak
    // force (N) against the force a cone crack needs to cross the skin.
    Real contactPressure() const { return contactP_; }
    Real drive() const { return drive_; }
    Real needed() const { return needed_; }
    bool byForce() const { return byForce_; }
    Real skinDepth() const { return depth_; }     // m, compressed skin at the head
    Real skinCompression() const { return meanC_; }  // Pa, its mean compression
    const std::string& outcome() const { return outcome_; }

private:
    void startFront(Real x0, bool towardHead, bool towardTail, Real when);
    void spray(Real x0, bool towardHead, bool towardTail, Real when);
    DropShape s_;
    Material g_;
    Rng rng_;
    Real k_ = 10;          // exponent of the residual profile
    Real time_ = 0, startedAt_ = -1, origin_ = 0, speed_ = 1500;
    bool headward_ = false, tailward_ = false;
    Real snipX_ = 0;
    Real contactP_ = 0, drive_ = 0, needed_ = 0, depth_ = 0, meanC_ = 0;
    bool byForce_ = false;
    std::string outcome_;
    std::vector<DropFragment> frags_;
};

}  // namespace gls
