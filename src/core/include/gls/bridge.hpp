// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Glazing on a host body: OpenRoad Dynamics vehicles (body frame x forward,
// y left, z up) and Aviation Dynamics aircraft (body frame FRD). A mount
// places a pane's frame in the host's body frame; strike() turns a host-frame
// impact into a pane launch, and the pane's status goes back to the host as
// what it can use (the driver's or pilot's view, whether the cabin is still
// sealed, how much area is open to the airflow).
//
// The core does not link either engine: the frames are written out here and
// tested in tests/test_bridge.cpp against the conventions both repositories
// document (OpenRoad ord/host_frame.hpp, Aviation ADR 0002).
#pragma once

#include "gls/math.hpp"
#include "gls/pane.hpp"

namespace gls {

// The pane frame in a host body frame: origin at the bottom-left corner of
// the outer (struck) face, unit axes along the width (x), up the pane (y),
// and the inward normal (z, from the outer face into the glass).
struct PaneMount {
    Vec3 origin;
    Vec3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};

    Vec3 toPane(const Vec3& hostPoint) const;
    Vec3 toPaneDirection(const Vec3& hostVector) const;
    Vec3 toHost(const Vec3& panePoint) const;
    Vec3 toHostDirection(const Vec3& paneVector) const;
};

// From three corners of the outer face, seen from outside: bottom-left,
// bottom-right, top-left. The inward normal follows from the right-hand rule.
PaneMount mountFromCorners(const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft);

// A host-frame impact: point on (or just outside) the outer face and the
// impactor's velocity relative to the host body (the host's own velocity
// already subtracted, so a stone at highway speed carries the car's speed).
int strike(Pane& pane, const PaneMount& mount, const ImpactorDef& impactor, const Vec3& hostPoint,
           const Vec3& relativeVelocity, Real lead = 0.05);

namespace vehicle {
// OpenRoad Dynamics body frame (ISO 8855): x forward, y left, z up, metres.
// A windscreen raked back by `rakeDeg` from vertical, its bottom edge centred
// at `baseCentre`.
PaneMount windscreen(const Vec3& baseCentre, Real width, Real height, Real rakeDeg);
// A side window on the left (y > 0) or right door, bottom edge centred at
// `baseCentre`, tilted inward at the top by `tumblehomeDeg`.
PaneMount sideWindow(const Vec3& baseCentre, Real width, Real height, bool left, Real tumblehomeDeg);
}  // namespace vehicle

namespace aircraft {
// Aviation Dynamics body frame: x forward, y right, z down (FRD), metres.
// A windshield panel facing forward and outboard: bottom edge centred at
// `baseCentre`, raked back from vertical by `rakeDeg` and turned outboard by
// `sweepDeg` (positive to the right).
PaneMount windshield(const Vec3& baseCentre, Real width, Real height, Real rakeDeg, Real sweepDeg);
// Velocity of a bird (at rest in the air) relative to an aircraft flying at
// true airspeed `tas` with angle of attack `alpha` and sideslip `beta`, in
// the body frame.
Vec3 relativeAirVelocity(Real tas, Real alpha, Real beta);
}  // namespace aircraft

}  // namespace gls
