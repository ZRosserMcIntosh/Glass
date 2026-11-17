// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/bridge.hpp"

#include <cmath>

namespace gls {

Vec3 PaneMount::toPane(const Vec3& p) const {
    const Vec3 d = p - origin;
    return {dot(d, x), dot(d, y), dot(d, z)};
}
Vec3 PaneMount::toPaneDirection(const Vec3& v) const { return {dot(v, x), dot(v, y), dot(v, z)}; }
Vec3 PaneMount::toHost(const Vec3& p) const { return origin + x * p.x + y * p.y + z * p.z; }
Vec3 PaneMount::toHostDirection(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

PaneMount mountFromCorners(const Vec3& bl, const Vec3& br, const Vec3& tl) {
    PaneMount m;
    m.origin = bl;
    m.x = normalize(br - bl);
    // Remove any skew so the frame is orthonormal.
    const Vec3 up = tl - bl;
    m.y = normalize(up - m.x * dot(up, m.x));
    // Seen from outside, x right and y up: x cross y points out toward the
    // viewer, so the inward normal is y cross x.
    m.z = cross(m.y, m.x);
    return m;
}

int strike(Pane& pane, const PaneMount& mount, const ImpactorDef& d, const Vec3& hostPoint, const Vec3& vel,
           Real lead) {
    const Vec3 p = mount.toPane(hostPoint);
    const Vec3 v = mount.toPaneDirection(vel);
    const Real sp = length(v);
    if (sp <= 0) return -1;
    const bool fromBack = v.z < 0;
    const Vec3 dir = v / sp;
    // Angles of the travel in the pane frame.
    const Real yaw = std::atan2(dir.x, std::fabs(dir.z));
    const Real pitch = std::asin(clampr(dir.y, -1, 1));
    return pane.fire(d, {p.x, p.y}, sp, yaw, pitch, fromBack, lead);
}

namespace vehicle {

PaneMount windscreen(const Vec3& c, Real w, Real h, Real rake) {
    // Outer face seen from ahead of the car: facing the car, the car's left
    // (+y) is on the viewer's right, so the pane's +x runs toward +y.
    const Real r = deg(rake);
    const Vec3 up{-std::sin(r), 0, std::cos(r)};  // leans back toward -x
    const Vec3 bl = c + Vec3{0, -0.5 * w, 0};
    const Vec3 br = c + Vec3{0, 0.5 * w, 0};
    return mountFromCorners(bl, br, bl + up * h);
}

PaneMount sideWindow(const Vec3& c, Real w, Real h, bool left, Real tumble) {
    const Real t = deg(tumble);
    // Seen from outside the left door the car's rear (-x) is on the viewer's
    // right; outside the right door, its front.
    const Vec3 fwd = left ? Vec3{-1, 0, 0} : Vec3{1, 0, 0};
    const Vec3 up{0, left ? -std::sin(t) : std::sin(t), std::cos(t)};
    const Vec3 bl = c - fwd * (0.5 * w);
    const Vec3 br = c + fwd * (0.5 * w);
    return mountFromCorners(bl, br, bl + up * h);
}

}  // namespace vehicle

namespace aircraft {

PaneMount windshield(const Vec3& c, Real w, Real h, Real rake, Real sweep) {
    // FRD: seen from ahead, the aircraft's right (+y) is the viewer's left.
    const Real r = deg(rake), s = deg(sweep);
    const Vec3 across{std::sin(s), -std::cos(s), 0};       // viewer's right along the panel
    const Vec3 up{-std::sin(r), 0, -std::cos(r)};          // up is -z, leaning back toward -x
    const Vec3 bl = c - across * (0.5 * w);
    const Vec3 br = c + across * (0.5 * w);
    return mountFromCorners(bl, br, bl + up * h);
}

Vec3 relativeAirVelocity(Real tas, Real alpha, Real beta) {
    // The air (and a bird resting in it) moves at -V_body relative to the
    // aircraft: V_body = tas (cos a cos b, sin b, sin a cos b).
    return Vec3{std::cos(alpha) * std::cos(beta), std::sin(beta), std::sin(alpha) * std::cos(beta)} * -tas;
}

}  // namespace aircraft

}  // namespace gls
