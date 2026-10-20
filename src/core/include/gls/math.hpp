// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Small vector and quaternion types. SI units throughout the core.
#pragma once

#include <algorithm>
#include <cmath>

namespace gls {

using Real = double;

inline constexpr Real kPi = 3.14159265358979323846;
inline constexpr Real kGravity = 9.80665;  // m/s^2
inline constexpr Real kAirDensity = 1.225;  // kg/m^3, ISA sea level

inline constexpr Real deg(Real d) { return d * kPi / 180.0; }
inline constexpr Real sq(Real x) { return x * x; }
inline Real clampr(Real v, Real lo, Real hi) { return std::min(hi, std::max(lo, v)); }
inline Real lerp(Real a, Real b, Real t) { return a + (b - a) * t; }

struct Vec2 {
    Real x = 0, y = 0;
    constexpr Vec2() = default;
    constexpr Vec2(Real x_, Real y_) : x(x_), y(y_) {}
    Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(Real s) const { return {x * s, y * s}; }
    Vec2 operator/(Real s) const { return {x / s, y / s}; }
    Vec2 operator-() const { return {-x, -y}; }
    Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(Real s) { x *= s; y *= s; return *this; }
};
inline Real dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline Real cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline Real length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline Vec2 normalize(Vec2 a) {
    const Real l = length(a);
    return l > 1e-300 ? a / l : Vec2{1, 0};
}
inline Vec2 perp(Vec2 a) { return {-a.y, a.x}; }
inline Vec2 rotate(Vec2 a, Real ang) {
    const Real c = std::cos(ang), s = std::sin(ang);
    return {c * a.x - s * a.y, s * a.x + c * a.y};
}

struct Vec3 {
    Real x = 0, y = 0, z = 0;
    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(Real s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(Real s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
};
inline Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real length(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(const Vec3& a) {
    const Real l = length(a);
    return l > 1e-300 ? a / l : Vec3{0, 0, 1};
}

// Unit quaternion (w, x, y, z): rotates body vectors into the pane frame.
struct Quat {
    Real w = 1, x = 0, y = 0, z = 0;
    Vec3 rotate(const Vec3& v) const {
        const Vec3 u{x, y, z};
        const Vec3 t = cross(u, v) * 2.0;
        return v + t * w + cross(u, t);
    }
    Quat operator*(const Quat& b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z, w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x, w * b.z + x * b.y - y * b.x + z * b.w};
    }
    void normalise() {
        const Real n = std::sqrt(w * w + x * x + y * y + z * z);
        w /= n; x /= n; y /= n; z /= n;
    }
    // Advances by angular velocity `om` (pane frame, rad/s) over dt.
    void integrate(const Vec3& om, Real dt) {
        const Real a = length(om) * dt;
        if (a < 1e-12) return;
        const Vec3 ax = normalize(om);
        const Real s = std::sin(a / 2);
        const Quat dq{std::cos(a / 2), ax.x * s, ax.y * s, ax.z * s};
        *this = dq * *this;
        normalise();
    }
};

}  // namespace gls
