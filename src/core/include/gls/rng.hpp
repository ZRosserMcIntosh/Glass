// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Deterministic PCG32 random numbers. The same seed gives the same fracture on
// every platform (no std::distribution, whose output is implementation
// defined).
#pragma once

#include <cmath>
#include <cstdint>

#include "gls/math.hpp"

namespace gls {

class Rng {
public:
    explicit Rng(std::uint64_t seed = 0x853c49e6748fea9bULL, std::uint64_t stream = 0xda3e39cb94b95bdbULL) {
        reseed(seed, stream);
    }
    void reseed(std::uint64_t seed, std::uint64_t stream = 0xda3e39cb94b95bdbULL) {
        state_ = 0;
        inc_ = (stream << 1u) | 1u;
        next();
        state_ += seed;
        next();
    }
    std::uint32_t next() {
        const std::uint64_t old = state_;
        state_ = old * 6364136223846793005ULL + inc_;
        const auto xorshifted = static_cast<std::uint32_t>(((old >> 18u) ^ old) >> 27u);
        const auto rot = static_cast<std::uint32_t>(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((32u - rot) & 31u));
    }
    // Uniform in [0, 1).
    Real uniform() { return static_cast<Real>(next()) * (1.0 / 4294967296.0); }
    Real uniform(Real lo, Real hi) { return lo + (hi - lo) * uniform(); }
    // Standard normal (Box-Muller, one value per call).
    Real normal() {
        const Real u1 = std::max(1e-12, uniform());
        const Real u2 = uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
    }
    // Two-parameter Weibull draw: scale * (-ln(1-U))^(1/m).
    Real weibull(Real scale, Real modulus) {
        const Real u = std::min(1.0 - 1e-12, uniform());
        return scale * std::pow(-std::log(1.0 - u), 1.0 / modulus);
    }

private:
    std::uint64_t state_ = 0, inc_ = 0;
};

}  // namespace gls
