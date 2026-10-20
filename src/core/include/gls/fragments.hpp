// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Fragments: the regions a crack network encloses, found by flood fill on the
// ply's occupancy grid and traced into polygons, then flown as rigid bodies
// (gravity, air drag, a floor with restitution and friction). A fragment that
// lands hard enough breaks again along a random line through its centroid.
#pragma once

#include <cstdint>
#include <vector>

#include "gls/cracks.hpp"
#include "gls/math.hpp"
#include "gls/rng.hpp"

namespace gls {

enum class FragmentState : std::uint8_t { Held = 0, Pending = 1, Free = 2, Resting = 3 };

struct Fragment {
    int ply = 0;
    std::vector<Vec2> outline;  // polygon about the centroid, m (pane-aligned at birth)
    Vec2 origin;                // centroid in the pane at birth
    Real area = 0, mass = 0, thickness = 0;
    Vec3 inertia;               // principal moments in the body frame, kg*m^2
    Vec3 pos, vel, omega;       // pane frame: centroid, m/s, rad/s
    Quat rot;
    FragmentState state = FragmentState::Held;
    Real releaseTime = 0;
    Real edgeContact = 0;       // m of pane edge it touches
    Real radius = 0;            // bounding radius, m
    Real bornAt = 0;            // s
    int generation = 0;
    int calm = 0;               // consecutive slow floor contacts (settling)
    int clump = -1;             // index of the clump it falls with, or -1
    Vec3 clumpOffset;           // centroid relative to the clump's, clump frame
    Real diceSize = 0;          // m, real tempered dice size of its ply (0: not tempered)
    Real freeAt = -1;  // s, when it last came free (a coarse tempered piece crumbles soon after)
    bool chip = false;          // spall / crush debris, not traced from a crack network
    bool retired = false;       // replaced by the pieces it broke into
    std::uint8_t edges = 0;     // pane edges touched: 1 left, 2 right, 4 bottom, 8 top
    std::uint32_t id = 0;
};

// Pieces that touch, and how long the shared boundary is (from build()).
struct FragmentLink {
    int other = -1;
    Real length = 0;  // m
};

// A slab of crazed tempered glass falling as one before it bursts.
struct Clump {
    std::vector<int> members;
    Vec3 pos, vel, omega;  // centre of mass, pane frame
    Quat rot;
    Real mass = 0, radius = 0;
    Vec3 inertia;
    Real breakupAt = 0;    // s, when it comes apart in the air if it has not landed
    bool alive = false;
};

// A coarse piece of tempered glass that landed hard: renderers scatter
// `count` real-size dice here (the physics drew fewer, larger ones).
struct DiceSpray {
    Vec3 pos, vel;
    int count = 0;
    Real size = 0, thickness = 0, time = 0;
    int ply = 0;
};

class FragmentSystem {
public:
    // Labels the ply's enclosed regions (cells: 4-connected flood fill, crack
    // cells dilated into their neighbours) and appends a fragment per region.
    // Cells of regions that become fragments are recorded per ply so a later
    // impact can find which fragment it hits. Returns the number added.
    int build(const CrackNetwork& net, int ply, Real zMid, Real thickness, Real density, Real time);

    // Index of the fragment covering pane point p in ply, or -1.
    int fragmentAt(int ply, Vec2 p) const;

    // Debris chip: a small random convex polygon.
    int addChip(int ply, Vec2 at, Real z, Real size, Real thickness, Real density, const Vec3& vel, Real time,
                Rng& rng);

    void step(Real dt, Real time, Rng& rng);

    // Makes the given (pane-aligned) pieces fall as one rigid slab.
    int makeClump(const std::vector<int>& members, const Vec3& vel, const Vec3& omega, Real time, Rng& rng);
    void dissolve(int clump, Real time, Rng& rng, Real spread);
    std::vector<Clump>& clumps() { return clumps_; }
    const std::vector<FragmentLink>& links(int fragment) const;
    const std::vector<DiceSpray>& sprays() const { return sprays_; }

    std::vector<Fragment>& fragments() { return frags_; }
    const std::vector<Fragment>& fragments() const { return frags_; }
    void clear();

    Real floorY = -0.9;           // m, pane frame (pane bottom edge at y = 0)
    Real restitution = 0.25, friction = 0.45;
    int maxFragments = 6000;
    Real breakEnergyFactor = 40;  // landing energy over crack surface energy needed to break
    Real toughnessEnergy = 7.8;   // J/m^2, G_Ic = K_Ic^2 / E of the glass
    std::uint64_t revision = 0;   // bumps when fragments are added or split

    // Per ply: fragment index of every grid cell (-1 none), from build().
    struct PlyMap {
        int nx = 0, ny = 0;
        Real cell = 0;
        std::vector<std::int32_t> label;
    };
    std::vector<PlyMap> maps;

private:
    void split(std::size_t k, Rng& rng, Real time);
    void stepClumps(Real dt, Real time, Rng& rng);
    std::vector<Fragment> frags_;
    std::vector<std::vector<FragmentLink>> links_;
    std::vector<Clump> clumps_;
    std::vector<DiceSpray> sprays_;
    std::uint32_t nextId_ = 1;
};

// Area-weighted centroid and area of a simple polygon.
Real polygonArea(const std::vector<Vec2>& p, Vec2* centroid = nullptr);

}  // namespace gls
