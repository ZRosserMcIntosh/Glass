// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// How a shattered tempered pane comes down. Once the dicing front has crossed
// it, the pane is a crazed sheet: thousands of dice with no strength between
// them, held in place only by friction where they press on each other and on
// the frame. A die stays while that friction carries its weight, and it
// carries most through the die below it (bearing), less sideways, little from
// above. So the sheet empties from the hole upward, edges crumble off for a
// while afterwards, and dice whose neighbours are going too leave together
// as slabs that burst when they land.
//
// Everything is procedural from the seed: the grip of every contact is drawn
// once, and every release follows from support and timing, so the same shot
// collapses the same way and a different shot differently.
#pragma once

#include <functional>
#include <queue>
#include <vector>

#include "gls/fragments.hpp"
#include "gls/math.hpp"
#include "gls/rng.hpp"

namespace gls {

struct CollapseParams {
    // Mean grip of a contact as a multiple of a typical die's weight per die
    // width of contact (estimated from footage: a holed pane empties in under
    // a second, leaving a fringe in the frame).
    Real grip = 1.3;
    // Share of a contact's grip that carries weight, by where the neighbour is.
    Real bearing = 1.0, side = 0.12, hang = 0.05;
    Real creepTime = 0.8;     // s, mean time before an exposed, lightly held die drops
    Real creepMargin = 2.5;   // an exposed die holding less than this many times its weight can crumble off
    // Friction needs the wedging of neighbours all round: a die's grip falls
    // as (neighbours still in place / neighbours it had)^confinement.
    Real confinement = 2.0;
    Real slabBond = 0.55;     // contacts gripping above this fraction of the mean pull slabs along
    int slabMax = 80;         // dice in one slab
    Real gapMin = 0.2e-3, gapMax = 1.5e-3;  // m a die slides before it clears its neighbours
};

class CrazedSheet {
public:
    using Release = std::function<void(int fragment)>;

    // dice: indices of the held pieces of one tempered ply (frame-held edge
    // dice included: their frame grip is part of their support). gone: those
    // already falling (the hole). Schedules the first checks.
    void begin(FragmentSystem& fs, const std::vector<int>& dice, const std::vector<char>& frameHeld,
               const std::vector<int>& gone, const Vec2& origin, Real frameGrip, Real time, Rng& rng,
               const CollapseParams& p = {});

    // Processes every check due by `time`; `release` frees a die (the pane
    // marks its cells as empty). Returns the number released.
    int step(FragmentSystem& fs, Real time, Rng& rng, const Release& release);

    // A new impact or a hard knock near `at` makes the dice there recheck.
    void disturb(Vec2 at, Real radius, Real time);

    bool active() const { return !events_.empty(); }
    int remaining() const;
    std::size_t size() const { return dice_.size(); }

private:
    struct Die {
        int frag = -1;
        Vec2 c;              // centroid in the pane
        Real weight = 0;     // N
        Real anchor = 0;     // N the frame grips it with
        bool inPlace = true;
        bool creepSet = false;
        int contacts = 0;    // neighbours it had
    };
    struct Bond {
        int other = -1;      // die index
        Real grip = 0;       // N
    };
    struct Event {
        Real time = 0;
        int die = -1;
        bool creep = false;
        bool operator>(const Event& o) const { return time > o.time; }
    };

    Real capacity(int d, const std::vector<char>* leaving = nullptr) const;
    void scheduleNeighbours(int d, Real time, Rng& rng);

    std::vector<Die> dice_;
    std::vector<std::vector<Bond>> bonds_;
    std::vector<int> dieOf_;  // fragment index -> die index (-1)
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events_;
    CollapseParams p_;
    Real meanGrip_ = 0;
};

}  // namespace gls
