// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// The crack pattern of one glass ply: through-thickness cracks as polylines
// grown tip by tip, with every vertex time-stamped (a renderer can replay the
// fracture at any speed), and an occupancy grid so a running crack stops when
// it meets another one, as cracks in glass do (they cannot cross a free
// surface).
#pragma once

#include <cstdint>
#include <vector>

#include "gls/math.hpp"
#include "gls/rng.hpp"

namespace gls {

enum class CrackKind : std::uint8_t { Radial = 0, Concentric = 1, Branch = 2, Dicing = 3, Edge = 4, Cone = 5 };

struct CrackSegment {
    Vec2 a, b;
    float ta = 0, tb = 0;  // s, when each end formed
    CrackKind kind = CrackKind::Radial;
    std::uint32_t crack = 0;
};

struct CrackTip {
    Vec2 pos, dir;
    Vec2 vertex;            // last emitted vertex
    Real vertexTime = 0;
    Real speed = 0;         // m/s
    Real sinceBranch = 0;   // m run since birth or the last fork
    Real length = 0;        // m run in total
    Real stress = 0;        // cached driving stress, Pa
    Vec2 target;            // cached preferred direction
    Real anisotropy = 0;    // 0 isotropic .. 1 uniaxial
    int evalAge = 1 << 20;  // steps since the stress was evaluated
    Vec2 anchor;            // ring cracks: the centre they run around
    bool fixedStress = false;  // ring cracks: driven by the wedge-root stress, not the plate
    bool dormant = false;      // arrested; restarts if the stress comes back
    Real nextCheck = 0;        // s, when a dormant tip is looked at again
    Real shield = 1;           // stress relief from a nearby parallel crack (1 none)
    Real kink = 0;             // rad still to turn (maximum hoop stress criterion)
    bool radial = false;       // driven outward by the wedges of a crack star
    Real wob = 0;              // rad, slowly wandering offset from the stress-preferred path
    std::uint32_t crack = 0;
    CrackKind kind = CrackKind::Radial;
    bool alive = true;
};

struct CrackInfo {
    std::uint32_t parent = 0, group = 0;
    Vec2 birth;
};

class CrackNetwork {
public:
    static constexpr std::uint32_t kRemoved = 1u << 31;  // glass gone
    static constexpr std::uint32_t kCrush = 1u << 30;    // comminuted (cone / crush zone)
    static constexpr std::uint32_t kIdMask = (1u << 30) - 1;

    void configure(Real width, Real height, Real cell);
    void clear();

    Real width() const { return W_; }
    Real height() const { return H_; }
    Real cell() const { return cell_; }
    int nx() const { return nx_; }
    int ny() const { return ny_; }
    std::uint32_t at(int i, int j) const { return grid_[static_cast<std::size_t>(j) * nx_ + i]; }
    std::uint32_t& at(int i, int j) { return grid_[static_cast<std::size_t>(j) * nx_ + i]; }
    const std::vector<std::uint32_t>& grid() const { return grid_; }
    bool cellOf(Vec2 p, int& i, int& j) const;
    Vec2 cellCentre(int i, int j) const { return {(i + 0.5) * cell_, (j + 0.5) * cell_}; }

    std::uint32_t newGroup() { return ++groups_; }
    // A new crack (tip) at p heading along dir; returns the tip index or -1
    // when over budget.
    int spawn(Vec2 p, Vec2 dir, Real t, CrackKind kind, std::uint32_t group, std::uint32_t parent, Real speed);
    // Forks a tip into two at +-halfAngle; the parent tip ends.
    void branch(int tip, Real halfAngle, Real t, Rng& rng);
    // Moves a tip to `to` at time t, marking the cells it crosses. Returns
    // false (and kills the tip) when it meets another crack, a crushed or
    // missing region, or the pane edge.
    bool advance(int tip, Vec2 to, Real t);
    void kill(int tip, Real t);
    void compactTips();

    // A closed ring crack (the surface trace of a Hertzian cone).
    void addRing(Vec2 c, Real r, Real t, CrackKind kind, std::uint32_t group);
    // A crushed disc (Hertzian cone, ballistic crush zone), stamped with group.
    // Crushed glass in a ragged disc: radius r (1 + sum a_k cos((k + 2) theta + ph_k)).
    void markCrush(Vec2 c, Real r, std::uint32_t group, const Real* a = nullptr, const Real* ph = nullptr, int n = 0);
    void markRemovedCells(const std::vector<std::uint32_t>& cells);
    bool crackedNear(Vec2 p, Real r) const;  // any crack or crush within r
    bool glassAt(Vec2 p) const;

    std::vector<CrackTip>& tips() { return tips_; }
    const std::vector<CrackTip>& tips() const { return tips_; }
    const std::vector<CrackSegment>& segments() const { return segs_; }
    const std::vector<CrackInfo>& cracks() const { return info_; }
    int aliveTips() const;   // running or dormant
    int activeTips() const;  // running
    void killDormant(Real t);
    // Distance (cells, up to maxCells) from a tip sideways to the nearest
    // other crack: a crack running alongside another is shielded by it.
    int clearance(const CrackTip& t, int maxCells) const;
    Real totalLength() const { return totalLength_; }
    Real lastActivity() const { return lastActivity_; }
    std::uint64_t revision() const { return revision_; }        // anything changed (renderers)
    std::uint64_t topology() const { return topology_; }        // cracks or crush changed

    // Budgets: the network stops forking or spawning past these.
    int maxTips = 512;
    std::size_t maxSegments = 200000;
    Real segmentLength = 0.002;  // m between emitted vertices

private:
    void emit(CrackTip& t, Vec2 to, Real time);
    bool ignorable(const CrackTip& t, std::uint32_t occupant, Vec2 where) const;

    Real W_ = 1, H_ = 1, cell_ = 0.002;
    int nx_ = 0, ny_ = 0;
    std::vector<std::uint32_t> grid_;
    std::vector<CrackTip> tips_;
    std::vector<CrackSegment> segs_;
    std::vector<CrackInfo> info_;  // index = crack id (0 unused)
    std::uint32_t groups_ = 0;
    Real totalLength_ = 0, lastActivity_ = -1;
    std::uint64_t revision_ = 0, topology_ = 0;
};

}  // namespace gls
