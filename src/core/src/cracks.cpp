// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/cracks.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gls {

void CrackNetwork::configure(Real W, Real H, Real cell) {
    W_ = W;
    H_ = H;
    cell_ = cell;
    nx_ = std::max(2, static_cast<int>(std::ceil(W / cell)));
    ny_ = std::max(2, static_cast<int>(std::ceil(H / cell)));
    segmentLength = cell;
    clear();
}

void CrackNetwork::clear() {
    grid_.assign(static_cast<std::size_t>(nx_) * ny_, 0u);
    tips_.clear();
    segs_.clear();
    info_.assign(1, CrackInfo{});
    groups_ = 0;
    totalLength_ = 0;
    lastActivity_ = -1;
    ++revision_;
}

bool CrackNetwork::cellOf(Vec2 p, int& i, int& j) const {
    i = static_cast<int>(std::floor(p.x / cell_));
    j = static_cast<int>(std::floor(p.y / cell_));
    return i >= 0 && j >= 0 && i < nx_ && j < ny_;
}

int CrackNetwork::aliveTips() const {
    int n = 0;
    for (const auto& t : tips_) n += t.alive ? 1 : 0;
    return n;
}

int CrackNetwork::activeTips() const {
    int n = 0;
    for (const auto& t : tips_) n += (t.alive && !t.dormant) ? 1 : 0;
    return n;
}

void CrackNetwork::killDormant(Real t) {
    for (int k = 0; k < static_cast<int>(tips_.size()); ++k)
        if (tips_[k].alive && tips_[k].dormant) kill(k, t);
    compactTips();
}

int CrackNetwork::clearance(const CrackTip& t, int maxCells) const {
    const Vec2 side = perp(t.dir);
    const CrackInfo& me = info_[t.crack];
    for (int k = 1; k <= maxCells; ++k) {
        for (const Real sgn : {1.0, -1.0}) {
            const Vec2 p = t.pos + side * (sgn * k * cell_) + t.dir * (0.5 * cell_);
            int i, j;
            if (!cellOf(p, i, j)) continue;
            const std::uint32_t occ = at(i, j);
            if (occ == 0 || (occ & kRemoved) || (occ & kCrush)) continue;
            const std::uint32_t id = occ & kIdMask;
            if (id == t.crack || id == me.parent) continue;
            if (info_[id].parent == t.crack) continue;
            // Siblings from the same fork only count once they have separated.
            if (info_[id].group == me.group && length(p - me.birth) < 3 * cell_) continue;
            return k;
        }
    }
    return maxCells + 1;
}

int CrackNetwork::spawn(Vec2 p, Vec2 dir, Real t, CrackKind kind, std::uint32_t group, std::uint32_t parent,
                        Real speed) {
    if (aliveTips() >= maxTips || segs_.size() >= maxSegments) return -1;
    if (p.x < 0 || p.y < 0 || p.x > W_ || p.y > H_) return -1;
    if (info_.size() >= kIdMask) return -1;
    info_.push_back({parent, group, p});
    CrackTip tip;
    tip.pos = tip.vertex = p;
    tip.vertexTime = t;
    tip.dir = normalize(dir);
    tip.target = tip.dir;
    tip.speed = speed;
    tip.crack = static_cast<std::uint32_t>(info_.size() - 1);
    tip.kind = kind;
    tips_.push_back(tip);
    lastActivity_ = t;
    return static_cast<int>(tips_.size() - 1);
}

void CrackNetwork::emit(CrackTip& t, Vec2 to, Real time) {
    if (segs_.size() >= maxSegments) return;
    CrackSegment s;
    s.a = t.vertex;
    s.b = to;
    s.ta = static_cast<float>(t.vertexTime);
    s.tb = static_cast<float>(time);
    s.kind = t.kind;
    s.crack = t.crack;
    segs_.push_back(s);
    t.vertex = to;
    t.vertexTime = time;
    ++revision_;
    ++topology_;
}

void CrackNetwork::kill(int k, Real t) {
    CrackTip& tip = tips_[k];
    if (!tip.alive) return;
    if (length(tip.pos - tip.vertex) > 1e-9) emit(tip, tip.pos, t);
    tip.alive = false;
}

void CrackNetwork::branch(int k, Real half, Real t, Rng& rng) {
    if (aliveTips() + 1 > maxTips || segs_.size() + 4 >= maxSegments) {
        tips_[k].sinceBranch = 0;
        return;
    }
    kill(k, t);
    const CrackTip parent = tips_[k];
    const std::uint32_t group = info_[parent.crack].group;
    const Real a1 = half * rng.uniform(0.7, 1.3), a2 = half * rng.uniform(0.7, 1.3);
    const CrackKind kind = parent.kind == CrackKind::Dicing ? CrackKind::Dicing : CrackKind::Branch;
    for (const Real a : {a1, -a2}) {
        const int c = spawn(parent.pos, rotate(parent.dir, a), t, kind, group, parent.crack, parent.speed);
        if (c >= 0) {
            tips_[c].length = parent.length;
            tips_[c].stress = parent.stress;
            tips_[c].anisotropy = parent.anisotropy;
            tips_[c].target = tips_[c].dir;
        }
    }
}

bool CrackNetwork::ignorable(const CrackTip& t, std::uint32_t occ, Vec2 where) const {
    if (occ == 0) return true;
    if (occ & kRemoved) return false;
    const CrackInfo& me = info_[t.crack];
    if (occ & kCrush) return (occ & kIdMask) == me.group;
    const std::uint32_t id = occ & kIdMask;
    if (id == t.crack) return true;
    const Real exclusion = 2.5 * cell_;
    if (length(where - me.birth) < exclusion) {
        const CrackInfo& other = info_[id];
        if (id == me.parent || other.group == me.group) return true;
    }
    return false;
}

bool CrackNetwork::advance(int k, Vec2 to, Real time) {
    CrackTip& tip = tips_[k];
    if (!tip.alive) return false;
    bool edge = false;
    // Clip to the pane.
    {
        const Vec2 d = to - tip.pos;
        Real s = 1;
        if (to.x < 0) s = std::min(s, -tip.pos.x / d.x);
        if (to.x > W_) s = std::min(s, (W_ - tip.pos.x) / d.x);
        if (to.y < 0) s = std::min(s, -tip.pos.y / d.y);
        if (to.y > H_) s = std::min(s, (H_ - tip.pos.y) / d.y);
        if (s < 1) {
            to = tip.pos + d * std::max(0.0, s);
            edge = true;
        }
    }
    const Vec2 d = to - tip.pos;
    const Real len = length(d);
    if (len < 1e-12) {
        if (edge) kill(k, time);
        return !edge;
    }
    int i, j;
    cellOf(tip.pos, i, j);
    i = std::clamp(i, 0, nx_ - 1);
    j = std::clamp(j, 0, ny_ - 1);
    int ie, je;
    cellOf(to, ie, je);
    ie = std::clamp(ie, 0, nx_ - 1);
    je = std::clamp(je, 0, ny_ - 1);
    const std::uint32_t id = tip.crack;
    {
        std::uint32_t& c = at(i, j);
        if (c == 0) c = id;
    }
    const int sx = d.x > 0 ? 1 : -1, sy = d.y > 0 ? 1 : -1;
    constexpr Real inf = std::numeric_limits<Real>::infinity();
    Real tMaxX = d.x != 0 ? (((i + (sx > 0 ? 1 : 0)) * cell_) - tip.pos.x) / d.x : inf;
    Real tMaxY = d.y != 0 ? (((j + (sy > 0 ? 1 : 0)) * cell_) - tip.pos.y) / d.y : inf;
    const Real tDX = d.x != 0 ? cell_ / std::fabs(d.x) : inf;
    const Real tDY = d.y != 0 ? cell_ / std::fabs(d.y) : inf;
    int guard = nx_ + ny_ + 8;
    while ((i != ie || j != je) && guard-- > 0) {
        Real tEnter;
        if (tMaxX < tMaxY) { tEnter = tMaxX; tMaxX += tDX; i += sx; }
        else { tEnter = tMaxY; tMaxY += tDY; j += sy; }
        if (i < 0 || j < 0 || i >= nx_ || j >= ny_) break;
        std::uint32_t& c = at(i, j);
        const Vec2 where = cellCentre(i, j);
        if (!ignorable(tip, c, where)) {
            // Meet the other crack: run just into its cell so the walls join.
            const Real s = std::min(1.0, tEnter + 0.5 * cell_ / len);
            const Vec2 hit = tip.pos + d * s;
            totalLength_ += len * s;
            tip.length += len * s;
            tip.pos = hit;
            lastActivity_ = time;
            kill(k, time);
            return false;
        }
        if (c == 0) c = id;
    }
    tip.pos = to;
    tip.length += len;
    tip.sinceBranch += len;
    totalLength_ += len;
    lastActivity_ = time;
    if (length(tip.pos - tip.vertex) >= segmentLength) emit(tip, tip.pos, time);
    if (edge) {
        kill(k, time);
        return false;
    }
    return true;
}

void CrackNetwork::compactTips() {
    tips_.erase(std::remove_if(tips_.begin(), tips_.end(), [](const CrackTip& t) { return !t.alive; }), tips_.end());
}

void CrackNetwork::addRing(Vec2 c, Real r, Real t, CrackKind kind, std::uint32_t group) {
    const int n = std::clamp(static_cast<int>(2 * kPi * r / (0.7 * cell_)), 10, 64);
    const int k = spawn(c + Vec2{r, 0}, {0, 1}, t, kind, group, 0, 0);
    if (k < 0) return;
    const Real keep = segmentLength;
    segmentLength = 0;  // a vertex at every corner of the polygon
    for (int i = 1; i <= n; ++i) {
        const Real a = 2 * kPi * i / n;
        if (!advance(k, c + Vec2{std::cos(a), std::sin(a)} * r, t)) break;
    }
    segmentLength = keep;
    kill(k, t);
}

void CrackNetwork::markCrush(Vec2 c, Real r, std::uint32_t group) {
    const int i0 = std::max(0, static_cast<int>((c.x - r) / cell_));
    const int i1 = std::min(nx_ - 1, static_cast<int>((c.x + r) / cell_));
    const int j0 = std::max(0, static_cast<int>((c.y - r) / cell_));
    const int j1 = std::min(ny_ - 1, static_cast<int>((c.y + r) / cell_));
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) {
            if (length(cellCentre(i, j) - c) > r) continue;
            std::uint32_t& g = at(i, j);
            if (!(g & kRemoved)) g = kCrush | (group & kIdMask);
        }
    ++revision_;
    ++topology_;
}

void CrackNetwork::markRemovedCells(const std::vector<std::uint32_t>& cells) {
    for (const auto idx : cells)
        if (idx < grid_.size()) grid_[idx] |= kRemoved;
    ++revision_;
}

bool CrackNetwork::crackedNear(Vec2 p, Real r) const {
    const int i0 = std::max(0, static_cast<int>((p.x - r) / cell_));
    const int i1 = std::min(nx_ - 1, static_cast<int>((p.x + r) / cell_));
    const int j0 = std::max(0, static_cast<int>((p.y - r) / cell_));
    const int j1 = std::min(ny_ - 1, static_cast<int>((p.y + r) / cell_));
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) {
            const std::uint32_t g = at(i, j);
            if ((g & ~kRemoved) != 0) return true;
        }
    return false;
}

bool CrackNetwork::glassAt(Vec2 p) const {
    int i, j;
    if (!cellOf(p, i, j)) return false;
    return !(at(i, j) & kRemoved);
}

}  // namespace gls
