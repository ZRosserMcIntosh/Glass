// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/collapse.hpp"

#include <algorithm>
#include <cmath>

namespace gls {

void CrazedSheet::begin(FragmentSystem& fs, const std::vector<int>& dice, const std::vector<char>& frameHeld,
                        const std::vector<int>& gone, const Vec2& origin, Real frameGrip, Real time, Rng& rng,
                        const CollapseParams& p) {
    p_ = p;
    const auto& fr = fs.fragments();
    dieOf_.assign(fr.size(), -1);
    dice_.clear();
    bonds_.clear();
    Real wSum = 0, aSum = 0;
    for (std::size_t i = 0; i < dice.size(); ++i) {
        const Fragment& f = fr[dice[i]];
        Die d;
        d.frag = dice[i];
        d.c = f.origin;
        d.weight = f.mass * kGravity;
        d.anchor = frameHeld[i] ? frameGrip * f.edgeContact + ((f.edges & 4) ? 10 * d.weight : 0) : 0;
        dieOf_[dice[i]] = static_cast<int>(dice_.size());
        dice_.push_back(d);
        wSum += d.weight;
        aSum += std::sqrt(f.area);
    }
    if (dice_.empty()) return;
    const Real n = static_cast<Real>(dice_.size());
    const Real wMean = wSum / n, aMean = aSum / n;
    meanGrip_ = p_.grip * wMean;
    // One grip per contact, drawn once: friction where two dice press.
    bonds_.assign(dice_.size(), {});
    for (std::size_t i = 0; i < dice_.size(); ++i) {
        for (const auto& l : fs.links(dice_[i].frag)) {
            const int j = l.other < static_cast<int>(dieOf_.size()) ? dieOf_[l.other] : -1;
            if (j < 0 || j <= static_cast<int>(i)) continue;
            const Real g = meanGrip_ * (l.length / aMean) * std::pow(rng.uniform(0.05, 1.0), 1.5) * 2.0;
            bonds_[i].push_back({j, g});
            bonds_[j].push_back({static_cast<int>(i), g});
            ++dice_[i].contacts;
            ++dice_[j].contacts;
        }
    }
    // The hole: dice already falling leave their neighbours to fend for themselves.
    for (const int g : gone)
        if (g >= 0 && g < static_cast<int>(dieOf_.size()) && dieOf_[g] >= 0) {
            dice_[dieOf_[g]].inPlace = false;
        }
    Real holeR = 0;
    for (const int g : gone)
        if (g >= 0 && g < static_cast<int>(dieOf_.size()) && dieOf_[g] >= 0) {
            scheduleNeighbours(dieOf_[g], time, rng);
            holeR = std::max(holeR, length(dice_[dieOf_[g]].c - origin) + fr[g].radius);
        }
    // Crushed glass round the hole touches nothing: the dice on its rim have
    // lost their support without sharing a crack with anything that fell.
    if (holeR > 0)
        for (std::size_t i = 0; i < dice_.size(); ++i)
            if (dice_[i].inPlace && length(dice_[i].c - origin) < holeR + 1.5 * aMean)
                events_.push({time + std::sqrt(2 * rng.uniform(p_.gapMin, p_.gapMax) / kGravity), static_cast<int>(i), false});
    // With no hole the sheet still settles: the dice nearest the origin go first.
    if (gone.empty()) {
        int nearest = -1;
        Real best = 1e9;
        for (std::size_t i = 0; i < dice_.size(); ++i) {
            const Real d = length(dice_[i].c - origin);
            if (d < best && dice_[i].anchor == 0) { best = d; nearest = static_cast<int>(i); }
        }
        if (nearest >= 0) events_.push({time + 0.02 + 0.05 * rng.uniform(), nearest, true});
    }
}

Real CrazedSheet::capacity(int d, const std::vector<char>* leaving) const {
    const Die& me = dice_[d];
    Real cap = 0;
    int left = 0;
    for (const auto& b : bonds_[d]) {
        const Die& o = dice_[b.other];
        if (!o.inPlace || (leaving && (*leaving)[b.other])) continue;
        ++left;
        const Vec2 rel = o.c - me.c;
        const Real l = length(rel);
        const Real dy = l > 0 ? rel.y / l : 0;
        // Bearing from below, friction from the sides, a little hanging from above.
        const Real share = dy < -0.4 ? p_.bearing : dy > 0.4 ? p_.hang : p_.side;
        cap += b.grip * share;
    }
    const Real conf = me.contacts ? std::pow(static_cast<Real>(left) / me.contacts, p_.confinement) : 1.0;
    return me.anchor + cap * conf;
}

void CrazedSheet::scheduleNeighbours(int d, Real time, Rng& rng) {
    for (const auto& b : bonds_[d]) {
        Die& o = dice_[b.other];
        if (!o.inPlace) continue;
        // The time to slide clear of a gap g under gravity, sqrt(2 g / 9.81).
        const Real gap = rng.uniform(p_.gapMin, p_.gapMax);
        events_.push({time + std::sqrt(2 * gap / kGravity) + 0.006 * rng.uniform(), b.other, false});
        // A die that now has an open side may crumble off later on its own.
        if (!o.creepSet && o.anchor == 0) {
            o.creepSet = true;
            const Real margin = capacity(b.other) / std::max(o.weight, 1e-9);
            const Real mean = p_.creepTime * clampr(margin, 0.3, 6.0) / 2;
            events_.push({time + 0.05 - mean * std::log(std::max(1e-9, rng.uniform())), b.other, true});
        }
    }
}

int CrazedSheet::step(FragmentSystem& fs, Real time, Rng& rng, const Release& release) {
    int released = 0;
    std::vector<char> leaving;
    while (!events_.empty() && events_.top().time <= time) {
        const Event e = events_.top();
        events_.pop();
        Die& d = dice_[e.die];
        if (!d.inPlace) continue;
        const bool falls = e.creep ? capacity(e.die) < p_.creepMargin * d.weight : capacity(e.die) < d.weight;
        if (!falls) continue;
        // A slab: neighbours gripping it hard, and that would not stay up
        // without it, come down with it.
        std::vector<int> slab{e.die};
        if (!e.creep) {
            leaving.assign(dice_.size(), 0);
            leaving[e.die] = 1;
            for (std::size_t k = 0; k < slab.size() && static_cast<int>(slab.size()) < p_.slabMax; ++k) {
                for (const auto& b : bonds_[slab[k]]) {
                    if (leaving[b.other] || !dice_[b.other].inPlace || dice_[b.other].anchor > 0) continue;
                    if (b.grip < p_.slabBond * meanGrip_) continue;
                    if (capacity(b.other, &leaving) >= dice_[b.other].weight) continue;
                    leaving[b.other] = 1;
                    slab.push_back(b.other);
                    if (static_cast<int>(slab.size()) >= p_.slabMax) break;
                }
            }
        }
        std::vector<int> frags;
        for (const int s : slab) {
            dice_[s].inPlace = false;
            frags.push_back(dice_[s].frag);
            release(dice_[s].frag);
            ++released;
        }
        // Dice tip out of the plane as they go, some forward, some back.
        const Vec3 v{rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.02), rng.uniform(-0.12, 0.12)};
        const Vec3 w{rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-1, 1)};
        if (frags.size() > 1) {
            fs.makeClump(frags, v, w * 0.5, time, rng);
        } else {
            Fragment& f = fs.fragments()[frags[0]];
            f.vel = v;
            f.omega = w;
        }
        for (const int s : slab) scheduleNeighbours(s, time, rng);
    }
    return released;
}

void CrazedSheet::sync(const FragmentSystem& fs) {
    for (auto& d : dice_)
        if (d.inPlace && fs.fragments()[d.frag].state != FragmentState::Held) d.inPlace = false;
}

void CrazedSheet::disturb(Vec2 at, Real radius, Real time) {
    for (std::size_t i = 0; i < dice_.size(); ++i)
        if (dice_[i].inPlace && length(dice_[i].c - at) < radius)
            events_.push({time + 0.001 * static_cast<Real>(i % 7), static_cast<int>(i), true});
}

int CrazedSheet::remaining() const {
    int n = 0;
    for (const auto& d : dice_) n += d.inPlace ? 1 : 0;
    return n;
}

}  // namespace gls
