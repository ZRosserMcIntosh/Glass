// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/fragments.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace gls {

Real polygonArea(const std::vector<Vec2>& p, Vec2* centroid) {
    Real a = 0, cx = 0, cy = 0;
    const std::size_t n = p.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& u = p[i];
        const Vec2& v = p[(i + 1) % n];
        const Real c = cross(u, v);
        a += c;
        cx += (u.x + v.x) * c;
        cy += (u.y + v.y) * c;
    }
    a *= 0.5;
    if (centroid) *centroid = std::fabs(a) > 1e-30 ? Vec2{cx / (6 * a), cy / (6 * a)} : (n ? p[0] : Vec2{});
    return a;
}

namespace {

// Second moments of area about the origin: (int y^2, int x^2).
Vec2 polygonMoments(const std::vector<Vec2>& p) {
    Real ixx = 0, iyy = 0;
    const std::size_t n = p.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& u = p[i];
        const Vec2& v = p[(i + 1) % n];
        const Real c = cross(u, v);
        ixx += c * (u.y * u.y + u.y * v.y + v.y * v.y);
        iyy += c * (u.x * u.x + u.x * v.x + v.x * v.x);
    }
    return {std::fabs(ixx) / 12, std::fabs(iyy) / 12};
}

Real segDistance(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 d = b - a;
    const Real l2 = dot(d, d);
    const Real t = l2 > 0 ? clampr(dot(p - a, d) / l2, 0, 1) : 0;
    return length(p - (a + d * t));
}

void douglasPeucker(const std::vector<Vec2>& in, std::size_t i0, std::size_t i1, Real tol, std::vector<char>& keep) {
    if (i1 <= i0 + 1) return;
    Real best = -1;
    std::size_t at = i0;
    for (std::size_t i = i0 + 1; i < i1; ++i) {
        const Real d = segDistance(in[i], in[i0], in[i1]);
        if (d > best) { best = d; at = i; }
    }
    if (best > tol) {
        keep[at] = 1;
        douglasPeucker(in, i0, at, tol, keep);
        douglasPeucker(in, at, i1, tol, keep);
    }
}

std::vector<Vec2> simplifyClosed(const std::vector<Vec2>& loop, Real tol) {
    const std::size_t n = loop.size();
    if (n <= 4) return loop;
    // Split at the vertex farthest from the first.
    std::size_t far = 0;
    Real best = -1;
    for (std::size_t i = 1; i < n; ++i) {
        const Real d = length(loop[i] - loop[0]);
        if (d > best) { best = d; far = i; }
    }
    std::vector<Vec2> ring(loop);
    ring.push_back(loop[0]);
    std::vector<char> keep(ring.size(), 0);
    keep[0] = keep[far] = keep[ring.size() - 1] = 1;
    douglasPeucker(ring, 0, far, tol, keep);
    douglasPeucker(ring, far, ring.size() - 1, tol, keep);
    std::vector<Vec2> out;
    for (std::size_t i = 0; i + 1 < ring.size(); ++i)
        if (keep[i]) out.push_back(ring[i]);
    return out.size() >= 3 ? out : loop;
}

// Sutherland-Hodgman: the part of polygon p with dot(x, n) >= 0.
std::vector<Vec2> clipHalf(const std::vector<Vec2>& p, Vec2 n) {
    std::vector<Vec2> out;
    const std::size_t m = p.size();
    for (std::size_t i = 0; i < m; ++i) {
        const Vec2 a = p[i], b = p[(i + 1) % m];
        const Real da = dot(a, n), db = dot(b, n);
        if (da >= 0) out.push_back(a);
        if ((da >= 0) != (db >= 0)) out.push_back(a + (b - a) * (da / (da - db)));
    }
    return out;
}

Vec3 bodyInverse(const Quat& q, const Vec3& I, const Vec3& x) {
    const Quat c{q.w, -q.x, -q.y, -q.z};
    const Vec3 b = c.rotate(x);
    return q.rotate({b.x / I.x, b.y / I.y, b.z / I.z});
}

void finishFragment(Fragment& f, Real density) {
    Vec2 c;
    const Real a = polygonArea(f.outline, &c);
    if (a < 0) std::reverse(f.outline.begin(), f.outline.end());
    // Holes: clockwise, and taken out of the area and the centroid.
    Real area = std::fabs(a);
    Vec2 sum = c * area;
    for (auto& h : f.holes) {
        Vec2 ch;
        const Real ah = polygonArea(h, &ch);
        if (ah > 0) std::reverse(h.begin(), h.end());
        area -= std::fabs(ah);
        sum = sum - ch * std::fabs(ah);
    }
    if (area > 1e-12) c = sum / area;
    else area = std::fabs(a);
    for (auto& p : f.outline) p -= c;
    for (auto& h : f.holes)
        for (auto& p : h) p -= c;
    f.area = area;
    f.mass = std::max(1e-7, density * f.thickness * f.area);
    Vec2 mom = polygonMoments(f.outline);
    for (const auto& h : f.holes) {
        std::vector<Vec2> ccw(h.rbegin(), h.rend());
        mom = mom - polygonMoments(ccw);
    }
    const Real tt = f.thickness * f.thickness / 12;
    f.inertia = {density * f.thickness * mom.x + f.mass * tt, density * f.thickness * mom.y + f.mass * tt,
                 density * f.thickness * (mom.x + mom.y)};
    f.inertia.x = std::max(f.inertia.x, 1e-14);
    f.inertia.y = std::max(f.inertia.y, 1e-14);
    f.inertia.z = std::max(f.inertia.z, 1e-14);
    f.radius = 0;
    for (const auto& p : f.outline) f.radius = std::max(f.radius, length(p));
    f.origin = f.origin + c;
}

}  // namespace

void FragmentSystem::clear() {
    frags_.clear();
    links_.clear();
    clumps_.clear();
    sprays_.clear();
    maps.clear();
    nextId_ = 1;
    ++revision;
}

int FragmentSystem::build(const CrackNetwork& net, int ply, Real zMid, Real thickness, Real density, Real time) {
    const int nx = net.nx(), ny = net.ny();
    const Real cell = net.cell();
    const auto& g = net.grid();
    const std::size_t N = g.size();
    if (static_cast<int>(maps.size()) <= ply) maps.resize(ply + 1);
    PlyMap& pm = maps[ply];
    if (pm.label.size() != N) {
        pm.nx = nx; pm.ny = ny; pm.cell = cell;
        pm.label.assign(N, -1);
    }

    auto fillable = [&](std::uint32_t c) {
        return !(c & CrackNetwork::kRemoved) && !(c & CrackNetwork::kCrush) && (c & CrackNetwork::kIdMask) == 0;
    };
    // 1. Flood fill the uncracked cells.
    std::vector<std::int32_t> lab(N, -1);
    std::vector<std::int32_t> queue;
    queue.reserve(1024);
    int regions = 0;
    for (std::size_t s = 0; s < N; ++s) {
        if (lab[s] >= 0 || !fillable(g[s])) continue;
        lab[s] = regions;
        queue.clear();
        queue.push_back(static_cast<std::int32_t>(s));
        for (std::size_t h = 0; h < queue.size(); ++h) {
            const int c = queue[h], i = c % nx, j = c / nx;
            const int nb[4] = {i > 0 ? c - 1 : -1, i < nx - 1 ? c + 1 : -1, j > 0 ? c - nx : -1, j < ny - 1 ? c + nx : -1};
            for (const int q : nb)
                if (q >= 0 && lab[q] < 0 && fillable(g[q])) { lab[q] = regions; queue.push_back(q); }
        }
        ++regions;
    }
    // 2. Dilate into crack cells so neighbouring pieces tile the pane.
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<std::int32_t> next = lab;
        for (std::size_t c = 0; c < N; ++c) {
            if (lab[c] >= 0 || (g[c] & CrackNetwork::kRemoved) || (g[c] & CrackNetwork::kCrush)) continue;
            const int i = static_cast<int>(c) % nx, j = static_cast<int>(c) / nx;
            const int nb[4] = {i > 0 ? int(c) - 1 : -1, i < nx - 1 ? int(c) + 1 : -1, j > 0 ? int(c) - nx : -1,
                               j < ny - 1 ? int(c) + nx : -1};
            for (const int q : nb)
                if (q >= 0 && lab[q] >= 0) { next[c] = lab[q]; break; }
        }
        lab.swap(next);
    }
    // 3. Which regions are unchanged pieces that already exist.
    std::vector<int> count(regions, 0), owner(regions, -2);
    for (std::size_t c = 0; c < N; ++c) {
        if (lab[c] < 0) continue;
        ++count[lab[c]];
        const int old = pm.label[c];
        int& o = owner[lab[c]];
        if (o == -2) o = old;
        else if (o != old) o = -1;
    }
    std::vector<int> oldCount;
    {
        int maxOld = -1;
        for (auto v : pm.label) maxOld = std::max(maxOld, static_cast<int>(v));
        oldCount.assign(maxOld + 1, 0);
        for (auto v : pm.label)
            if (v >= 0) ++oldCount[v];
    }
    std::vector<char> isNew(regions, 1);
    for (int r = 0; r < regions; ++r)
        if (owner[r] >= 0 && oldCount[owner[r]] == count[r]) isNew[r] = 0;
    // 3b. Over the piece budget: merge the smallest new regions into a
    // neighbour until they fit, so the whole pane is still covered (coarser
    // pieces, never missing ones). A few slots stay free for chips and splits.
    {
        int newCount = 0;
        for (int r = 0; r < regions; ++r) newCount += isNew[r] && count[r] > 0;
        // Pieces in play: a retired record (split into others) is not one.
        int live = 0;
        for (const auto& f : frags_) live += !f.retired;
        const int room = std::max(1, maxFragments - live - std::max(8, maxFragments / 20));
        if (newCount > room) {
            std::vector<int> parent(regions);
            for (int r = 0; r < regions; ++r) parent[r] = r;
            auto find = [&](int r) {
                while (parent[r] != r) r = parent[r] = parent[parent[r]];
                return r;
            };
            std::vector<std::vector<int>> adj(regions);
            for (int j = 0; j < ny; ++j)
                for (int i = 0; i < nx; ++i) {
                    const int a = lab[j * nx + i];
                    if (a < 0 || !isNew[a]) continue;
                    if (i + 1 < nx) { const int b = lab[j * nx + i + 1]; if (b >= 0 && b != a && isNew[b]) adj[a].push_back(b); }
                    if (j + 1 < ny) { const int b = lab[(j + 1) * nx + i]; if (b >= 0 && b != a && isNew[b]) adj[a].push_back(b); }
                }
            for (int r = 0; r < regions; ++r)
                for (const int b : std::vector<int>(adj[r])) adj[b].push_back(r);
            for (auto& v : adj) { std::sort(v.begin(), v.end()); v.erase(std::unique(v.begin(), v.end()), v.end()); }
            std::vector<int> size(count);
            std::vector<int> order;
            for (int r = 0; r < regions; ++r)
                if (isNew[r] && count[r] > 0) order.push_back(r);
            // Repeated sweeps, smallest first, until the count fits.
            for (int sweep = 0; sweep < 8 && newCount > room; ++sweep) {
                std::sort(order.begin(), order.end(), [&](int a, int b) { return size[find(a)] < size[find(b)]; });
                for (const int r0 : order) {
                    if (newCount <= room) break;
                    const int r = find(r0);
                    if (r != r0) continue;  // visit each root once per sweep
                    int best = -1;
                    for (const int b : adj[r]) {
                        const int rb = find(b);
                        if (rb != r && (best < 0 || size[rb] < size[best])) best = rb;
                    }
                    if (best < 0) continue;
                    parent[r] = best;
                    size[best] += size[r];
                    for (const int b : adj[r]) adj[best].push_back(b);
                    --newCount;
                }
            }
            for (std::size_t c = 0; c < N; ++c)
                if (lab[c] >= 0 && isNew[lab[c]]) lab[c] = find(lab[c]);
            std::fill(count.begin(), count.end(), 0);
            for (std::size_t c = 0; c < N; ++c)
                if (lab[c] >= 0) ++count[lab[c]];
        }
    }
    // Old pieces that were split are retired.
    for (std::size_t c = 0; c < N; ++c) {
        if (lab[c] < 0 || !isNew[lab[c]]) continue;
        const int old = pm.label[c];
        if (old >= 0 && old < static_cast<int>(frags_.size()) && !frags_[old].retired) {
            frags_[old].retired = true;
            frags_[old].outline.clear();
        }
    }

    // 4. Boundary edges of every new region, region on the left (CCW).
    struct Edge {
        std::int32_t region;
        std::int32_t from, to;
    };
    std::vector<Edge> edges;
    std::vector<Real> edgeLen(regions, 0);
    std::vector<std::uint8_t> edgeMask(regions, 0);
    const int cx = nx + 1;
    auto corner = [&](int i, int j) { return j * cx + i; };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int c = j * nx + i, r = lab[c];
            if (r < 0 || !isNew[r]) continue;
            auto other = [&](int ii, int jj) {
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) return -1;
                return static_cast<int>(lab[jj * nx + ii]);
            };
            if (other(i, j - 1) != r) edges.push_back({r, corner(i, j), corner(i + 1, j)});
            if (other(i + 1, j) != r) edges.push_back({r, corner(i + 1, j), corner(i + 1, j + 1)});
            if (other(i, j + 1) != r) edges.push_back({r, corner(i + 1, j + 1), corner(i, j + 1)});
            if (other(i - 1, j) != r) edges.push_back({r, corner(i, j + 1), corner(i, j)});
            if (i == 0) { edgeLen[r] += cell; edgeMask[r] |= 1; }
            if (i == nx - 1) { edgeLen[r] += cell; edgeMask[r] |= 2; }
            if (j == 0) { edgeLen[r] += cell; edgeMask[r] |= 4; }
            if (j == ny - 1) { edgeLen[r] += cell; edgeMask[r] |= 8; }
        }
    }
    std::sort(edges.begin(), edges.end(),
              [](const Edge& a, const Edge& b) { return std::tie(a.region, a.from) < std::tie(b.region, b.from); });

    // 5. Chain each region's edges into loops; the largest is the outline.
    std::vector<int> regionIndex(regions, -1);
    int added = 0;
    std::size_t e0 = 0;
    std::vector<char> used(edges.size(), 0);
    const Real W = net.width(), H = net.height();
    while (e0 < edges.size()) {
        std::size_t e1 = e0;
        while (e1 < edges.size() && edges[e1].region == edges[e0].region) ++e1;
        const int r = edges[e0].region;
        std::vector<Vec2> best;
        Real bestArea = 0;
        std::vector<std::vector<Vec2>> loops;
        for (std::size_t s = e0; s < e1; ++s) {
            if (used[s]) continue;
            std::vector<Vec2> loop;
            std::size_t cur = s;
            int guard = static_cast<int>(e1 - e0) + 2;
            while (!used[cur] && guard-- > 0) {
                used[cur] = 1;
                const int from = edges[cur].from;
                loop.push_back({std::min(W, (from % cx) * cell), std::min(H, (from / cx) * cell)});
                const int to = edges[cur].to;
                // Next unused edge starting at `to`.
                auto it = std::lower_bound(edges.begin() + e0, edges.begin() + e1, to,
                                           [](const Edge& e, int v) { return e.from < v; });
                std::size_t nxt = cur;
                for (; it != edges.begin() + e1 && it->from == to; ++it) {
                    const std::size_t k = static_cast<std::size_t>(it - edges.begin());
                    if (!used[k]) { nxt = k; break; }
                }
                if (nxt == cur) break;
                cur = nxt;
            }
            const Real a = std::fabs(polygonArea(loop));
            if (a > bestArea) {
                if (best.size() >= 3) loops.push_back(std::move(best));
                bestArea = a;
                best = std::move(loop);
            } else if (loop.size() >= 3) {
                loops.push_back(std::move(loop));
            }
        }
        e0 = e1;
        if (best.size() < 3 || static_cast<int>(frags_.size()) >= maxFragments) continue;
        Fragment f;
        f.ply = ply;
        f.thickness = thickness;
        f.outline = simplifyClosed(best, 0.75 * cell);
        // The other loops are holes through it (worth keeping above a few cells).
        for (auto& l : loops)
            if (std::fabs(polygonArea(l)) > 4 * cell * cell) f.holes.push_back(simplifyClosed(l, 0.75 * cell));
        f.origin = {0, 0};
        finishFragment(f, density);
        f.pos = {f.origin.x, f.origin.y, zMid};
        f.edgeContact = edgeLen[r];
        f.edges = edgeMask[r];
        f.bornAt = time;
        f.id = nextId_++;
        regionIndex[r] = static_cast<int>(frags_.size());
        frags_.push_back(std::move(f));
        ++added;
    }
    // 6. The cell map now points at the new pieces.
    for (std::size_t c = 0; c < N; ++c) {
        const int r = lab[c];
        if (r < 0) { pm.label[c] = -1; continue; }
        if (isNew[r]) pm.label[c] = regionIndex[r];
    }
    // 7. Which new pieces touch, and along how much boundary.
    links_.resize(frags_.size());
    {
        std::vector<std::pair<std::uint64_t, int>> pairs;
        auto add = [&](int a, int b) {
            if (a < 0 || b < 0 || a == b || !isNew[a] || !isNew[b]) return;
            int fa = regionIndex[a], fb = regionIndex[b];
            if (fa < 0 || fb < 0) return;
            if (fa > fb) std::swap(fa, fb);
            pairs.push_back({(static_cast<std::uint64_t>(fa) << 32) | static_cast<std::uint32_t>(fb), 1});
        };
        // Pieces either side of a crack cell share that bit of crack.
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const int c = j * nx + i;
                if (lab[c] >= 0) {
                    if (i + 1 < nx) add(lab[c], lab[c + 1]);
                    if (j + 1 < ny) add(lab[c], lab[c + nx]);
                    continue;
                }
                int seen[8], ns = 0;
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di) {
                        const int x = i + di, y = j + dj;
                        if ((di == 0 && dj == 0) || x < 0 || y < 0 || x >= nx || y >= ny) continue;
                        const int r = lab[y * nx + x];
                        if (r < 0) continue;
                        bool dup = false;
                        for (int k = 0; k < ns; ++k) dup = dup || seen[k] == r;
                        if (!dup) seen[ns++] = r;
                    }
                for (int a = 0; a < ns; ++a)
                    for (int b = a + 1; b < ns; ++b) add(seen[a], seen[b]);
            }
        std::sort(pairs.begin(), pairs.end());
        for (std::size_t k = 0; k < pairs.size();) {
            std::size_t e = k;
            while (e < pairs.size() && pairs[e].first == pairs[k].first) ++e;
            const int fa = static_cast<int>(pairs[k].first >> 32), fb = static_cast<int>(pairs[k].first & 0xffffffffu);
            const Real len = static_cast<Real>(e - k) * cell;
            links_[fa].push_back({fb, len});
            links_[fb].push_back({fa, len});
            k = e;
        }
    }
    if (added) ++revision;
    return added;
}

const std::vector<FragmentLink>& FragmentSystem::links(int k) const {
    static const std::vector<FragmentLink> none;
    return k >= 0 && k < static_cast<int>(links_.size()) ? links_[k] : none;
}

int FragmentSystem::makeClump(const std::vector<int>& members, const Vec3& vel, const Vec3& omega, Real time,
                              Rng& rng) {
    if (members.empty()) return -1;
    Clump c;
    c.members = members;
    for (const int k : members) {
        const Fragment& f = frags_[k];
        c.mass += f.mass;
        c.pos += f.pos * f.mass;
    }
    c.pos = c.pos / c.mass;
    Real ix = 0, iy = 0, iz = 0;
    for (const int k : members) {
        Fragment& f = frags_[k];
        const Vec3 r = f.pos - c.pos;
        ix += f.inertia.x + f.mass * (r.y * r.y + r.z * r.z);
        iy += f.inertia.y + f.mass * (r.x * r.x + r.z * r.z);
        iz += f.inertia.z + f.mass * (r.x * r.x + r.y * r.y);
        c.radius = std::max(c.radius, length(r) + f.radius);
        f.clump = static_cast<int>(clumps_.size());
        f.clumpOffset = r;  // pieces are pane-aligned while they hang in the frame
        f.rot = Quat{};
        f.state = FragmentState::Free;
    }
    c.inertia = {std::max(ix, 1e-12), std::max(iy, 1e-12), std::max(iz, 1e-12)};
    c.vel = vel;
    c.omega = omega;
    // Slabs of loose dice come apart in the air within a few tenths of a second.
    c.breakupAt = time + 0.15 + 0.5 * rng.uniform() + 0.02 * std::sqrt(static_cast<Real>(members.size()));
    c.alive = true;
    clumps_.push_back(std::move(c));
    ++revision;
    return static_cast<int>(clumps_.size() - 1);
}

void FragmentSystem::dissolve(int ci, Real, Rng& rng, Real spread) {
    Clump& c = clumps_[ci];
    if (!c.alive) return;
    c.alive = false;
    for (const int k : c.members) {
        Fragment& f = frags_[k];
        if (f.clump != ci) continue;
        f.clump = -1;
        const Vec3 r = c.rot.rotate(f.clumpOffset);
        f.vel = c.vel + cross(c.omega, r) +
                Vec3{rng.uniform(-spread, spread), rng.uniform(0, spread), rng.uniform(-spread, spread)};
        f.omega = c.omega + Vec3{rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(-10, 10)} * (spread + 0.1);
        f.state = FragmentState::Free;
    }
}

Real Clump::fold(Real t) const {
    if (foldTime <= 0) return t >= foldStart ? 1.0 : 0.0;
    const Real x = std::clamp((t - foldStart) / foldTime, 0.0, 1.0);
    return x * x * (3 - 2 * x);
}

Real Clump::foldRate(Real t) const {
    if (foldTime <= 0) return 0;
    const Real x = (t - foldStart) / foldTime;
    return x <= 0 || x >= 1 ? 0.0 : 6 * x * (1 - x) / foldTime;
}

namespace {
// The rotation a share s of the way from none to q (normalised lerp).
Quat partway(const Quat& q, Real s) {
    Quat r{1 - s + s * q.w, s * q.x, s * q.y, s * q.z};
    if (q.w < 0) r = {1 - s - s * q.w, -s * q.x, -s * q.y, -s * q.z};
    r.normalise();
    return r;
}
}  // namespace

// A laminate's sheet: one body whatever its glass is doing, folding as it
// goes. On the floor it stops whole (it slides, it does not burst).
void FragmentSystem::stepSheet(Clump& c, Real dt, Real time) {
    if (c.resting) return;
    const Real M = c.mass + c.extraMass, sp = length(c.vel);
    Real area = c.dragArea;
    if (area <= 0)
        for (const int k : c.members) area += frags_[k].area;
    c.vel += (Vec3{0, -kGravity, 0} + c.vel * (-0.5 * kAirDensity * 1.2 * area * sp / M)) * dt;
    c.omega *= std::exp(-0.6 * dt);
    c.pos += c.vel * dt;
    c.rot.integrate(c.omega, dt);
    const Real s = c.fold(time + dt), ds = c.foldRate(time + dt);
    Real low = 0;
    for (const int k : c.members) {
        Fragment& f = frags_[k];
        const Vec3 r = c.rot.rotate(f.clumpOffset + (f.foldOffset - f.clumpOffset) * s);
        f.pos = c.pos + r;
        f.rot = c.rot * partway(f.foldRot, s);
        f.vel = c.vel + cross(c.omega, r) + c.rot.rotate(f.foldOffset - f.clumpOffset) * ds;
        f.omega = c.omega;
        // A flat piece reaches down its radius times how far it is turned from lying flat.
        const Real ny = f.rot.rotate({0, 0, 1}).y;
        low = std::max(low, floorY - (f.pos.y - f.radius * std::sqrt(std::max(0.0, 1 - ny * ny)) - 0.5 * f.thickness));
    }
    if (low <= 0) return;
    c.pos.y += low;
    for (const int k : c.members) frags_[k].pos.y += low;
    if (c.vel.y < 0) c.vel.y *= -restitution;
    const Real vh = std::hypot(c.vel.x, c.vel.z), slow = friction * kGravity * dt;
    const Real keep = vh > slow ? 1 - slow / vh : 0.0;
    c.vel.x *= keep;
    c.vel.z *= keep;
    c.omega *= 0.7;
    if (length(c.vel) < 0.05 && length(c.omega) < 0.2) {
        c.resting = true;
        for (const int k : c.members) {
            Fragment& f = frags_[k];
            f.vel = {};
            f.omega = {};
            f.state = FragmentState::Resting;
        }
    }
}

void FragmentSystem::stepClumps(Real dt, Real time, Rng& rng) {
    for (int ci = 0; ci < static_cast<int>(clumps_.size()); ++ci) {
        Clump& c = clumps_[ci];
        if (!c.alive) continue;
        if (c.bonded) { stepSheet(c, dt, time); continue; }
        const Real sp = length(c.vel);
        Real area = 0;
        for (const int k : c.members) area += frags_[k].area;
        const Vec3 drag = c.vel * (-0.5 * kAirDensity * 1.2 * 0.5 * area * sp);
        c.vel += (Vec3{0, -kGravity, 0} + drag / c.mass) * dt;
        if (pushPressure != 0 && inOpening(c.pos) && std::fabs(c.pos.z) < pushReach)
            c.vel.z += pushPressure * (1 - std::fabs(c.pos.z) / pushReach) * area / c.mass * dt;
        c.omega *= std::exp(-0.6 * dt);
        c.pos += c.vel * dt;
        c.rot.integrate(c.omega, dt);
        bool landed = false;
        for (const int k : c.members) {
            Fragment& f = frags_[k];
            const Vec3 r = c.rot.rotate(f.clumpOffset);
            f.pos = c.pos + r;
            f.rot = c.rot;
            f.vel = c.vel + cross(c.omega, r);
            f.omega = c.omega;
            if (f.pos.y - f.radius < floorY) landed = true;
        }
        // A slab of loose dice does not survive the floor, nor long in the air.
        if (landed) dissolve(ci, time, rng, 0.35 + 0.08 * std::fabs(c.vel.y));
        else if (time >= c.breakupAt) dissolve(ci, time, rng, 0.25);
    }
}

int FragmentSystem::fragmentAt(int ply, Vec2 p) const {
    if (ply < 0 || ply >= static_cast<int>(maps.size())) return -1;
    const PlyMap& m = maps[ply];
    if (m.label.empty()) return -1;
    const int i = static_cast<int>(std::floor(p.x / m.cell)), j = static_cast<int>(std::floor(p.y / m.cell));
    if (i < 0 || j < 0 || i >= m.nx || j >= m.ny) return -1;
    return m.label[static_cast<std::size_t>(j) * m.nx + i];
}

int FragmentSystem::addChip(int ply, Vec2 at, Real z, Real size, Real thickness, Real density, const Vec3& vel,
                            Real time, Rng& rng) {
    if (static_cast<int>(frags_.size()) >= maxFragments) return -1;
    Fragment f;
    f.ply = ply;
    f.thickness = thickness;
    f.chip = true;
    const int k = 5 + static_cast<int>(rng.uniform() * 3);
    Real a0 = rng.uniform(0, 2 * kPi);
    for (int i = 0; i < k; ++i) {
        const Real a = a0 + 2 * kPi * i / k + rng.uniform(-0.3, 0.3) * kPi / k;
        f.outline.push_back(Vec2{std::cos(a), std::sin(a)} * (size * rng.uniform(0.45, 1.0)));
    }
    f.origin = at;
    finishFragment(f, density);
    f.pos = {f.origin.x, f.origin.y, z};
    f.vel = vel;
    f.omega = {rng.uniform(-60, 60), rng.uniform(-60, 60), rng.uniform(-30, 30)};
    f.state = FragmentState::Free;
    f.bornAt = time;
    f.id = nextId_++;
    frags_.push_back(std::move(f));
    ++revision;
    return static_cast<int>(frags_.size() - 1);
}

void FragmentSystem::split(std::size_t k, Rng& rng, Real time) {
    if (static_cast<int>(frags_.size()) + 2 > maxFragments) return;
    Fragment parent = frags_[k];
    const Real ang = rng.uniform(0, kPi);
    const Vec2 n{std::cos(ang), std::sin(ang)};
    const Vec2 shift = perp(n) * rng.uniform(-0.25, 0.25) * parent.radius;
    std::vector<Vec2> local;
    local.reserve(parent.outline.size());
    for (const auto& p : parent.outline) local.push_back(p - shift);
    std::vector<Vec2> halves[2] = {clipHalf(local, n), clipHalf(local, -n)};
    for (auto& h : halves)
        if (h.size() < 3 || std::fabs(polygonArea(h)) < 1e-7) return;
    frags_[k].retired = true;
    frags_[k].outline.clear();
    const Real density = parent.mass / std::max(1e-12, parent.area * parent.thickness);
    for (int s = 0; s < 2; ++s) {
        Fragment f = parent;
        f.outline = halves[s];
        f.holes.clear();  // a hole is rarely what a landing cuts through; drop it from the halves
        for (auto& p : f.outline) p += shift;
        f.origin = {0, 0};
        finishFragment(f, density);
        const Vec3 off = parent.rot.rotate({f.origin.x, f.origin.y, 0});
        f.pos = parent.pos + off;
        const Vec3 sep = parent.rot.rotate({(s ? -n.x : n.x) * 0.3, (s ? -n.y : n.y) * 0.3, 0});
        f.vel = parent.vel + cross(parent.omega, off) + sep + Vec3{0, 0.4, 0};
        f.omega = parent.omega + Vec3{rng.uniform(-8, 8), rng.uniform(-8, 8), rng.uniform(-8, 8)};
        f.generation = parent.generation + 1;
        f.state = FragmentState::Free;
        f.retired = false;
        f.bornAt = time;
        f.id = nextId_++;
        frags_.push_back(std::move(f));
    }
    ++revision;
}

std::vector<int> FragmentSystem::breakUp(int k, Real maxRadius, Rng& rng, Real time) {
    std::vector<int> out, todo{k};
    while (!todo.empty()) {
        const int i = todo.back();
        todo.pop_back();
        if (frags_[i].radius <= maxRadius) { out.push_back(i); continue; }
        const std::size_t before = frags_.size();
        split(static_cast<std::size_t>(i), rng, time);
        if (!frags_[i].retired) { out.push_back(i); continue; }  // could not split it
        for (std::size_t j = before; j < frags_.size(); ++j) todo.push_back(static_cast<int>(j));
    }
    return out;
}

bool FragmentSystem::inOpening(const Vec3& p) const {
    constexpr Real kMargin = 0.05;
    return p.x > -kMargin && p.y > -kMargin && p.x < pushSize.x + kMargin && p.y < pushSize.y + kMargin;
}

void FragmentSystem::step(Real dt, Real time, Rng& rng) {
    stepClumps(dt, time, rng);
    std::vector<std::size_t> toSplit;
    const std::size_t n = frags_.size();
    const Real spinDecay = std::exp(-0.4 * dt);
    for (std::size_t k = 0; k < n; ++k) {
        Fragment& f = frags_[k];
        if (f.retired || f.clump >= 0) continue;
        if (f.state == FragmentState::Pending && time >= f.releaseTime) f.state = FragmentState::Free;
        if (f.state != FragmentState::Free) continue;
        // A tempered piece drawn coarser than the real dice is a handful of
        // them barely locked together: in the air it soon comes apart, and
        // renderers scatter the real dice from where it was.
        if (f.diceSize > 0 && !f.chip && f.radius > 0.75 * f.diceSize) {
            if (f.freeAt < 0) f.freeAt = time;
            const std::uint32_t h = static_cast<std::uint32_t>(f.id) * 2654435761u;
            const Real delay = 0.04 + 0.12 * static_cast<Real>(h >> 8) / 16777216.0;
            if (time - f.freeAt > delay) {
                DiceSpray d;
                d.pos = f.pos;
                d.vel = f.vel;
                d.count = static_cast<int>(std::clamp(f.area / (f.diceSize * f.diceSize), 2.0, 400.0));
                d.size = f.diceSize;
                d.thickness = f.thickness;
                d.time = time;
                d.ply = f.ply;
                d.omega = f.omega;
                d.fragment = static_cast<int>(k);
                sprays_.push_back(d);
                f.retired = true;
                f.outline.clear();
                ++revision;
                continue;
            }
        }
        // Gravity and quadratic air drag on the mean tumbling area.
        const Real sp = length(f.vel);
        const Vec3 drag = f.vel * (-0.5 * kAirDensity * 1.2 * 0.5 * f.area * sp);
        f.vel += (Vec3{0, -kGravity, 0} + drag / f.mass) * dt;
        // Gas venting through the opening drives the pieces still in it.
        if (pushPressure != 0 && inOpening(f.pos)) {
            const Real d = std::fabs(f.pos.z);
            if (d < pushReach) f.vel.z += pushPressure * (1 - d / pushReach) * f.area / f.mass * dt;
        }
        f.omega *= spinDecay;
        f.pos += f.vel * dt;
        f.rot.integrate(f.omega, dt);
        // Floor: the lowest corner of the slab (only near the floor).
        if (f.pos.y - f.radius - f.thickness > floorY) continue;
        Real minY = 1e9;
        Vec3 low;
        for (const auto& p : f.outline) {
            for (const Real zz : {-0.5 * f.thickness, 0.5 * f.thickness}) {
                const Vec3 w = f.pos + f.rot.rotate({p.x, p.y, zz});
                if (w.y < minY) { minY = w.y; low = w; }
            }
        }
        if (minY >= floorY) continue;
        // A face or edge on the floor scrubs off spin a point contact cannot
        // (about the normal, and rocking): rolling and spinning friction.
        f.omega *= std::exp(-12 * dt);
        f.pos.y += floorY - minY;
        low.y = floorY;
        const Vec3 r = low - f.pos;
        const Vec3 nrm{0, 1, 0};
        const Vec3 vc = f.vel + cross(f.omega, r);
        const Real vn = dot(vc, nrm);
        if (vn < 0) {
            const Vec3 rn = cross(r, nrm);
            const Real kn = 1 / f.mass + dot(cross(bodyInverse(f.rot, f.inertia, rn), r), nrm);
            const Real jn = -(1 + restitution) * vn / kn;
            Vec3 J = nrm * jn;
            Vec3 vt = vc - nrm * vn;
            const Real vtl = length(vt);
            if (vtl > 1e-6) {
                const Vec3 t = vt / vtl;
                const Real kt = 1 / f.mass + dot(cross(bodyInverse(f.rot, f.inertia, cross(r, t)), r), t);
                const Real jt = std::max(-friction * jn, -vtl / kt);
                J += t * jt;
            }
            f.vel += J / f.mass;
            f.omega += bodyInverse(f.rot, f.inertia, cross(r, J));
            // A tempered piece drawn coarser than the real dice bursts into
            // them when it lands; renderers scatter the real ones.
            if (f.diceSize > 0 && f.radius > 0.75 * f.diceSize && -vn > 0.8 && !f.chip) {
                DiceSpray d;
                d.pos = f.pos;
                d.vel = f.vel;
                d.count = static_cast<int>(std::clamp(f.area / (f.diceSize * f.diceSize), 2.0, 400.0));
                d.size = f.diceSize;
                d.thickness = f.thickness;
                d.time = time;
                d.ply = f.ply;
                d.omega = f.omega;
                d.fragment = static_cast<int>(k);
                sprays_.push_back(d);
                f.retired = true;
                f.outline.clear();
                ++revision;
                continue;
            }
            // Hard landings break the piece (energy against new crack surface).
            const Real cut = 2 * std::sqrt(f.area / kPi);
            if (0.5 * f.mass * vn * vn > breakEnergyFactor * toughnessEnergy * f.thickness * cut && f.generation < 3 &&
                f.radius > 0.004)
                toSplit.push_back(k);
        }
        // Settled: a few slow contacts in a row put it to sleep.
        // Spin counts by how fast it moves the edge: a small die jitters on its
        // corner at a few rad/s with the edge barely moving.
        f.calm = (length(f.vel) + length(f.omega) * f.radius < 0.12) ? f.calm + 1 : 0;
        if (f.calm > 6) {
            f.vel = {};
            f.omega = {};
            f.state = FragmentState::Resting;
        }
    }
    for (const auto k : toSplit) split(k, rng, time);
}

}  // namespace gls
