// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/pane.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace gls {

Budget budgetForTier(int tier) {
    Budget b;
    switch (std::clamp(tier, 0, kTierCount - 1)) {
        case 0: b = {"potato", 10, 0.006, 40000, 96, 20000, 600, 0.05, 0.016, 25}; break;
        case 1: b = {"low", 16, 0.004, 100000, 192, 50000, 1500, 0.035, 0.010, 36}; break;
        case 2: b = {"medium", 24, 0.003, 250000, 384, 120000, 3000, 0.025, 0.007, 36}; break;
        case 3: b = {"high", 32, 0.002, 500000, 768, 250000, 6000, 0.02, 0.005, 36}; break;
        default: b = {"ultra", 44, 0.0015, 1000000, 1536, 500000, 12000, 0.015, 0.003, 49}; break;
    }
    return b;
}

namespace {

// ---- Calibration constants (docs/architecture/physics.md lists their basis) ----
constexpr Real kQuiet = 150e-6;            // s of crack silence before a ply is cut into pieces
constexpr Real kDynamicWindow = 0.02;      // s after a load during which flaws are probed
constexpr Real kBranchSpacing = 3.0;       // fork spacing / mirror radius (A_b / sigma)^2
constexpr Real kBranchHalfAngle = deg(17); // annealed / bending forks
constexpr Real kDiceHalfAngle = deg(28);   // tempered forks
constexpr Real kDiceEnergyFraction = 0.5;  // share of stored energy that makes new surface
constexpr Real kDiceBranchRatio = 0.9;     // fork spacing / energy-balance fragment size
constexpr Real kWedgeSupport = 1.0;        // wedge-root moment / tip-loaded cantilever value
constexpr Real kFragmentStiffening = 2.0;  // bonded fragments stiffen the interlayer membrane
constexpr Real kStrainConcentration = 1.0; // interlayer strain at fragment edges over the mean
constexpr Real kMushroom = 0.3;            // ballistic nose flattening coefficient
constexpr Real kPushRadius = 1.2;          // pushed-glass radius / impactor radius
constexpr Real kConeDrive = 50;            // energy to drive a cone through / its surface energy
constexpr Real kAuerbach = 5e4;            // N/m: Hertzian cone load / ball radius (Auerbach's law)
constexpr Real kWedgeStiffness = 1.0;      // wedge ring stiffness / (2 pi D / L^2)
constexpr Real kRadialDrive = 1.0;         // hoop stress at radial tips / wedge-root stress
constexpr Real kCrushRagged = 0.14;        // amplitude of the crushed zone's ragged edge (of its radius)
constexpr Real kWanderAngle = 0.12;        // rad, rms wander of a crack about its stress-preferred path
constexpr Real kWanderLength = 0.04;       // m, over which the wander changes
constexpr Real kMinTurnRadius = 0.1;       // m, tightest bend of a running (non-ring) crack
constexpr Real kBlastActive = 200;         // Pa of blast pressure that counts as loading the pane
constexpr Real kBlastHold = 1000;          // Pa above which a blast tears loose pieces from their hold
constexpr Real kLaminateWedgeSupport = 0.15; // the same for wedges bonded to an interlayer
// Behind crushed glass, a polymer stack thinner than this share of the
// projectile's diameter dishes as a membrane instead of shearing out a plug
// (thin-target behaviour, Backman & Goldsmith 1978).
constexpr Real kMembraneRatio = 0.3;
constexpr Real kEdgeBond = 1e6;            // Pa: a sealant bond at break over the bite (structural silicone ~0.7-1 MPa)
constexpr Real kWrapPiece = 0.06;          // m: bonded glass folded round an impactor breaks to pieces this size
constexpr Real kShakerRadius = 0.01;        // m, the foot of a shaker (Chladni drive)
constexpr Real kToneStepsPerCycle = 32;    // steps per cycle of a tone (the force is held over each)
constexpr Real kToneProbesPerCycle = 24;   // flaw checks per cycle (catches the peak to within 1 %)
// Under a cyclic load a flaw grows only near the peaks of tension: with the
// stress-corrosion exponent n ~ 16, the integral of max(0, sin)^n over a cycle
// is about 0.1 of the cycle. That share of the time the tone has played is the
// equivalent time at its peak stress for the loading-rate factor.
constexpr Real kCyclicShare = 0.1;
constexpr Real kSheetPiece = 0.15;         // m: and elsewhere, cracked into pieces no bigger than this, still bonded
constexpr Real kWrapCone = 1.0;            // rad: a sheet wrapped whole folds round the nose this far, then trails as a cone
constexpr Real kWrapTight = 2.2;           // rad: a torn-out disc wraps round the impactor at most this far
constexpr Real kWrapDish = 0.12;           // the outer sheet left behind dishes this share of its radius
constexpr Real kWrapDisc = 2.0;            // the disc a tearing laminate wraps round an impactor, in rim radii
constexpr int kMaxRings = 5;               // concentric rings per crack star
constexpr Real kDormantCheck = 8e-6;       // s between looks at an arrested tip

std::string fmt(const char* f, double a = 0, double b = 0, double c = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, f, a, b, c);
    return buf;
}

Real noseOffset(const ImpactorDef& d) {
    if (d.kind == ImpactorKind::Sphere) return d.radius;
    return d.length > 0 ? 0.5 * d.length : d.radius;
}

}  // namespace

// ---- Construction ------------------------------------------------------------------

Pane::Pane(const GlazingDef& def, const Budget& budget, std::uint64_t seed) : def_(def), budget_(budget), rng_(seed) {
    for (auto& p : def_.plies) {
        auto m = std::make_unique<Material>(p.mat ? *p.mat : Material{});
        p.mat = m.get();
        mats_.push_back(std::move(m));
    }
    lam_ = makeLaminate(def_);
    const Real W = def_.width, H = def_.height;
    int M, N;
    if (W >= H) {
        M = budget_.modes;
        N = std::max(3, static_cast<int>(std::lround(budget_.modes * H / W)));
    } else {
        N = budget_.modes;
        M = std::max(3, static_cast<int>(std::lround(budget_.modes * W / H)));
    }
    plate_.configure(W, H, lam_.D, lam_.massPerArea, lam_.poisson, def_.damping, M, N);
    {
        // In-plane stiffness of the layup and its Poisson ratio (the stiff plies dominate).
        Real A = 0, An = 0;
        for (const auto& l : lam_.layers) {
            A += l.plateModulus * (l.z1 - l.z0);
            An += l.poisson * l.plateModulus * (l.z1 - l.z0);
        }
        plate_.configureStretch(A, A > 0 ? An / A : lam_.poisson, lam_.thickness, budget_.stretchModes);
    }
    plate_.addPressure(def_.pressure);
    plate_.settleStatic();

    int nGlass = 0;
    for (const auto& l : lam_.layers) nGlass += l.kind == MaterialKind::Glass ? 1 : 0;
    // Small panes get a finer grid (at least ~120 cells across the short side).
    const Real cell = std::max(std::min(budget_.crackCell, std::min(W, H) / 120), std::sqrt(W * H / budget_.maxCells));
    for (int l = 0; l < static_cast<int>(lam_.layers.size()); ++l) {
        const LaminateLayer& L = lam_.layers[l];
        if (L.kind != MaterialKind::Glass) {
            monolithic_ = false;
            const Material* m = def_.plies[L.ply].mat;
            membranePlies_.push_back({L.z1 - L.z0, m->youngs, m->flowStress, m->tensile, m->failureStrain, false});
            continue;
        }
        GlassPly G;
        G.ply = L.ply;
        G.layer = l;
        G.z0 = L.z0;
        G.z1 = L.z1;
        G.mat = def_.plies[L.ply].mat;
        const Ply& p = def_.plies[L.ply];
        G.centralTension = p.centralTension();
        G.surfaceCompression = p.surfaceCompression;
        G.compressionDepth = p.compressionDepth();
        // Residual tension forks a running crack when it exceeds the branching
        // stress for the core (mirror constant over the core's depth).
        const Real core = std::max(p.thickness - 2 * G.compressionDepth, 0.2 * p.thickness);
        G.dicing = G.centralTension > 0 && G.centralTension >= G.mat->branching / std::sqrt(0.6 * core);
        if (G.dicing) {
            // Energy balance: stored strain energy per area U = t (1-nu) CT^2 / (5E)
            // (parabolic profile) pays for the new crack faces of fragments of size a.
            const Real ct = G.centralTension;
            const Real a = 10 * sq(G.mat->toughness) / ((1 - G.mat->poisson) * kDiceEnergyFraction * ct * ct);
            // Coarsen to the tier's fragment budget (the real count is reported separately).
            const Real budgetSize = std::sqrt(W * H / (0.7 * budget_.maxFragments / nGlass));
            G.diceSpacing = std::max({kDiceBranchRatio * a, budget_.minDiceSpacing, kDiceBranchRatio * budgetSize});
            G.diceSize = a;
        }
        G.net.configure(W, H, cell);
        G.net.maxTips = std::max(32, budget_.maxTips / nGlass);
        G.net.maxSegments = std::max<std::size_t>(4000, budget_.maxSegments / nGlass);
        // Flaw probes: a Weibull strength per face over each probe's area.
        const Real s = std::min(budget_.probeSpacing, std::min(W, H) / 16);
        G.probeNx = std::max(4, static_cast<int>(std::lround(W / s)) + 1);
        G.probeNy = std::max(4, static_cast<int>(std::lround(H / s)) + 1);
        const Real area = (W / (G.probeNx - 1)) * (H / (G.probeNy - 1));
        const Real scale = G.mat->weibullScale * std::pow(1.0 / area, 1.0 / G.mat->weibullModulus);
        for (int j = 0; j < G.probeNy; ++j) {
            for (int i = 0; i < G.probeNx; ++i) {
                Probe pr;
                pr.p = {W * i / (G.probeNx - 1), H * j / (G.probeNy - 1)};
                pr.edge = i <= 1 || j <= 1 || i >= G.probeNx - 2 || j >= G.probeNy - 2;
                for (int f = 0; f < 2; ++f)
                    pr.strength[f] = rng_.weibull(scale, G.mat->weibullModulus) * (pr.edge ? G.mat->edgeFactor : 1.0);
                G.probes.push_back(pr);
            }
        }
        glass_.push_back(std::move(G));
    }
    frags_.maxFragments = budget_.maxFragments;
    frags_.pushSize = {W, H};
    if (!glass_.empty()) frags_.toughnessEnergy = sq(glass_[0].mat->toughness) / glass_[0].mat->youngs;
}

// ---- Launching -------------------------------------------------------------------------

int Pane::launch(const ImpactorDef& d, const Vec3& pos, const Vec3& vel) {
    Impactor I;
    I.def = d;
    I.pos = pos;
    I.vel = vel;
    I.axis = normalize(vel);
    I.nose = noseOffset(d);
    I.side = vel.z >= 0 ? 1 : -1;
    I.report.speedIn = length(vel);
    I.report.energyIn = 0.5 * d.mass * dot(vel, vel);
    I.brokenAtLoad.assign(glass_.size(), 0);
    imp_.push_back(I);
    lastDynamic_ = std::max(lastDynamic_, time_);
    return static_cast<int>(imp_.size() - 1);
}

int Pane::fire(const ImpactorDef& d, Vec2 at, Real speed, Real yaw, Real pitch, bool fromBack, Real lead) {
    if (d.pellets <= 1) return fireOne(d, at, speed, yaw, pitch, fromBack, lead);
    // A shot load: each pellet leaves at its own small angle (rms `spread`)
    // and a little faster or slower, and lands where that angle carries it over
    // the range; the shot string spreads their arrival.
    int first = -1;
    for (int k = 0; k < d.pellets; ++k) {
        const Real ax = d.spread * rng_.normal(), ay = d.spread * rng_.normal();
        const Vec2 off{std::tan(ax) * d.range, std::tan(ay) * d.range};
        const Real v = speed * (1 + 0.02 * rng_.normal());
        const int i = fireOne(d, at + off, v, yaw + ax, pitch + ay, fromBack, lead + d.stringLength * rng_.uniform());
        if (first < 0) first = i;
    }
    return first;
}

int Pane::fireOne(const ImpactorDef& d, Vec2 at, Real speed, Real yaw, Real pitch, bool fromBack, Real lead) {
    Vec3 dir{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    if (fromBack) dir.z = -dir.z;
    const Real nose = noseOffset(d);
    const Vec3 contact{at.x, at.y, fromBack ? lam_.thickness : 0.0};
    const Vec3 centre = contact - dir * nose;
    // Start `lead` back along the path, offset so gravity brings it onto the
    // aim point with the intended velocity: y0 = yc - vy tf - g tf^2 / 2.
    const Real tf = lead / std::max(speed, 1e-3);
    Vec3 pos = centre - dir * lead;
    pos.y -= 0.5 * kGravity * tf * tf;
    Vec3 vel = dir * speed;
    vel.y += kGravity * tf;
    return launch(d, pos, vel);
}

// ---- Time stepping --------------------------------------------------------------------

bool Pane::quiet() const {
    if (toneActive()) return false;
    for (const auto& b : blasts_)
        if (time_ < b.end()) return false;
    for (const auto& I : imp_)
        if (I.phase != ImpactorPhase::Done) return false;
    for (const auto& G : glass_)
        if (G.net.activeTips() > 0) return false;
    for (const auto& f : frags_.fragments())
        if (!f.retired && (f.state == FragmentState::Free || f.state == FragmentState::Pending)) return false;
    for (const auto& sh : sheets_)
        if (sh.active()) return false;
    return plate_.energy() < 1e-7;
}

Real Pane::chooseStep() const {
    Real dt = 2e-3;
    bool fragsMoving = false;
    for (const auto& f : frags_.fragments())
        if (!f.retired && (f.state == FragmentState::Free || f.state == FragmentState::Pending)) { fragsMoving = true; break; }
    for (const auto& sh : sheets_) fragsMoving = fragsMoving || sh.active();
    if (fragsMoving) dt = std::min(dt, 0.5e-3);
    for (const auto& I : imp_) {
        switch (I.phase) {
            case ImpactorPhase::Flight: {
                dt = std::min(dt, 1e-3);
                if (!I.through && I.side * I.vel.z > 0) {
                    const Real face = I.side > 0 ? 0.0 : lam_.thickness;
                    const Real gap = I.side * (face - (I.pos.z + I.side * I.nose));
                    const Real tz = gap / (I.side * I.vel.z);
                    dt = std::min(dt, std::max(0.1e-6, 0.5 * tz));
                }
                break;
            }
            case ImpactorPhase::Contact:
                if (I.pushing) { dt = std::min(dt, 2e-6); break; }
                dt = std::min(dt, I.contactStep);
                break;
            case ImpactorPhase::Ballistic: dt = std::min(dt, 0.05e-6); break;
            case ImpactorPhase::Membrane: dt = std::min(dt, 4e-6); break;
            case ImpactorPhase::Done: break;
        }
    }
    for (const auto& G : glass_)
        if (G.net.activeTips() > 0) dt = std::min(dt, 0.5 * G.net.cell() / G.mat->crackSpeed);
    for (const auto& b : blasts_) {
        if (time_ < b.arrival) dt = std::min(dt, std::max(0.2e-6, b.arrival - time_));          // land on the shock
        else if (time_ < b.arrival + 2 * b.duration) dt = std::min(dt, std::max(0.2e-6, b.duration / 50));
        else if (time_ < b.end()) dt = std::min(dt, 50e-6);
    }
    if (toneActive()) dt = std::min(dt, 1 / (kToneStepsPerCycle * std::max(tone_.frequency, 1.0)));
    if (time_ - lastDynamic_ < kDynamicWindow) dt = std::min(dt, 10e-6);
    return dt;
}

int Pane::advance(Real t, int maxSteps) {
    int steps = 0;
    while (time_ < t && steps < maxSteps) {
        if (quiet()) {
            // Nothing is happening: jump (the plate's leftover ring is below 0.1 uJ).
            time_ = t;
            break;
        }
        const Real dt = chooseStep();
        step(dt);
        ++steps;
    }
    return steps;
}

void Pane::log(const std::string& s) {
    if (events_.size() < 400) events_.push_back(fmt("%9.1f us  ", time_ * 1e6) + s);
}

Real Pane::rateFactor(Real loadTime) const {
    const Real n = glass_.empty() ? 16.0 : glass_[0].mat->corrosionN;
    return std::min(3.0, std::pow(60.0 / std::max(loadTime, 1e-6), 1.0 / (n + 1)));
}

void Pane::step(Real dt) {
    lastDt_ = dt;
    plate_.clearForces();
    const Real pb = blastPressure(time_);
    plate_.addPressure(def_.pressure + pb);
    const bool toned = toneActive();
    if (toned) {
        const Real F = toneValue(dt);
        if (tone_.shaker) plate_.addPatch(tone_.at.x, tone_.at.y, kShakerRadius, F);
        else plate_.addPressure(F);
        lastLoaded_ = time_;
    }
    if (std::fabs(pb) > kBlastActive) {
        if (blastOnset_ < 0) {
            blastOnset_ = time_;
            log(fmt("blast wave arrives: %.0f kPa reflected", pb / 1e3));
        }
        lastDynamic_ = lastLoaded_ = time_;
    }
    for (int k = 0; k < static_cast<int>(imp_.size()); ++k) stepImpactor(k, dt);
    plate_.step(dt);
    for (int k = 0; k < static_cast<int>(imp_.size()); ++k) {
        if (imp_[k].phase == ImpactorPhase::Contact && !imp_[k].pushing) loadPointChecks(k);
        if (imp_[k].phase == ImpactorPhase::Ballistic || imp_[k].ballisticEventsSeen < imp_[k].ballistic.events().size())
            ballisticEvents(k);
    }
    growCracks(dt);
    ringCracks();
    const bool inContact = std::any_of(imp_.begin(), imp_.end(), [](const Impactor& I) {
        return I.phase == ImpactorPhase::Contact || I.phase == ImpactorPhase::Ballistic ||
               I.phase == ImpactorPhase::Membrane;
    });
    if (inContact) lastLoaded_ = time_;
    Real probeEvery = 100e-6;
    for (const auto& I : imp_)
        if (I.phase == ImpactorPhase::Contact || I.phase == ImpactorPhase::Ballistic)
            probeEvery = std::min(probeEvery, std::max(5e-6, 5 * I.contactStep));
    const bool dynamic = time_ - lastDynamic_ < kDynamicWindow;
    if (toned && !dynamic) probeEvery = 1 / (kToneProbesPerCycle * std::max(tone_.frequency, 1.0));
    if ((dynamic || toned) && time_ - lastProbe_ >= probeEvery) {
        probeChecks();
        if (toned) toneMeterCheck();
        lastProbe_ = time_;
    }
    if (toned) {
        tonePhase_ = std::fmod(tonePhase_ + 2 * kPi * tone_.frequency * dt, 2 * kPi);
        if (tone_.sweep != 0) {
            tone_.frequency = std::max(0.1, tone_.frequency + tone_.sweep * dt);
            if (tone_.sweepTo > 0)
                tone_.frequency = tone_.sweep > 0 ? std::min(tone_.frequency, tone_.sweepTo) : std::max(tone_.frequency, tone_.sweepTo);
        }
        toneAmp_ += (tone_.level - toneAmp_) * (1 - std::exp(-dt / std::max(tone_.ramp, 1e-6)));
        if (tone_.level <= 0 && toneAmp_ < 1e-3 * std::max(1.0, toneAmp_)) toneAmp_ = 0;
    }
    time_ += dt;
    // Arrested cracks stay arrested once the load is long gone.
    if (time_ - lastDynamic_ > kDynamicWindow || time_ - lastLoaded_ > 3e-3)
        for (auto& G : glass_)
            if (G.net.aliveTips() > G.net.activeTips()) G.net.killDormant(time_);
    freezeCheck();
    // Pieces fly on a fixed 0.5 ms step of their own (renderers extrapolate
    // between), not at the microsecond steps of the fracture.
    constexpr Real kFragmentStep = 0.5e-3;
    if (time_ - fragTime_ >= kFragmentStep) {
        for (int i = 0; i < static_cast<int>(frags_.fragments().size()); ++i) {
            const Fragment& f = frags_.fragments()[i];
            if (!f.retired && f.state == FragmentState::Pending && time_ >= f.releaseTime) releaseFragment(i);
        }
        const Real pf = blastPressure(time_);
        if (std::fabs(pf) > kBlastHold) blowOut(pf);
        while (time_ - fragTime_ >= kFragmentStep) {
            fragTime_ += kFragmentStep;
            for (auto& sh : sheets_)
                if (sh.active()) sh.step(frags_, fragTime_, rng_, [this](int f) { releaseFragment(f); });
            frags_.pushPressure = blastPressure(fragTime_);
            frags_.step(kFragmentStep, fragTime_, rng_);
        }
    }
}

// ---- Impactors --------------------------------------------------------------------------

void Pane::stepImpactor(int k, Real dt) {
    Impactor& I = imp_[k];
    const Real m = I.def.mass, T = lam_.thickness;
    const Real W = def_.width, H = def_.height;
    switch (I.phase) {
        case ImpactorPhase::Done: return;
        case ImpactorPhase::Flight: {
            if (I.carriedBy >= 0) {
                // Wrapped in the laminate it tore out: it goes where the sheet goes
                // (stepped on the pieces' own clock, so carried on to now).
                const Clump& c = frags_.clumps()[I.carriedBy];
                I.vel = c.vel;
                I.pos = c.pos + c.vel * (time_ - fragTime_) + c.rot.rotate(I.carryOffset);
                if (c.resting) I.phase = ImpactorPhase::Done;
                return;
            }
            const Real sp = length(I.vel);
            const Real area = kPi * I.def.radius * I.def.radius;
            const Vec3 drag = I.vel * (-0.5 * kAirDensity * 0.4 * area * sp / m);
            I.vel += (Vec3{0, -kGravity, 0} + drag) * dt;
            I.pos += I.vel * dt;
            if (!I.through && I.side * I.vel.z > 0) {
                const Vec2 p = I.touched ? I.hit : Vec2{I.pos.x, I.pos.y};
                const Real face = (I.side > 0 ? 0.0 : T) + plate_.deflection(p);
                const Real noseZ = I.pos.z + I.side * I.nose;
                if (I.side * (noseZ - face) >= 0) {
                    const bool inside = p.x > 0 && p.y > 0 && p.x < W && p.y < H;
                    const GlassPly& first = I.side > 0 ? glass_.front() : glass_.back();
                    if (!inside || !first.net.glassAt(p)) {
                        I.through = true;  // missed, or through an existing hole
                        if (inside) I.report.outcome = "Passed through an existing hole";
                    } else if (I.touched) {
                        // Contact again (the plate sprang away and comes back).
                        I.phase = ImpactorPhase::Contact;
                        const Material& fm = *(I.side > 0 ? def_.plies.front() : def_.plies.back()).mat;
                        const Real Es = contactModulus(I.def.youngs, I.def.poisson, fm.youngs, fm.poisson);
                        const Real vn = std::max(0.01, I.side * I.vel.z);
                        if (I.def.kind == ImpactorKind::Sphere)
                            I.sphere.configure(I.def.contactCurvature(), Es, I.def.yield, I.def.restitution, vn);
                        else if (I.def.kind == ImpactorKind::Flat)
                            I.flat.configure(I.def.contactCurvature(), Es, I.def.yield);
                        else if (I.def.kind == ImpactorKind::Sharp)
                            I.sharp.configure(I.def.tipAngle, I.def.yield > 0 ? std::min(fm.hardness, I.def.yield) : fm.hardness, I.def.contactRadius);
                        contactStepFor(I);
                        lastDynamic_ = time_;
                        if (!monolithic_) {
                            bool all = true;
                            for (auto b : I.brokenAtLoad) all = all && b;
                            if (all) startMembrane(k);
                        }
                        return;
                    } else {
                        I.hit = p;
                        beginContact(k);
                        return;
                    }
                }
            }
            // The floor, then out of the scene.
            if (I.pos.y - I.def.radius < frags_.floorY) {
                I.pos.y = frags_.floorY + I.def.radius;
                if (I.vel.y < 0) I.vel.y *= -0.3;
                I.vel.x *= 0.8;
                I.vel.z *= 0.8;
                if (length(I.vel) < 0.3) { I.vel = {}; I.phase = ImpactorPhase::Done; }
            }
            if (std::fabs(I.pos.z) > 8 || std::fabs(I.pos.x - 0.5 * W) > 8) I.phase = ImpactorPhase::Done;
            return;
        }
        case ImpactorPhase::Contact: {
            const Vec2 h = I.hit;
            const Real w = plate_.deflection(h), wd = plate_.velocity(h);
            const Real face = (I.side > 0 ? 0.0 : T) + w;
            Real F = 0, a = 0;
            if (I.pushing) {
                const Real tp = time_ - I.pushStart;
                const Real noseZ = I.pos.z + I.side * I.nose;
                const Real delta = I.side * (noseZ - I.pushZ0);
                bool done = false;
                if (I.wedge) {
                    // The wedges between the radial cracks bend away like
                    // cantilevers from their roots at the crack tips.
                    Real reach = 0;
                    bool ringed = false;
                    for (const auto& st : stars_) {
                        if (st.impactor != k) continue;
                        reach = std::max(reach, st.reach);
                        ringed = ringed || st.lastRing > 0.9 * I.def.radius;
                    }
                    reach = std::max(reach, 1.5 * std::max(I.contactRadius, 1e-3));
                    const Real kw = kWedgeStiffness * 2 * kPi * lam_.D / (reach * reach);
                    // Petals under the impactor are bent round it: a petal pushed
                    // delta over the contact circle r_c = sqrt(2 R delta) carries
                    // sigma ~ E h delta / r_c^2 at the contact line and snaps there.
                    if (I.def.kind != ImpactorKind::Projectile && delta > 0) {
                        const Real R = I.def.contactCurvature();
                        const Real rc = std::sqrt(std::max(0.0, 2 * R * std::min(delta, R) - sq(std::min(delta, R))));
                        for (auto& st : stars_) {
                            if (st.impactor != k) continue;
                            const GlassPly& G = glass_[st.ply];
                            const Real hh = G.z1 - G.z0;
                            const Real sig = G.mat->youngs * hh * delta / std::max(rc * rc, 1e-12);
                            if (rc >= 3 * G.net.cell() && rc >= 1.5 * st.lastRing && rc <= 1.3 * R &&
                                sig > st.nextStrength * rateFactor(time_ - I.loadOnset))
                                spawnRing(st, rc * rng_.uniform(0.9, 1.15), sig);
                        }
                    }
                    F = ringed ? 0 : kw * std::max(0.0, delta);
                    // Petals broken off at a ring, or bent clear of the impactor.
                    done = ringed || delta > I.def.radius + T || I.side * I.vel.z <= 0;
                } else {
                    F = I.pushForce * std::exp(-tp / I.pushTau);
                    done = tp > 5 * I.pushTau;
                }
                a = std::max(I.contactRadius, 1e-4);
                I.vel.z -= I.side * F / m * dt;
                I.vel.y -= kGravity * dt;
                I.pos += I.vel * dt;
                if (done) {
                    I.force = 0;
                    I.pushing = false;
                    I.phase = ImpactorPhase::Flight;
                    I.report.contactTime = time_ - I.report.contactStart;
                    I.report.speedOut = length(I.vel);
                    I.report.absorbed = I.report.energyIn - 0.5 * m * dot(I.vel, I.vel);
                    if (I.side * I.vel.z > 0) {
                        I.report.outcome = fmt("Broke through; carried on at %.1f m/s (%.0f%% of its energy left)",
                                               length(I.vel),
                                               100 * (1 - I.report.absorbed / std::max(I.report.energyIn, 1e-12)));
                    } else {
                        I.report.penetrated = false;
                        I.report.rebounded = true;
                        I.report.outcome = "Cracked the glass and bounced back";
                    }
                    for (auto& hole : holes_)
                        if (length(hole.c - h) < 1e-9) hole.vel = I.vel;
                    log(I.report.outcome);
                    return;
                }
            } else if (I.def.kind == ImpactorKind::SoftBody) {
                const Real tc = time_ - I.report.contactStart;
                F = I.soft.force(tc);
                a = I.soft.patchRadius(tc);
                I.squash = std::min(2 * I.nose, I.soft.normalSpeed * tc);
                I.pos.z = face - I.side * (I.nose - 0.5 * I.squash);
                I.pos.x += I.vel.x * dt;
                I.pos.y += I.vel.y * dt;
                if (tc > I.soft.duration() + I.soft.radius / std::max(1.0, I.soft.normalSpeed)) {
                    I.force = 0;
                    I.vel = {};
                    I.phase = ImpactorPhase::Done;
                    I.report.contactTime = tc;
                    I.report.absorbed = I.report.energyIn;
                    if (I.report.outcome.empty())
                        I.report.outcome = I.report.broke ? "Bird spread on the glass; outer plies cracked, the structure held"
                                                          : "Bird spread on the glass; the glazing held";
                    return;
                }
            } else {
                // Above the highest mode the plate under a point load moves like
                // an infinite plate: a dashpot of impedance 8 sqrt(D rho h)
                // (Zener). That motion relaxes into the modes below their cutoff.
                const Real Z = 8 * std::sqrt(lam_.D * lam_.massPerArea);
                const Real tau = 1 / (2 * kPi * std::max(1.0, plate_.maxFrequencyHz()));
                const Real noseZ = I.pos.z + I.side * I.nose;
                const Real d = I.side * (noseZ - face) - I.localW;
                const Real rate = I.side * (I.vel.z - wd) - (I.force / Z - I.localW / tau);
                switch (I.def.kind) {
                    case ImpactorKind::Sphere: F = I.sphere.force(d, rate); a = I.sphere.contactRadius(); break;
                    case ImpactorKind::Sharp: F = I.sharp.force(d, rate); a = I.sharp.contactRadius(); break;
                    case ImpactorKind::Flat: F = I.flat.force(d, rate); a = I.flat.contactRadius(); break;
                    default: break;
                }
                I.vel.z -= I.side * F / m * dt;
                I.vel.y -= kGravity * dt;
                I.pos += I.vel * dt;
                const Real decay = std::exp(-dt / tau);
                I.localW = I.localW * decay + F / Z * tau * (1 - decay);
                if (d <= 0 && rate < 0 && time_ - I.report.contactStart > 1e-6) {
                    I.force = 0;
                    endContact(k, "rebound");
                    return;
                }
            }
            I.force = F;
            I.contactRadius = a;
            I.report.peakForce = std::max(I.report.peakForce, F);
            plate_.addPatch(h.x, h.y, std::max(a, 1e-4), I.side * F);
            return;
        }
        case ImpactorPhase::Ballistic: {
            const Real F = I.ballistic.step(dt, time_);
            I.force = F;
            I.contactRadius = I.ballistic.columnRadius();
            I.report.peakForce = std::max(I.report.peakForce, F);
            plate_.addPatch(I.hit.x, I.hit.y, std::max(I.contactRadius, 1e-3), I.side * F * I.losCos);
            const Real depth = I.ballistic.depth() * I.losCos;  // along the path -> through the pane
            const Real noseZ = (I.side > 0 ? depth : T - depth) + plate_.deflection(I.hit);
            I.pos.z = noseZ - I.side * I.nose;
            I.vel = I.axis * I.ballistic.speed();
            I.report.depth = std::max(I.report.depth, std::min(depth, T));
            I.report.bulge = I.ballistic.bulge();
            if (I.ballistic.handedOff()) {
                // The glass is rubble; the polymer behind it dishes round the projectile.
                I.contactRadius = I.ballistic.projectileRadius();
                startMembrane(k);
                I.membraneZ0 = I.pos.z + I.side * I.nose;
                return;
            }
            if (I.ballistic.done()) {
                I.report.absorbed = I.ballistic.absorbed();
                I.report.contactTime = time_ + dt - I.report.contactStart;
                if (I.ballistic.exited()) {
                    I.phase = ImpactorPhase::Flight;
                    I.through = true;
                    I.report.penetrated = true;
                    I.report.speedOut = I.ballistic.speed();
                    I.report.outcome = fmt("Penetrated; exits at %.0f m/s (%.0f%% of its energy absorbed)",
                                           I.ballistic.speed(), 100 * (1 - sq(I.ballistic.speed() / I.report.speedIn)));
                    holes_.push_back({I.hit, std::max(I.ballistic.projectileRadius(), 0.003), I.vel});
                    // Spall from the back face unless a polymer backing caught it.
                    const Ply& last = I.side > 0 ? def_.plies.back() : def_.plies.front();
                    const GlassPly& gl = I.side > 0 ? glass_.back() : glass_.front();
                    const bool backed = last.mat->kind == MaterialKind::Polymer;
                    const int n = backed ? 3 : 18;
                    for (int i = 0; i < n; ++i) {
                        const Real ang = rng_.uniform(0, 2 * kPi), cone = rng_.uniform(0, 0.6);
                        const Real v = I.report.speedOut * rng_.uniform(0.15, 0.6) + 5;
                        const Vec3 vv{std::cos(ang) * std::sin(cone) * v, std::sin(ang) * std::sin(cone) * v,
                                      I.side * std::cos(cone) * v};
                        const Vec2 at = I.hit + Vec2{std::cos(ang), std::sin(ang)} * rng_.uniform(0, I.contactRadius);
                        frags_.addChip(gl.ply, at, I.side > 0 ? T : 0.0, rng_.uniform(0.6e-3, 2.5e-3),
                                       std::min(gl.z1 - gl.z0, 2e-3), gl.mat->density, vv, time_, rng_);
                    }
                    log(fmt("projectile exits the back at %.0f m/s", I.ballistic.speed()));
                } else {
                    I.phase = ImpactorPhase::Done;
                    I.vel = {};
                    I.report.embedded = true;
                    I.report.speedOut = 0;
                    const int layer = I.ballistic.layer();
                    const int plyIdx = layer >= 0 ? (I.side > 0 ? layer : static_cast<int>(def_.plies.size()) - 1 - layer) : 0;
                    I.report.outcome = fmt("Stopped in ply %.0f of %.0f at %.1f mm depth", plyIdx + 1,
                                           static_cast<double>(def_.plies.size()), 1e3 * std::min(depth, T));
                    if (I.ballistic.bulge() > 0) I.report.outcome += fmt("; backing bulged %.1f mm", 1e3 * I.ballistic.bulge());
                    log(I.report.outcome);
                }
            }
            return;
        }
        case ImpactorPhase::Membrane: {
            const Real noseZ = I.pos.z + I.side * I.nose;
            const Real delta = I.side * (noseZ - I.membraneZ0);
            if (I.def.kind == ImpactorKind::Sphere || I.def.kind == ImpactorKind::SoftBody ||
                I.def.kind == ImpactorKind::Projectile) {
                const Real R = I.def.radius, dd = std::min(std::max(delta, 0.0), R);
                I.membrane.setRim(std::max(0.5 * R, std::sqrt(2 * R * dd - dd * dd)));
            }
            Real F;
            if (delta >= I.membraneMax) {
                F = I.membrane.force(delta);
                I.membraneMax = delta;
                I.membraneSet = F;
            } else {
                // Unloading: the interlayer has yielded; it springs back elastically.
                const Real dp = 0.6 * I.membraneMax;
                F = delta > dp ? I.membraneSet * (delta - dp) / (I.membraneMax - dp) : 0;
            }
            I.force = F;
            I.contactRadius = I.membrane.rimRadius();
            I.report.peakForce = std::max(I.report.peakForce, F);
            I.vel.z -= I.side * F / m * dt;
            I.vel.y -= kGravity * dt;
            I.pos += I.vel * dt;
            // The membrane hands its pull to the laminate around the broken zone.
            plate_.addPatch(I.hit.x, I.hit.y, 0.5 * I.membrane.outerRadius(), I.side * F);
            for (auto& b : bulges_)
                if (b.impactor == k) b.depth = std::max(0.0, std::max(delta, 0.6 * I.membraneMax));
            I.report.bulge = std::max(I.report.bulge, delta);
            // The membrane's tension falls off as 1/r out to the frame: if what
            // reaches the nearest edge is more than the frame holds, the edge
            // slides out of its rebate and the whole laminate goes.
            if (!I.membrane.torn() && I.membrane.rimTension() > 0) {
                const Real edge = std::max({I.membrane.outerRadius(),
                                            std::min(std::min(I.hit.x, def_.width - I.hit.x), std::min(I.hit.y, def_.height - I.hit.y))});
                const Real pull = I.membrane.rimTension() * I.membrane.rimRadius() / edge;
                if (pull > edgeHold()) {
                    pullOut(k, pull);
                    return;
                }
            }
            if (I.membrane.torn()) {
                I.phase = ImpactorPhase::Flight;
                I.through = true;
                I.report.penetrated = true;
                I.report.speedOut = std::fabs(I.vel.z);
                I.report.contactTime = time_ - I.report.contactStart;
                I.report.absorbed = I.report.energyIn - 0.5 * m * dot(I.vel, I.vel);
                I.report.outcome = fmt("Tore through the interlayer; exits at %.1f m/s", std::fabs(I.vel.z));
                const Real r = I.membrane.rimRadius();
                holes_.push_back({I.hit, r, I.vel});
                for (auto& G : glass_) {
                    std::vector<std::uint32_t> cells;
                    for (int j = 0; j < G.net.ny(); ++j)
                        for (int i = 0; i < G.net.nx(); ++i)
                            if (length(G.net.cellCentre(i, j) - I.hit) < r) cells.push_back(j * G.net.nx() + i);
                    G.net.markRemovedCells(cells);
                    for (int c = 0; c < 10; ++c) {
                        const Real ang = rng_.uniform(0, 2 * kPi);
                        const Vec2 at = I.hit + Vec2{std::cos(ang), std::sin(ang)} * rng_.uniform(0.3, 1.0) * r;
                        const Vec3 v{std::cos(ang) * 0.8, std::sin(ang) * 0.8, I.vel.z * rng_.uniform(0.3, 0.8)};
                        frags_.addChip(G.ply, at, 0.5 * (G.z0 + G.z1), rng_.uniform(2e-3, 6e-3), G.z1 - G.z0,
                                       G.mat->density, v, time_, rng_);
                    }
                }
                for (auto& b : bulges_)
                    if (b.impactor == k) b.depth = std::max(b.depth, 0.6 * I.membraneMax);
                log(I.report.outcome);
            } else if (I.side * I.vel.z < 0 && delta <= 0.6 * I.membraneMax) {
                I.phase = ImpactorPhase::Flight;
                I.report.rebounded = true;
                I.report.speedOut = std::fabs(I.vel.z);
                I.report.contactTime = time_ - I.report.contactStart;
                I.report.absorbed = I.report.energyIn - 0.5 * m * dot(I.vel, I.vel);
                I.report.outcome = fmt("Held by the interlayer: %.0f mm bulge, %.0f%% of the energy absorbed",
                                       1e3 * I.membraneMax, 100 * I.report.absorbed / std::max(I.report.energyIn, 1e-9));
                log(I.report.outcome);
            }
            return;
        }
    }
}

void Pane::contactStepFor(Impactor& I) {
    // A tenth of the contact period at the expected peak stiffness, with the
    // mass on the glass side taken as the plate within the modal resolution.
    const Material& fm = *(I.side > 0 ? def_.plies.front() : def_.plies.back()).mat;
    const Real Es = contactModulus(I.def.youngs, I.def.poisson, fm.youngs, fm.poisson);
    const Real v = std::max(0.1, std::fabs(I.vel.z));
    const Real m = I.def.mass;
    const Real mLocal = std::min(m, lam_.massPerArea * kPi * sq(plate_.resolutionRadius()));
    Real k = 1e6;
    switch (I.def.kind) {
        case ImpactorKind::Sphere: {
            const Real R = I.def.contactCurvature();
            const Real dmax = std::pow(15 * m * v * v / (16 * Es * std::sqrt(R)), 0.4);
            k = 2 * Es * std::sqrt(R * dmax);
            break;
        }
        case ImpactorKind::Sharp: {
            const Real alpha = kPi * sq(std::tan(I.def.tipAngle));
            const Real dmax = std::cbrt(1.5 * m * v * v / (alpha * fm.hardness));
            k = 2 * alpha * fm.hardness * dmax;
            break;
        }
        case ImpactorKind::Flat: k = 2 * Es * I.def.contactCurvature(); break;
        default: I.contactStep = 1e-6; return;
    }
    I.contactStep = clampr(0.1 * std::sqrt(mLocal / k), 0.05e-6, 5e-6);
}

void Pane::beginContact(int k) {
    Impactor& I = imp_[k];
    I.touched = true;
    I.phase = ImpactorPhase::Contact;
    I.report.contactStart = time_;
    for (auto& sh : sheets_) sh.disturb(I.hit, 0.08, time_);
    I.loadOnset = time_;
    lastDynamic_ = time_;
    // The struck surface: the first ply in the direction of travel.
    const Ply& face = I.side > 0 ? def_.plies.front() : def_.plies.back();
    const Material& fm = *face.mat;
    const Real vn = std::max(0.01, I.side * I.vel.z);
    const Real Es = contactModulus(I.def.youngs, I.def.poisson, fm.youngs, fm.poisson);
    // Local strength at the load point: Weibull over the highly stressed patch.
    I.strengthAtLoad.assign(2 * glass_.size(), 0);
    for (std::size_t g = 0; g < glass_.size(); ++g) {
        const Material& gm = *glass_[g].mat;
        const Real aeff = kPi * sq(std::max(0.01, I.def.kind == ImpactorKind::Flat ? 2 * I.def.contactCurvature() : 0.0));
        for (int f = 0; f < 2; ++f)
            I.strengthAtLoad[2 * g + f] =
                rng_.weibull(gm.weibullScale * std::pow(1.0 / aeff, 1.0 / gm.weibullModulus), gm.weibullModulus);
        if (glass_[g].net.crackedNear(I.hit, 2 * glass_[g].net.cell())) I.brokenAtLoad[g] = 1;
    }
    contactStepFor(I);
    const Material& g0 = *(I.side > 0 ? glass_.front() : glass_.back()).mat;
    I.coneStrength = rng_.weibull(g0.weibullScale * std::pow(1.0 / 2e-7, 1.0 / g0.weibullModulus), g0.weibullModulus);
    log(I.def.name + fmt(" strikes at %.1f m/s", I.report.speedIn));
    switch (I.def.kind) {
        case ImpactorKind::Sphere:
            I.sphere.configure(I.def.contactCurvature(), Es, I.def.yield, I.def.restitution, vn);
            break;
        case ImpactorKind::Sharp: I.sharp.configure(I.def.tipAngle, I.def.yield > 0 ? std::min(fm.hardness, I.def.yield) : fm.hardness, I.def.contactRadius); break;
        case ImpactorKind::Flat: I.flat.configure(I.def.contactCurvature(), Es, I.def.yield); break;
        case ImpactorKind::SoftBody:
            I.soft.density = I.def.density;
            I.soft.c0 = 1483 * std::pow(1 - I.def.porosity, 4);
            I.soft.radius = I.def.radius;
            I.soft.length = std::max(I.def.length, 2 * I.def.radius);
            I.soft.normalSpeed = vn;
            break;
        case ImpactorKind::Projectile: {
            // Oblique: the bullet keeps its speed and meets every layer at
            // its slant (line-of-sight) thickness; near grazing it glances off.
            const Real sp = std::max(length(I.vel), vn);
            I.losCos = clampr(vn / sp, 0.05, 1.0);
            if (I.losCos < std::cos(72 * kPi / 180)) {
                const GlassPly& gl = I.side > 0 ? glass_.front() : glass_.back();
                const int g = I.side > 0 ? 0 : static_cast<int>(glass_.size()) - 1;
                const std::uint32_t group = glass_[g].net.newGroup();
                crush(g, I.hit, std::max(1.5 * I.def.radius, gl.net.cell()), group, 1);
                startRadials(g, I.hit, std::max(2 * I.def.radius, gl.net.cell()), 3, group, CrackKind::Radial,
                             gl.mat->crackSpeed);
                I.report.broke = true;
                I.report.rebounded = true;
                I.vel.z = -0.2 * I.vel.z;
                I.vel.x *= 0.8;
                I.vel.y *= 0.8;
                I.report.speedOut = length(I.vel);
                I.phase = ImpactorPhase::Flight;
                I.report.outcome = fmt("Ricocheted at %.0f degrees from square", std::acos(I.losCos) * 180 / kPi);
                log(I.report.outcome);
                break;
            }
            std::vector<BallisticLayer> layers;
            const int n = static_cast<int>(lam_.layers.size());
            for (int i = 0; i < n; ++i) {
                const LaminateLayer& L = lam_.layers[I.side > 0 ? i : n - 1 - i];
                BallisticLayer b;
                b.ply = L.ply;
                b.kind = L.kind;
                b.mat = def_.plies[L.ply].mat;
                for (const auto& c : crushes_)
                    if (glass_[c.ply].ply == L.ply) b.rubble = std::max(b.rubble, c.r - length(c.c - I.hit));
                b.z0 = (I.side > 0 ? L.z0 : lam_.thickness - L.z1) / I.losCos;
                b.z1 = (I.side > 0 ? L.z1 : lam_.thickness - L.z0) / I.losCos;
                layers.push_back(b);
            }
            // Polymer the projectile meets once the glass is crushed: plugged if
            // thick beside it, dished as a membrane if thin.
            Real polymer = 0;
            for (const auto& L : lam_.layers)
                if (L.kind != MaterialKind::Glass) polymer += L.z1 - L.z0;
            const bool dish = !monolithic_ && polymer < kMembraneRatio * 2 * I.def.radius;
            I.ballistic.configure(layers, I.def, sp, kMushroom, dish);
            I.phase = ImpactorPhase::Ballistic;
            // Back-spray: chips thrown back off the struck face.
            const GlassPly& gl = I.side > 0 ? glass_.front() : glass_.back();
            for (int i = 0; i < 10; ++i) {
                const Real ang = rng_.uniform(0, 2 * kPi), cone = rng_.uniform(0, 1.0);
                const Real v = rng_.uniform(4, 25);
                const Vec3 vv{std::cos(ang) * std::sin(cone) * v, std::sin(ang) * std::sin(cone) * v,
                              -I.side * std::cos(cone) * v};
                frags_.addChip(gl.ply, I.hit, I.side > 0 ? 0.0 : lam_.thickness, rng_.uniform(0.4e-3, 1.5e-3),
                               std::min(gl.z1 - gl.z0, 1.5e-3), gl.mat->density, vv, time_, rng_);
            }
            break;
        }
    }
    // A laminate already broken here: straight onto the interlayer.
    if (!monolithic_ && I.phase == ImpactorPhase::Contact) {
        bool all = true;
        for (auto b : I.brokenAtLoad) all = all && b;
        if (all) startMembrane(k);
    }
}

void Pane::endContact(int k, const char* why) {
    Impactor& I = imp_[k];
    I.phase = ImpactorPhase::Flight;
    I.report.contactTime = time_ - I.report.contactStart;
    I.report.speedOut = length(I.vel);
    I.report.absorbed = I.report.energyIn - 0.5 * I.def.mass * dot(I.vel, I.vel);
    I.report.rebounded = I.side * I.vel.z < 0;
    {
        if (I.report.broke)
            I.report.outcome = "Cracked the glass and bounced off";
        else
            I.report.outcome = fmt("Bounced off at %.1f m/s; the glass held (peak %.2f kN)", I.report.speedOut,
                                   I.report.peakForce / 1e3);
    }
    if (I.report.rebounded) log(std::string("contact ends: ") + why);
}

void Pane::loadPointChecks(int k) {
    Impactor& I = imp_[k];
    const Real F = I.force;
    if (F <= 0) return;
    const Vec2 h = I.hit;
    const Curvature kc = plate_.curvature(h);
    const PlaneStrain em = membraneAt(h);  // smooth: the lattice holds it
    const Real re = equivalentRadius(std::max(I.contactRadius, 1e-4), lam_.thickness);
    const Real nf = I.side * plate_.nearField(F, 0, re, time_ - I.loadOnset);
    const Real rf = rateFactor(time_ - I.loadOnset);
    // Once some plies are broken under the load, the intact ones carry it on
    // their own: locally layered, and as much more curved as they are less stiff.
    // Only a broken ply on the tension side matters: one on the compression
    // side still carries load across its closed cracks.
    bool anyBroken = false;
    for (std::size_t g = 0; g < glass_.size(); ++g) {
        if (!I.brokenAtLoad[g]) continue;
        const Real mid = 0.5 * (glass_[g].z0 + glass_[g].z1);
        if (I.side * (mid - lam_.neutralAxis) > 0) anyBroken = true;
    }
    Real scale = 1;
    if (anyBroken) {
        Real Dloc = 0;
        for (std::size_t l = 0; l < lam_.layers.size(); ++l) {
            const LaminateLayer& L = lam_.layers[l];
            bool broken = false;
            for (std::size_t g = 0; g < glass_.size(); ++g)
                if (glass_[g].layer == static_cast<int>(l) && I.brokenAtLoad[g]) broken = true;
            if (!broken) Dloc += L.plateModulus * std::pow(L.z1 - L.z0, 3) / 12;
        }
        scale = lam_.D / std::max(Dloc, 1e-9 * lam_.D);
    }
    // (The local near field is what the broken plies stop sharing; the plate's
    // overall bending is unchanged.)
    const int struck = I.side > 0 ? 0 : static_cast<int>(glass_.size()) - 1;
    for (std::size_t g = 0; g < glass_.size(); ++g) {
        if (I.brokenAtLoad[g]) continue;
        // The load reaches a ply below the struck face spread over about its depth.
        const GlassPly& G = glass_[g];
        // Contact damage cuts into the struck ply: what is left of it bends harder.
        Real damage = 1;
        if (static_cast<int>(g) == struck && I.coneDepth > 0 && G.centralTension <= 0) {
            const Real hp = G.z1 - G.z0;
            damage = sq(hp / (hp - std::min(I.coneDepth, 0.8 * hp)));
        }
        const Real depth = I.side > 0 ? G.z0 : lam_.thickness - G.z1;
        const Real nfg = depth > re
                             ? I.side * plate_.nearField(F, 0, equivalentRadius(depth, G.z1 - G.z0), time_ - I.loadOnset)
                             : nf;
        for (int f = 0; f < 2; ++f) {
            const PlaneStress s = stressFromCurvature(static_cast<int>(g), f == 1, kc, nfg * scale, anyBroken, em);
            Real s1, s2, ang;
            s.principal(s1, s2, ang);
            if (s1 > 0) s1 = (s1 + G.surfaceCompression) * damage - G.surfaceCompression;
            const Real strength = I.strengthAtLoad[2 * g + f] * rf;
            I.report.maxStress = std::max(I.report.maxStress, std::min(s1, strength));
            if (s1 > strength) {
                breakAtLoad(k, static_cast<int>(g), s1, strength, 0);
                break;
            }
        }
        if (I.phase != ImpactorPhase::Contact) return;
    }
    // The struck face: Hertzian cone or sharp median crack.
    const int gs = I.side > 0 ? 0 : static_cast<int>(glass_.size()) - 1;
    const Ply& facePly = I.side > 0 ? def_.plies.front() : def_.plies.back();
    if (facePly.mat->kind != MaterialKind::Glass) return;
    GlassPly& G = glass_[gs];
    const Material& gm = *G.mat;
    if (I.def.kind == ImpactorKind::Sphere && I.cone && !I.brokenAtLoad[gs])
        I.coneDepth = std::max(I.coneDepth, 0.375 * coneCrackLength(F, gm.toughness));
    if (I.def.kind == ImpactorKind::Sphere && !I.cone && !I.brokenAtLoad[gs]) {
        const Real a = std::max(I.sphere.contactRadius(), 1e-6);
        // Auerbach's law sets the cone load (F_c ~ R, with flaw scatter); residual
        // surface compression has to be overcome on top of it.
        const Real R = I.def.contactCurvature();
        const Real scatter = I.coneStrength / (gm.weibullScale * std::pow(1.0 / 2e-7, 1.0 / gm.weibullModulus));
        const Real Fc = kAuerbach * R * clampr(scatter, 0.5, 2.0);
        const Real aC = std::cbrt(3 * Fc * R / (4 * contactModulus(I.def.youngs, I.def.poisson, gm.youngs, gm.poisson)));
        const Real sH = hertzEdgeTension(F, a, gm.poisson) - G.surfaceCompression;
        if (F >= Fc && sH >= hertzEdgeTension(Fc, aC, gm.poisson)) {
            I.cone = true;
            const std::uint32_t group = G.net.newGroup();
            G.net.addRing(h, std::max(1.1 * a, 0.75 * G.net.cell()), time_, CrackKind::Cone, group);
            const Real depth = 0.375 * coneCrackLength(F, gm.toughness);
            I.coneDepth = depth;
            log(fmt("Hertzian cone crack, %.2f mm deep", depth * 1e3));
            if (G.centralTension > 0 && depth >= G.compressionDepth) {
                breakAtLoad(k, gs, sH, sH, 0);
                return;
            }
            // A cone that can be driven through a monolithic pane punches a plug.
            const Real t = G.z1 - G.z0;
            const Real r1 = a, r2 = a + t * std::tan(gm.coneAngle);
            const Real slant = t / std::cos(gm.coneAngle);
            const Real Acone = kPi * (r1 + r2) * slant;
            const Real KE = 0.5 * I.def.mass * sq(I.side * I.vel.z);
            // Only a small, hard ball (radius under ~1.5 plate thicknesses) drives
            // its cone through before the plate fails in bending; never in
            // tempered glass, where the cone stays in the compression layer.
            if (monolithic_ && G.centralTension <= 0 && R < 1.5 * t &&
                KE > kConeDrive * sq(gm.toughness) / gm.youngs * Acone) {
                const Real plug = gm.density * kPi * t / 3 * (r1 * r1 + r1 * r2 + r2 * r2);
                const Real r = 0.5 * (r1 + r2);
                crush(gs, h, r, group, 0);
                std::vector<std::uint32_t> cells;
                for (int j = 0; j < G.net.ny(); ++j)
                    for (int i = 0; i < G.net.nx(); ++i)
                        if (length(G.net.cellCentre(i, j) - h) < r) cells.push_back(j * G.net.nx() + i);
                G.net.markRemovedCells(cells);
                I.vel.z = I.side * std::fabs(I.vel.z) * I.def.mass / (I.def.mass + plug);
                frags_.addChip(G.ply, h, I.side > 0 ? t : 0.0, r2, std::min(t, r2), gm.density,
                               I.vel * 0.95, time_, rng_);
                for (int c = 0; c < 6; ++c)
                    frags_.addChip(G.ply, h, 0.5 * t, rng_.uniform(0.5e-3, 1.5e-3), 1e-3, gm.density,
                                   I.vel * rng_.uniform(0.3, 0.8) + Vec3{rng_.uniform(-2, 2), rng_.uniform(-2, 2), 0},
                                   time_, rng_);
                startRadials(gs, h, r + G.net.cell(), 3 + static_cast<int>(rng_.uniform() * 3), group,
                             CrackKind::Radial, gm.crackSpeed);
                I.brokenAtLoad[gs] = 1;
                I.report.broke = true;
                I.report.penetrated = true;
                I.phase = ImpactorPhase::Flight;
                I.through = true;
                holes_.push_back({h, r, I.vel});
                I.report.outcome = fmt("Punched a cone hole %.1f mm across and flew on at %.0f m/s", 2e3 * r2,
                                       std::fabs(I.vel.z));
                I.report.contactTime = time_ - I.report.contactStart;
                log(I.report.outcome);
                if (G.originTime < 0) { G.origin = h; G.originTime = time_; }
                lastDynamic_ = time_;
            }
        }
    }
    if (I.def.kind == ImpactorKind::Sharp && !I.brokenAtLoad[gs]) {
        const Real c = medianCrackDepth(F, gm.youngs, gm.hardness, gm.toughness);
        I.coneDepth = std::max(I.coneDepth, c);
        if (G.centralTension > 0 && c >= G.compressionDepth) {
            log(fmt("median crack %.2f mm reaches the tension core", c * 1e3));
            breakAtLoad(k, gs, F, F, 0);
            return;
        }
        if (c >= G.z1 - G.z0) {
            log(fmt("median crack %.2f mm cuts through the ply", c * 1e3));
            breakAtLoad(k, gs, F, F, 0);
            return;
        }
        if (!I.cone && c > 0.5 * G.net.cell()) {
            I.cone = true;  // the pit and its short surface cracks
            const std::uint32_t group = G.net.newGroup();
            crush(gs, h, std::max(I.contactRadius, 0.5 * G.net.cell()), group, 0);
            startRadials(gs, h, std::max(I.contactRadius, G.net.cell()), 4, group, CrackKind::Radial,
                         gm.crackSpeed);
            log(fmt("sharp tip: %.2f mm median crack", c * 1e3));
        }
    }
}

void Pane::startRadials(int g, Vec2 c, Real r0, int n, std::uint32_t group, CrackKind kind, Real speed) {
    GlassPly& G = glass_[g];
    const Real base = rng_.uniform(0, 2 * kPi);
    for (int i = 0; i < n; ++i) {
        const Real a = base + 2 * kPi * (i + rng_.uniform(-0.35, 0.35)) / n;
        const Vec2 d{std::cos(a), std::sin(a)};
        Vec2 p = c + d * r0;
        p.x = clampr(p.x, 1e-6, def_.width - 1e-6);
        p.y = clampr(p.y, 1e-6, def_.height - 1e-6);
        G.net.spawn(p, d, time_, kind, group, 0, speed);
    }
}

void Pane::breakAtLoad(int k, int g, Real sigma, Real strength, int radials) {
    Impactor& I = imp_[k];
    GlassPly& G = glass_[g];
    I.brokenAtLoad[g] = 1;
    I.report.broke = true;
    lastDynamic_ = time_;
    if (G.originTime < 0) { G.origin = I.hit; G.originTime = time_; }
    const std::uint32_t group = G.net.newGroup();
    const Real cell = G.net.cell();
    const Real r0 = std::max(1.3 * I.contactRadius, 1.5 * cell);
    if (G.dicing) {
        I.report.diced = true;
        crush(g, I.hit, std::max(I.contactRadius, cell), group, 0);
        startRadials(g, I.hit, r0, 8, group, CrackKind::Dicing, G.mat->crackSpeed);
        log(fmt("tempered ply %.0f fails: the tension core takes over (CT %.0f MPa)", g + 1, G.centralTension / 1e6));
    } else {
        // More load over the strength, and more energy behind it, more radials:
        // a ball that barely breaks a pane leaves a handful, a cannonball a sunburst.
        const int n = radials > 0 ? radials
                                  : std::clamp(static_cast<int>(4 + 4 * std::log(std::max(1.0, sigma / strength)) +
                                                                1.5 * std::log(1 + I.report.energyIn / 100) +
                                                                rng_.uniform(0, 3)),
                                               4, 18);
        crush(g, I.hit, std::max(0.6 * I.contactRadius, 0.5 * cell), group, 0);
        startRadials(g, I.hit, r0, n, group, CrackKind::Radial, G.mat->crackSpeed);
        Star s;
        s.ply = g;
        s.impactor = k;
        s.centre = I.hit;
        s.reach = r0;
        s.born = time_;
        s.group = group;
        const Real h = G.z1 - G.z0;
        s.nextStrength = rng_.weibull(G.mat->weibullScale * std::pow(1.0 / (2 * kPi * 0.05 * h * 10), 1.0 / G.mat->weibullModulus),
                                      G.mat->weibullModulus);
        stars_.push_back(s);
        log(fmt("ply %.0f breaks under the load (local strength %.0f MPa), %.0f radial cracks", g + 1,
                std::min(sigma, strength) / 1e6, n));
    }
    if (I.phase != ImpactorPhase::Contact) return;
    if (monolithic_) {
        passThrough(k, 0);
    } else {
        bool all = true;
        for (auto b : I.brokenAtLoad) all = all && b;
        if (all) startMembrane(k);
    }
}

void Pane::passThrough(int k, Real) {
    Impactor& I = imp_[k];
    if (I.pushing) return;
    const Real R = std::max(I.def.radius, I.contactRadius);
    I.pushing = true;
    I.pushStart = time_;
    I.pushZ0 = I.pos.z + I.side * I.nose;
    I.pushForce = std::max(I.force, 1.0);
    I.through = true;
    I.report.penetrated = true;
    bool diced = false;
    for (const auto& G : glass_) diced = diced || (G.dicing && G.originTime >= 0);
    if (diced || I.def.kind == ImpactorKind::SoftBody) {
        // Nothing left to push against: the load collapses as the glass crumbles.
        I.wedge = false;
        const Real mPush = lam_.massPerArea * kPi * sq(kPushRadius * R);
        const Real dp = I.def.mass * std::fabs(I.vel.z) * mPush / (I.def.mass + mPush);
        I.pushTau = clampr(dp / I.pushForce, 5e-6, 5e-4);
    } else {
        // The wedges between the radial cracks take the load, continuously:
        // their stiffness 2 pi D / L^2 falls as the cracks lengthen, so the
        // force collapses as fast as the cracks run.
        I.wedge = true;
        const Real L0 = 1.5 * std::max(I.contactRadius, 1e-3);
        const Real kw0 = kWedgeStiffness * 2 * kPi * lam_.D / (L0 * L0);
        I.pushZ0 -= I.side * I.pushForce / kw0;
    }
    holes_.push_back({I.hit, kPushRadius * R, I.vel});
    log("broken under the load: shoving the broken glass aside");
}

void Pane::startMembrane(int k) {
    Impactor& I = imp_[k];
    if (membranePlies_.empty()) return;
    const Real R = I.def.radius;
    const Real rim = clampr(std::max(I.contactRadius, 0.6 * R), 0.004, std::max(0.004, R));
    const Real edge = std::min(std::min(I.hit.x, def_.width - I.hit.x), std::min(I.hit.y, def_.height - I.hit.y));
    const Real outer = std::max(1.6 * rim, std::min(0.95 * edge, std::max(5 * rim, 0.2)));
    I.membrane.configure(membranePlies_, rim, outer, kFragmentStiffening, kStrainConcentration);
    const Real w = plate_.deflection(I.hit);
    I.membraneZ0 = (I.side > 0 ? 0.0 : lam_.thickness) + w;
    I.membraneMax = 0;
    I.membraneSet = 0;
    I.phase = ImpactorPhase::Membrane;
    bulges_.push_back({I.hit, rim, outer, 0, k});
    log(fmt("all glass broken under the load: the interlayer takes it (rim %.0f mm, span %.0f mm)", rim * 1e3,
            outer * 1e3));
}

void Pane::crush(int g, Vec2 c, Real r, std::uint32_t group, int kind) {
    // Never a clean circle: crushed glass breaks out along a ragged edge.
    Real a[5], ph[5];
    for (int k = 0; k < 5; ++k) {
        a[k] = kCrushRagged * rng_.uniform(0.3, 1.0) / std::sqrt(k + 1.0);
        ph[k] = rng_.uniform(0, 2 * kPi);
    }
    glass_[g].net.markCrush(c, r, group, a, ph, 5);
    crushes_.push_back({g, c, r, time_, kind});
}

void Pane::ballisticEvents(int k) {
    Impactor& I = imp_[k];
    auto& ev = I.ballistic.events();
    for (; I.ballisticEventsSeen < ev.size(); ++I.ballisticEventsSeen) {
        const auto& e = ev[I.ballisticEventsSeen];
        int g = -1;
        for (std::size_t i = 0; i < glass_.size(); ++i)
            if (glass_[i].ply == e.ply) g = static_cast<int>(i);
        if (g < 0) continue;
        GlassPly& G = glass_[g];
        I.brokenAtLoad[g] = 1;
        I.report.broke = true;
        lastDynamic_ = time_;
        if (G.originTime < 0) { G.origin = I.hit; G.originTime = time_; }
        const std::uint32_t group = G.net.newGroup();
        const Real rc = std::max(1.15 * e.coneRadius, G.net.cell());
        crush(g, I.hit, rc, group, 1);
        if (G.dicing) {
            I.report.diced = true;
            startRadials(g, I.hit, rc + G.net.cell(), 8, group, CrackKind::Dicing, G.mat->crackSpeed);
        } else {
            const int n = std::clamp(static_cast<int>(std::lround(5 + 3 * std::log10(1 + e.energy / 20))), 5, 16);
            startRadials(g, I.hit, rc + G.net.cell(), n, group, CrackKind::Radial, G.mat->crackSpeed);
            Star s;
            s.ply = g;
            s.impactor = k;
            s.centre = I.hit;
            s.reach = rc;
            s.born = time_;
            s.group = group;
            const Real h = G.z1 - G.z0;
            s.nextStrength = rng_.weibull(
                G.mat->weibullScale * std::pow(1.0 / (2 * kPi * 0.05 * h * 10), 1.0 / G.mat->weibullModulus),
                G.mat->weibullModulus);
            stars_.push_back(s);
        }
        log(fmt("projectile enters glass ply %.0f at %.0f m/s; crush zone %.1f mm across", g + 1, e.speed,
                2e3 * rc));
    }
}

// ---- Cracks --------------------------------------------------------------------------------

PlaneStress Pane::stressFromCurvature(int g, bool back, const Curvature& k, Real nf, bool layered,
                                      const PlaneStrain& e) const {
    const GlassPly& G = glass_[g];
    const LaminateLayer& L = lam_.layers[G.layer];
    const Real z = back ? L.z1 : L.z0;
    // Layered: the ply bends about its own mid-plane (its neighbours are broken here).
    const Real lever = layered ? z - L.mid() : lam_.lever(G.layer, z);
    const Real E = L.plateModulus, nu = L.poisson;
    const Real kxx = k.kxx + nf, kyy = k.kyy + nf;
    // Bending, plus the stretch of the mid-surface (the same through the layup).
    PlaneStress s;
    s.sxx = E * (lever * (kxx + nu * kyy) + e.exx + nu * e.eyy) - G.surfaceCompression;
    s.syy = E * (lever * (kyy + nu * kxx) + e.eyy + nu * e.exx) - G.surfaceCompression;
    s.sxy = E * (1 - nu) * (lever * k.kxy + 0.5 * e.gxy);
    return s;
}

Curvature Pane::curvatureAt(Vec2 p) {
    // Whether the lattice is due is decided once per step.
    if (kCheckedAt_ != time_ || kGrid_.empty()) {
        kCheckedAt_ = time_;
        const bool loaded = std::any_of(imp_.begin(), imp_.end(), [](const Impactor& I) {
            return I.phase == ImpactorPhase::Contact || I.phase == ImpactorPhase::Ballistic ||
                   I.phase == ImpactorPhase::Membrane;
        });
        // A tenth of the shortest mode period resolves the modal field; the sharp
        // part under a load is added analytically per point (nearFieldAt).
        const Real modal = 0.1 / std::max(1.0, plate_.maxFrequencyHz());
        bool running = false;
        for (const auto& G : glass_) running = running || G.net.activeTips() > 0;
        const Real refresh = running ? std::min(5e-6, modal) : loaded ? 20e-6 : 25e-6;
        kDue_ = kGrid_.empty() || time_ - kGridTime_ >= refresh || time_ < kGridTime_;
    }
    if (kDue_) {
        kDue_ = false;
        const Real W = def_.width, H = def_.height;
        // Two samples per half-wavelength of the shortest mode.
        const int nLong = std::max(16, 2 * std::max(plate_.modesX(), plate_.modesY()) + 1);
        kNx_ = W >= H ? nLong : std::max(8, static_cast<int>(std::lround(nLong * W / H)));
        kNy_ = W >= H ? std::max(8, static_cast<int>(std::lround(nLong * H / W))) : nLong;
        kGrid_.resize(static_cast<std::size_t>(kNx_) * kNy_);
        plate_.sampleGrid(kNx_, kNy_, nullptr, nullptr, kGrid_.data());
        // The stretch changes only as fast as the membrane force is updated:
        // its lattice is refreshed at most once per membrane kick interval.
        const bool live = plate_.stretching();
        if (live != eLive_ || (live && (time_ - eGridTime_ >= plate_.stretchStep() || time_ < eGridTime_))) {
            eLive_ = live;
            eGridTime_ = time_;
            eGridBound_ = 0;
            if (live) {
                // Its own lattice, three nodes per half-wave of the shortest
                // stretching mode (the stretch is smoother than the bending).
                eNx_ = std::min(kNx_, 6 * plate_.stretchModesX() + 1);
                eNy_ = std::min(kNy_, 6 * plate_.stretchModesY() + 1);
                eGrid_.resize(static_cast<std::size_t>(eNx_) * eNy_);
                plate_.sampleGrid(eNx_, eNy_, nullptr, nullptr, nullptr, eGrid_.data());
                // The stretch is smooth: its largest strain on the lattice, with a
                // margin for between the nodes and for the time to the next refresh, bounds it.
                for (const auto& e : eGrid_) eGridBound_ = std::max(eGridBound_, std::fabs(e.exx) + std::fabs(e.eyy) + std::fabs(e.gxy));
                eGridBound_ *= 1.5;
            }
        }
        kGridTime_ = time_;
        kGridBound_ = plate_.curvatureBound();
    }
    const Real fx = clampr(p.x / def_.width, 0, 1) * (kNx_ - 1), fy = clampr(p.y / def_.height, 0, 1) * (kNy_ - 1);
    const int i = std::min(kNx_ - 2, static_cast<int>(fx)), j = std::min(kNy_ - 2, static_cast<int>(fy));
    const Real u = fx - i, v = fy - j;
    const Curvature& a = kGrid_[static_cast<std::size_t>(j) * kNx_ + i];
    const Curvature& b = kGrid_[static_cast<std::size_t>(j) * kNx_ + i + 1];
    const Curvature& c = kGrid_[static_cast<std::size_t>(j + 1) * kNx_ + i];
    const Curvature& d = kGrid_[static_cast<std::size_t>(j + 1) * kNx_ + i + 1];
    auto mix = [&](Real Curvature::*f) {
        return (1 - u) * (1 - v) * a.*f + u * (1 - v) * b.*f + (1 - u) * v * c.*f + u * v * d.*f;
    };
    return {mix(&Curvature::kxx), mix(&Curvature::kyy), mix(&Curvature::kxy)};
}

PlaneStrain Pane::membraneAt(Vec2 p) {
    curvatureAt(p);  // refreshes both lattices when due
    if (!eLive_) return {};
    const Real fx = clampr(p.x / def_.width, 0, 1) * (eNx_ - 1), fy = clampr(p.y / def_.height, 0, 1) * (eNy_ - 1);
    const int i = std::min(eNx_ - 2, static_cast<int>(fx)), j = std::min(eNy_ - 2, static_cast<int>(fy));
    const Real u = fx - i, v = fy - j;
    const PlaneStrain& a = eGrid_[static_cast<std::size_t>(j) * eNx_ + i];
    const PlaneStrain& b = eGrid_[static_cast<std::size_t>(j) * eNx_ + i + 1];
    const PlaneStrain& c = eGrid_[static_cast<std::size_t>(j + 1) * eNx_ + i];
    const PlaneStrain& d = eGrid_[static_cast<std::size_t>(j + 1) * eNx_ + i + 1];
    auto mix = [&](Real PlaneStrain::*f) {
        return (1 - u) * (1 - v) * a.*f + u * (1 - v) * b.*f + (1 - u) * v * c.*f + u * v * d.*f;
    };
    return {mix(&PlaneStrain::exx), mix(&PlaneStrain::eyy), mix(&PlaneStrain::gxy)};
}

PlaneStress Pane::stress(int g, bool back, Vec2 p) const {
    return stressFromCurvature(g, back, plate_.curvature(p.x, p.y), nearFieldAt(p), false, plate_.membraneStrain(p));
}

Real Pane::nearFieldAt(Vec2 p) const {
    Real nf = 0;
    for (const auto& I : imp_) {
        if ((I.phase != ImpactorPhase::Contact && I.phase != ImpactorPhase::Ballistic &&
             I.phase != ImpactorPhase::Membrane) ||
            I.force <= 0)
            continue;
        const Real r0 = I.phase == ImpactorPhase::Membrane ? 0.5 * I.membrane.outerRadius() : I.contactRadius;
        const Real re = equivalentRadius(std::max(r0, 1e-4), lam_.thickness);
        nf += I.side * plate_.nearField(I.force, length(p - I.hit), re, time_ - I.loadOnset);
    }
    return nf;
}

namespace {
// Kink angle of a crack under mixed-mode loading, K_II / K_I = r
// (maximum tangential stress criterion, Erdogan & Sih 1963).
Real kinkAngle(Real r) {
    if (std::fabs(r) < 1e-9) return 0;
    return 2 * std::atan((1 - std::sqrt(1 + 8 * r * r)) / (4 * r));
}
}  // namespace

void Pane::growCracks(Real dt) {
    const Real t1 = time_ + dt;
    int evalBudget = 4 * budget_.maxTips;
    Real hsum = 0;
    for (const auto& gg : glass_) hsum += gg.z1 - gg.z0;
    for (std::size_t g = 0; g < glass_.size(); ++g) {
        GlassPly& G = glass_[g];
        CrackNetwork& net = G.net;
        if (net.aliveTips() == 0) continue;
        const Material& m = *G.mat;
        const Real h = G.z1 - G.z0;
        const Real lcap = 0.6 * h;
        const Real cell = net.cell();
        const Real ct = G.centralTension;
        const Real branchMin = m.branching / std::sqrt(lcap);
        // Radial cracks of a star whose impactor still pushes: the bent
        // wedges put the uncracked ring around their tips in hoop tension
        // equal to the wedge-root stress (a plate with a hole loaded by an
        // edge moment), so they keep running outward while the load lasts.
        struct Drive {
            std::uint32_t group;
            Real sigma;
            Vec2 centre;
        };
        std::vector<Drive> drive;
        for (const auto& st : stars_) {
            if (st.ply != static_cast<int>(g)) continue;
            const Impactor& I = imp_[st.impactor];
            if (I.force <= 0 || I.phase == ImpactorPhase::Done || I.phase == ImpactorPhase::Flight) continue;
            const Real support = monolithic_ ? kWedgeSupport : kLaminateWedgeSupport;
            drive.push_back({st.group, kRadialDrive * support * 3 * I.force * (h / hsum) / (kPi * h * h), st.centre});
        }
        const int shieldCells = std::max(4, static_cast<int>(std::lround(1.5 * h / cell)));
        const std::size_t n0 = net.tips().size();
        for (std::size_t i = 0; i < n0; ++i) {
            if (!net.tips()[i].alive) continue;
            {
                CrackTip& tip = net.tips()[i];
                if (tip.dormant && time_ < tip.nextCheck) continue;
                const int every = tip.kind == CrackKind::Dicing ? 8 : 1;
                if (tip.fixedStress) {
                    // Ring crack: follows the circle about its centre.
                    Vec2 tan = perp(normalize(tip.pos - tip.anchor));
                    if (dot(tan, tip.dir) < 0) tan = -tan;
                    tip.kink = std::atan2(cross(tip.dir, tan), dot(tip.dir, tan));
                } else if (tip.evalAge >= every && evalBudget > 0) {
                    --evalBudget;
                    const Curvature kc = curvatureAt(tip.pos);
                    const PlaneStrain em = membraneAt(tip.pos);
                    const Real nf = nearFieldAt(tip.pos);
                    // Bending and stretch: the residual profile acts through its core tension.
                    PlaneStress f = stressFromCurvature(static_cast<int>(g), false, kc, nf, false, em);
                    PlaneStress b = stressFromCurvature(static_cast<int>(g), true, kc, nf, false, em);
                    for (PlaneStress* s : {&f, &b}) {
                        s->sxx += G.surfaceCompression + ct;
                        s->syy += G.surfaceCompression + ct;
                    }
                    const Real sf = f.normalAcross(tip.dir), sb = b.normalAcross(tip.dir);
                    const PlaneStress& use = sf > sb ? f : b;
                    const Vec2 n = perp(tip.dir);
                    Real sn = std::max(sf, sb);
                    const Real tau = n.x * (use.sxx * tip.dir.x + use.sxy * tip.dir.y) +
                                     n.y * (use.sxy * tip.dir.x + use.syy * tip.dir.y);
                    // Residual tension pulls equally every way: the more of the
                    // stress it is, the less the bending steers the crack (else
                    // heat-strengthened cracks loop round on the plate's ring-down).
                    const Real dev = 0.5 * std::hypot(use.sxx - use.syy, 2 * use.sxy);
                    const Real steer = tip.kind == CrackKind::Dicing ? 1.0 : dev / (dev + G.surfaceCompression + ct + 1e-9);
                    tip.kink = (sn > 0 ? kinkAngle(tau / sn) * steer : 0) + (tip.kind == CrackKind::Dicing ? 0 : tip.wob);
                    tip.radial = false;
                    if (tip.kind != CrackKind::Dicing) {
                        for (const auto& dv : drive) {
                            if (net.cracks()[tip.crack].group != dv.group) continue;
                            const Vec2 out = normalize(tip.pos - dv.centre);
                            const Real c = dot(tip.dir, out);
                            const Real sd = dv.sigma * c * std::fabs(c);
                            if (sd > sn) {
                                sn = sd;
                                tip.radial = true;
                                tip.kink = std::atan2(cross(tip.dir, out), dot(tip.dir, out)) + tip.wob;
                            }
                        }
                    }
                    tip.stress = sn;
                    tip.evalAge = 0;
                } else {
                    ++tip.evalAge;
                }
                if (!tip.fixedStress && tip.kind != CrackKind::Dicing) {
                    // A crack running close alongside another is unloaded by it;
                    // one cell away they coalesce.
                    // Of two cracks running side by side, the shielded one stops.
                    const int c = net.clearance(tip, shieldCells);
                    const bool young = length(tip.pos - net.cracks()[tip.crack].birth) < 4 * cell;
                    if (c <= 1 || (!young && 2 * c <= shieldCells)) {
                        net.kill(static_cast<int>(i), t1);
                        continue;
                    }
                    tip.shield = c > shieldCells ? 1.0 : std::max(0.05, sq(static_cast<Real>(c) / shieldCells));
                }
                // Arrest (dormant) when the stress intensity over the plate's depth drops
                // below K_Ic; otherwise run at a speed rising toward the terminal velocity.
                const Real K = tip.stress * tip.shield * std::sqrt(kPi * lcap);
                if (K < m.toughness) {
                    if (tip.fixedStress) { net.kill(static_cast<int>(i), t1); continue; }
                    tip.dormant = true;
                    tip.speed = 0;
                    tip.nextCheck = time_ + kDormantCheck;
                    tip.evalAge = 1 << 20;
                    continue;
                }
                tip.dormant = false;
                const Real v = m.crackSpeed * clampr(1 - sq(m.toughness / K), 0.05, 1.0);
                tip.speed = v;
                const Real ds = v * dt;
                // The kink develops over about a plate thickness of growth.
                Real turn = tip.kink * std::min(1.0, ds / (tip.fixedStress ? cell : h));
                // A running crack keeps its heading: it bends, it does not hook
                // round (sharp changes of direction come as forks).
                if (!tip.fixedStress && tip.kind != CrackKind::Dicing)
                    turn = clampr(turn, -ds / kMinTurnRadius, ds / kMinTurnRadius);
                tip.kink -= turn;
                const Real noise = tip.kind == CrackKind::Dicing ? 0.09 : (tip.fixedStress ? 0.02 : 0.035);
                turn += rng_.normal() * noise * std::sqrt(ds / cell);
                // Glass is not perfectly uniform: a running crack wanders a few
                // degrees either side of where the stress points it, slowly.
                if (tip.kind != CrackKind::Dicing && !tip.fixedStress)
                    tip.wob += -tip.wob * ds / kWanderLength +
                               kWanderAngle * std::sqrt(2 * ds / kWanderLength) * rng_.normal();
                tip.dir = rotate(tip.dir, turn);
                if (!net.advance(static_cast<int>(i), tip.pos + tip.dir * ds, t1)) continue;
            }
            CrackTip& tip = net.tips()[i];
            if (tip.fixedStress) continue;
            const Real se = tip.stress * tip.shield;
            Real spacing;
            if (tip.kind == CrackKind::Dicing)
                spacing = G.diceSpacing * sq(ct / std::max(tip.stress, ct));
            else
                spacing = kBranchSpacing * sq(m.branching / std::max(se, 1.0));
            spacing = std::max(spacing, (tip.kind == CrackKind::Dicing ? 2 : 3) * cell);
            if (se >= branchMin && tip.shield >= 1 && tip.sinceBranch >= spacing)
                net.branch(static_cast<int>(i), tip.kind == CrackKind::Dicing ? kDiceHalfAngle : kBranchHalfAngle, t1,
                           rng_);
        }
        net.compactTips();
    }
}

void Pane::ringCracks() {
    for (auto& s : stars_) {
        const Impactor& I = imp_[s.impactor];
        GlassPly& G = glass_[s.ply];
        // How far the star's radials have split the plate (kept after the load goes).
        for (const auto& t : G.net.tips()) {
            if (!t.alive || t.fixedStress) continue;
            if (G.net.cracks()[t.crack].group != s.group) continue;
            s.reach = std::max(s.reach, length(t.pos - s.centre));
        }
        if ((I.phase != ImpactorPhase::Contact && I.phase != ImpactorPhase::Ballistic &&
             I.phase != ImpactorPhase::Membrane) ||
            I.force <= 0)
            continue;
        const Real cell = G.net.cell();
        if (s.reach < 6 * cell || s.reach < 1.6 * s.lastRing) continue;
        // Wedges between radial cracks are cantilevers loaded at the tip: the
        // root moment per unit arc is F / (2 pi), so the face toward the impact
        // carries sigma = 3 F / (pi h^2) at their roots.
        Real hsum = 0;
        for (const auto& gg : glass_) hsum += gg.z1 - gg.z0;
        const Real h = G.z1 - G.z0;
        const Real share = I.force * h / hsum;
        // Bonded wedges (laminates) are held by the interlayer and the other
        // plies, so far less of the moment reaches their roots.
        const Real support = monolithic_ ? kWedgeSupport : kLaminateWedgeSupport;
        const Real root = support * 3 * share / (kPi * h * h);
        if (s.rings >= kMaxRings || root < s.nextStrength * rateFactor(time_ - I.loadOnset)) continue;
        spawnRing(s, s.reach * rng_.uniform(0.55, 0.85), root);
    }
}

void Pane::spawnRing(Star& s, Real R, Real stress) {
    GlassPly& G = glass_[s.ply];
    const int n = std::clamp(static_cast<int>(2 * kPi * R / 0.035), 6, 28);
    const Real base = rng_.uniform(0, 2 * kPi);
    for (int i = 0; i < n; ++i) {
        const Real a = base + 2 * kPi * (i + rng_.uniform(-0.25, 0.25)) / n;
        const Vec2 p = s.centre + Vec2{std::cos(a), std::sin(a)} * R;
        if (p.x <= 0 || p.y <= 0 || p.x >= def_.width || p.y >= def_.height) continue;
        const Vec2 tan{-std::sin(a), std::cos(a)};
        const std::uint32_t group = G.net.newGroup();
        for (const Real sgn : {1.0, -1.0}) {
            const int c = G.net.spawn(p, tan * sgn, time_, CrackKind::Concentric, group, 0, G.mat->crackSpeed);
            if (c < 0) continue;
            CrackTip& t = G.net.tips()[c];
            t.fixedStress = true;
            t.anchor = s.centre;
            t.stress = stress;
        }
    }
    s.lastRing = R;
    ++s.rings;
    const Real hh = G.z1 - G.z0;
    s.nextStrength = rng_.weibull(
        G.mat->weibullScale * std::pow(1.0 / (2 * kPi * R * hh * 10), 1.0 / G.mat->weibullModulus),
        G.mat->weibullModulus);
    log(fmt("concentric ring at %.0f mm in ply %.0f", R * 1e3, static_cast<double>(s.ply + 1)));
}

void Pane::probeChecks() {
    Real onset = -1;
    for (const auto& I : imp_)
        if (I.touched) onset = std::max(onset, I.loadOnset);
    if (blastOnset_ >= 0) onset = std::max(onset, blastOnset_);
    if (onset < 0 && toneOnset_ < 0) return;
    const Real rf = strengthFactor();
    // With nothing pressing on the pane there is no near field, and no probe
    // can see more than the plate's curvature bound: skip the faces whose
    // weakest unfired probe that cannot reach. s1 <= E |lever| K (3 - nu) - SC.
    const bool loaded = std::any_of(imp_.begin(), imp_.end(), [](const Impactor& I) {
        return (I.phase == ImpactorPhase::Contact || I.phase == ImpactorPhase::Ballistic ||
                I.phase == ImpactorPhase::Membrane) && I.force > 0;
    });
    if (!glass_.empty()) curvatureAt(glass_[0].probes.empty() ? Vec2{} : glass_[0].probes[0].p);  // refresh if due
    for (std::size_t g = 0; g < glass_.size(); ++g) {
        GlassPly& G = glass_[g];
        if (G.fragmented) continue;
        if (!loaded) {
            const LaminateLayer& L = lam_.layers[G.layer];
            bool skip = true;
            for (int f = 0; f < 2; ++f) {
                if (G.probeMin[f] < 0) {
                    G.probeMin[f] = 1e30;
                    for (const auto& pr : G.probes)
                        if (!pr.fired[f]) G.probeMin[f] = std::min(G.probeMin[f], pr.strength[f]);
                }
                const Real lever = std::fabs(lam_.lever(G.layer, f ? L.z1 : L.z0));
                const Real bound = L.plateModulus * (lever * kGridBound_ * (3 - L.poisson) + (1 + L.poisson) * eGridBound_) -
                                   G.surfaceCompression;
                skip = skip && bound <= G.probeMin[f] * rf;
            }
            if (skip) continue;
        }
        const int nx = G.probeNx, ny = G.probeNy;
        for (int j = 1; j < ny - 1; ++j) {
            for (int i = 1; i < nx - 1; ++i) {
                Probe& pr = G.probes[static_cast<std::size_t>(j) * nx + i];
                if (pr.fired[0] && pr.fired[1]) continue;
                // Under a load the load-point model owns the stress (it spreads
                // the load through the plies); probes start beyond it.
                bool underLoad = false;
                for (const auto& I : imp_)
                    if ((I.phase == ImpactorPhase::Contact || I.phase == ImpactorPhase::Ballistic ||
                         I.phase == ImpactorPhase::Membrane) &&
                        length(pr.p - I.hit) < plate_.resolutionRadius())
                        underLoad = true;
                if (underLoad) continue;
                // Inside a crack star the plate is wedges split by its radials,
                // not the continuous plate the modes describe: its rings come
                // from the wedges (ringCracks), not from the plate's bending wave.
                bool inStar = false;
                for (const auto& st : stars_)
                    if (st.ply == static_cast<int>(g) && length(pr.p - st.centre) < st.reach) inStar = true;
                if (inStar) continue;
                const Curvature kp = curvatureAt(pr.p);
                const PlaneStrain em = membraneAt(pr.p);
                const Real nf = nearFieldAt(pr.p);
                for (int f = 0; f < 2; ++f) {
                    if (pr.fired[f]) continue;
                    const PlaneStress s = stressFromCurvature(static_cast<int>(g), f == 1, kp, nf, false, em);
                    Real s1, s2, ang;
                    s.principal(s1, s2, ang);
                    if (s1 <= pr.strength[f] * rf) continue;
                    pr.fired[f] = true;
                    G.probeMin[f] = -1;
                    if (!G.net.glassAt(pr.p) || G.net.crackedNear(pr.p, 2 * G.net.cell())) continue;
                    lastDynamic_ = time_;
                    if (G.originTime < 0) { G.origin = pr.p; G.originTime = time_; }
                    for (auto& I : imp_) {
                        if (!I.touched || I.report.broke) continue;
                        I.report.broke = true;
                        if (I.phase != ImpactorPhase::Contact)
                            I.report.outcome = "Bounced off, but the pane's bending broke it away from the impact point";
                    }
                    const std::uint32_t group = G.net.newGroup();
                    if (G.dicing) {
                        startRadials(static_cast<int>(g), pr.p, G.net.cell(), 6, group, CrackKind::Dicing, G.mat->crackSpeed);
                        log(fmt("tempered ply %.0f fails at a flaw (%.0f MPa)", static_cast<double>(g + 1), s1 / 1e6));
                        continue;
                    }
                    const Vec2 d = perp(Vec2{std::cos(ang), std::sin(ang)});
                    const CrackKind kind = pr.edge ? CrackKind::Edge : CrackKind::Concentric;
                    G.net.spawn(pr.p, d, time_, kind, group, 0, G.mat->crackSpeed);
                    G.net.spawn(pr.p, -d, time_, kind, group, 0, G.mat->crackSpeed);
                    log(fmt("flaw fails in ply %.0f at %.0f MPa", static_cast<double>(g + 1), s1 / 1e6) + (f ? " (back face)" : " (front face)"));
                }
            }
        }
    }
}

// ---- Blast -------------------------------------------------------------------------------------

Real BlastLoad::pressure(Real t) const {
    const Real tau = t - arrival;
    if (tau < 0) return 0;
    Real p = 0;
    // Friedlander: the positive phase, then a shallow suction, cut off once negligible.
    if (duration > 0 && tau < 4 * duration) p += peak * (1 - tau / duration) * std::exp(-decay * tau / duration);
    if (gasPeak > 0 && gasTau > 0)
        p += gasPeak * (1 - std::exp(-tau / std::max(gasRise, 1e-6))) * std::exp(-tau / gasTau);
    return p;
}

Real BlastLoad::end() const { return arrival + std::max(4 * duration, gasPeak > 0 ? 6 * gasTau : 0.0); }

// The loading-rate factor on strength for what loads the pane now: the
// latest impact or blast; else a tone, by its equivalent time at peak.
Real Pane::strengthFactor() const {
    Real onset = -1;
    for (const auto& I : imp_)
        if (I.touched) onset = std::max(onset, I.loadOnset);
    if (blastOnset_ >= 0) onset = std::max(onset, blastOnset_);
    if (onset >= 0) return rateFactor(time_ - onset);
    return toneOnset_ >= 0 ? rateFactor(kCyclicShare * (time_ - toneOnset_)) : rateFactor(60.0);
}

void Pane::tone(const ToneLoad& t) {
    const bool was = tone_.level > 0;
    tone_ = t;
    tone_.frequency = std::max(t.frequency, 0.1);
    tone_.level = std::max(t.level, 0.0);
    if (tone_.level > 0 && toneOnset_ < 0) toneOnset_ = time_;
    if (tone_.level > 0 && !was)
        log((t.shaker ? fmt("shaker on at (%.2f, %.2f) m: %.1f Hz", t.at.x, t.at.y, tone_.frequency) + fmt(", %.2f N", tone_.level)
                      : fmt("tone on: %.1f Hz, %.0f Pa at the glass", tone_.frequency, tone_.level)) +
            (t.sweep != 0 ? fmt(", gliding %.1f Hz/s", t.sweep) : std::string()));
    else if (tone_.level <= 0 && was) log("tone off");
}

Real Pane::toneValue(Real dt) const {
    // Held over the step at its value in the middle of it.
    return toneAmp_ * std::sin(tonePhase_ + kPi * tone_.frequency * dt);
}

Real Pane::toneSwing() const {
    // One mode swinging at amplitude A holds E = (rho h a b / 4) w^2 A^2 / 2.
    const Real w = 2 * kPi * std::max(tone_.frequency, plate_.fundamentalHz());
    const Real modalMass = lam_.massPerArea * def_.width * def_.height / 4;
    return std::sqrt(2 * plate_.energy() / (modalMass * w * w));
}

// Every flaw against its own strength at this loading rate.
void Pane::toneMeterCheck() {
    const Real rf = strengthFactor();
    for (std::size_t g = 0; g < glass_.size(); ++g) {
        const GlassPly& G = glass_[g];
        if (G.fragmented) continue;
        for (const auto& pr : G.probes) {
            const Curvature k = curvatureAt(pr.p);
            const PlaneStrain em = membraneAt(pr.p);
            for (int f = 0; f < 2; ++f) {
                if (pr.fired[f]) continue;
                Real s1, s2, ang;
                stressFromCurvature(static_cast<int>(g), f == 1, k, 0, false, em).principal(s1, s2, ang);
                const Real st = pr.strength[f] * rf, r = s1 / st;
                if (r > meterRatio_) { meterRatio_ = r; meterStress_ = s1; meterStrength_ = st; }
            }
        }
    }
}

Real Pane::toneMeter(Real& stress, Real& strength) {
    if (!toneActive()) {
        meterRatio_ = 0;
        toneMeterCheck();
    }
    const Real r = std::max(0.0, meterRatio_);
    stress = meterStress_;
    strength = meterStrength_;
    meterRatio_ = -1e30;
    return r;
}

void Pane::blast(const BlastLoad& b) {
    blasts_.push_back(b);
    log(fmt("blast due at %.2f ms: %.0f kPa reflected, %.2f ms positive phase", b.arrival * 1e3, b.peak / 1e3,
            b.duration * 1e3) + (b.gasPeak > 0 ? fmt(", %.0f kPa of gas", b.gasPeak / 1e3) : std::string()));
}

Real Pane::blastPressure(Real t) const {
    Real p = 0;
    for (const auto& b : blasts_) p += b.pressure(t);
    return p;
}

// The pressure on each loose piece still in the frame against what holds it
// there (its frame grip; the crazed sheet's friction is a few times a die's
// weight, nothing against a blast): what cannot hold goes, with the glass's
// own motion and a scatter from the turbulent jet.
void Pane::blowOut(Real p) {
    int n = 0;
    for (int i = 0; i < static_cast<int>(frags_.fragments().size()); ++i) {
        Fragment& f = frags_.fragments()[i];
        if (f.retired || f.chip || f.state != FragmentState::Held || f.clump >= 0) continue;
        if (std::fabs(p) * f.area <= def_.frameGrip * f.edgeContact + f.mass * kGravity) continue;
        f.vel = {rng_.uniform(-0.4, 0.4), rng_.uniform(-0.4, 0.4), plate_.velocity(f.origin.x, f.origin.y)};
        f.omega = {rng_.uniform(-15, 15), rng_.uniform(-15, 15), rng_.uniform(-8, 8)};
        releaseFragment(i);
        ++n;
    }
    if (!n) return;
    for (auto& sh : sheets_) sh.sync(frags_);
    log(fmt("the blast tears %.0f loose pieces out of the frame", static_cast<double>(n)));
}

Real Pane::edgeHold() const { return def_.edgeHold > 0 ? def_.edgeHold : kEdgeBond * def_.bite; }

// The laminate's edge has slid out of the frame: the sheet goes with the
// impactor, folded round it, and its glass breaks into pieces where it folds.
// Dragging the rest of the sheet up to speed over about its own radius takes
// a rim tension of M u^2 / (2 L) / (2 pi a); if the polymer cannot carry that,
// it tears round the impactor, which goes on wrapped in the disc it tore out,
// and the rest of the sheet follows more slowly with what the polymer gave it
// before it tore.
void Pane::pullOut(int k, Real edgePull) {
    Impactor& I = imp_[k];
    pulledOut_ = true;
    const Real m = I.def.mass, v0 = std::max(0.0, I.side * I.vel.z);
    const Vec3 axis = I.side * I.vel.z > 0 ? normalize(I.vel) : Vec3{0, 0, static_cast<Real>(I.side)};
    const Real rim = I.membrane.rimRadius(), strength = I.membrane.strength();
    log(fmt("the laminate's edge pulls out of the frame (%.0f kN/m at the edge, the frame holds %.0f)", edgePull / 1e3,
            edgeHold() / 1e3));
    // The glass is bonded to the interlayer: however it is cracked, the
    // laminate goes as one sheet. Where it folds round the impactor its glass
    // breaks small; the rest stays in hand-sized pieces still stuck to the polymer.
    const Real R = std::max(I.def.radius, 1e-3);
    const Real disc = kWrapDisc * rim;
    const Real foldReach = std::max(R * kWrapCone, disc);
    std::vector<int> pieces;
    for (auto& G : glass_) {
        CrackNetwork& net = G.net;
        for (int t = 0; t < static_cast<int>(net.tips().size()); ++t)
            if (net.tips()[t].alive) net.kill(t, time_);
        const std::size_t before = frags_.fragments().size();
        frags_.build(net, G.ply, 0.5 * (G.z0 + G.z1), G.z1 - G.z0, G.mat->density, time_);
        G.builtRevision = net.topology();
        G.fragmented = true;
        if (G.originTime < 0) { G.origin = I.hit; G.originTime = time_; }
        const std::size_t built = frags_.fragments().size();
        for (std::size_t i = before; i < built; ++i) {
            const Fragment& f = frags_.fragments()[i];
            if (f.chip) continue;
            const bool folds = length(Vec2{f.pos.x, f.pos.y} - I.hit) - f.radius < foldReach + kWrapPiece;
            for (const int part : frags_.breakUp(static_cast<int>(i), folds ? kWrapPiece : kSheetPiece, rng_, time_))
                pieces.push_back(part);
        }
        std::vector<std::uint32_t> crushed;
        for (std::size_t c = 0; c < net.grid().size(); ++c)
            if ((net.grid()[c] & CrackNetwork::kCrush) && !(net.grid()[c] & CrackNetwork::kRemoved))
                crushed.push_back(static_cast<std::uint32_t>(c));
        net.markRemovedCells(crushed);
    }
    Real rMax = 1e-3;
    // (A piece split off another keeps its centroid relative to its parent in origin: use where it is.)
    auto at2 = [&](int i) { const Vec3& q = frags_.fragments()[i].pos; return Vec2{q.x, q.y}; };
    for (const int i : pieces) rMax = std::max(rMax, length(at2(i) - I.hit));
    std::vector<char> inner(pieces.size());
    Real glassIn = 0, glassOut = 0;
    for (std::size_t n = 0; n < pieces.size(); ++n) {
        const Fragment& f = frags_.fragments()[pieces[n]];
        inner[n] = length(at2(pieces[n]) - I.hit) < disc;
        (inner[n] ? glassIn : glassOut) += f.mass;
    }
    // The polymer goes with the glass it is bonded to.
    const Real paneArea = def_.width * def_.height;
    const Real sheetMass = lam_.massPerArea * paneArea;
    const Real discArea = std::min(kPi * disc * disc, paneArea);
    const Real polyArea = std::max(0.0, sheetMass - glassIn - glassOut) / paneArea;
    const Real inMass = glassIn + polyArea * discArea, outMass = glassOut + polyArea * (paneArea - discArea);
    // Wrapped whole: the sheet takes up the impactor's momentum.
    Real u = m * v0 / (m + inMass + outMass), uOut = u;
    const Real need = outMass * u * u / (2 * rMax) / (2 * kPi * rim);
    const bool tore = need > strength;
    if (tore) {
        // The rim pulls the outer sheet with all the polymer has until it tears.
        uOut = u * std::sqrt(strength / need);
        u = std::max(0.0, (m * v0 - outMass * uOut) / (m + inMass));
    }
    // Where each piece ends up once the sheet has folded: round the impactor
    // from its nose, then (wrapped whole) trailing back as a cone, like a
    // canopy; torn off, the outer sheet only dishes round the hole.
    Vec3 radial0{1, 0, 0};
    if (std::fabs(axis.x) > 0.9) radial0 = {0, 1, 0};
    auto turn = [&](const Vec3& from, const Vec3& to) {
        const Vec3 c = cross(from, to);
        const Real d = std::clamp(dot(from, to), -1.0, 1.0), sn = length(c);
        if (sn < 1e-9) return Quat{};
        const Real a = std::atan2(sn, d), h = std::sin(a / 2) / sn;
        return Quat{std::cos(a / 2), c.x * h, c.y * h, c.z * h};
    };
    const Vec3 C = I.pos;
    auto place = [&](Fragment& f, bool wrapsTight, Real dish) {
        const Vec2 d = Vec2{f.pos.x, f.pos.y} - I.hit;
        const Real r = length(d);
        Vec3 rad = r > 1e-9 ? Vec3{d.x / r, d.y / r, 0} : radial0;
        rad = rad - axis * dot(rad, axis);
        rad = length(rad) > 1e-9 ? normalize(rad) : radial0;
        // Depth of the ply below the struck face, along the sheet's normal.
        const Real h = I.side > 0 ? f.pos.z : lam_.thickness - f.pos.z;
        Vec3 at, nrm;
        if (dish > 0) {
            const Real x = r / rMax, slope = -2 * dish * x / rMax;
            at = f.pos + axis * (dish * (1 - x * x));
            nrm = normalize(axis - rad * slope);
        } else {
            const Real end = wrapsTight ? std::min(disc / R, kWrapTight) : kWrapCone;
            const Real th = std::min(r / R, end);
            nrm = axis * std::cos(th) + rad * std::sin(th);
            at = C + nrm * R + (rad * std::cos(th) - axis * std::sin(th)) * std::max(0.0, r - R * th) + nrm * h;
        }
        f.foldOffset = at;  // made relative to the sheet's centre below
        f.foldRot = turn(axis, nrm);
    };
    auto sheet = [&](const std::vector<int>& members, Real speed, Real extra, Real area, Real foldTime, bool carries) {
        if (members.empty()) return -1;
        for (const int i : members) releaseFragment(i);
        const int ci = frags_.makeClump(members, axis * speed,
                                        {rng_.uniform(-1, 1), rng_.uniform(-1, 1), rng_.uniform(-0.5, 0.5)}, time_, rng_);
        Clump& c = frags_.clumps()[ci];
        c.bonded = true;
        c.extraMass = extra;
        c.dragArea = area;
        c.foldStart = time_;
        c.foldTime = foldTime;
        for (const int i : members) frags_.fragments()[i].foldOffset -= c.pos;
        if (carries) {
            I.carriedBy = ci;
            I.carryOffset = C - c.pos;
        }
        return ci;
    };
    const Real fold = std::clamp(rMax / std::max(u, 1.0), 1e-3, 0.3);
    if (!tore) {
        for (const int i : pieces) place(frags_.fragments()[i], false, 0);
        // Folded like a canopy it shows the air about half its area.
        sheet(pieces, u, sheetMass - glassIn - glassOut + m, 0.5 * paneArea, fold, true);
    } else {
        std::vector<int> in, out;
        for (std::size_t n = 0; n < pieces.size(); ++n) {
            Fragment& f = frags_.fragments()[pieces[n]];
            place(f, true, inner[n] ? 0.0 : kWrapDish * rMax);
            (inner[n] ? in : out).push_back(pieces[n]);
        }
        sheet(in, u, polyArea * discArea + m, 0.5 * discArea, std::clamp(disc / std::max(u, 1.0), 0.5e-3, 0.1), true);
        sheet(out, uOut, polyArea * (paneArea - discArea), paneArea - discArea, fold, false);
    }
    for (auto& sh : sheets_) sh.sync(frags_);
    I.vel = axis * u;
    I.force = 0;
    I.phase = ImpactorPhase::Flight;
    I.through = true;
    I.report.penetrated = true;
    I.report.speedOut = u;
    I.report.contactTime = time_ - I.report.contactStart;
    I.report.absorbed = I.report.energyIn - 0.5 * m * u * u;
    if (!tore) {
        I.report.outcome = fmt("Ripped the whole window out of its frame: the laminate wrapped round it and went with "
                               "it at %.0f m/s",
                               u);
    } else {
        holes_.push_back({I.hit, disc, I.vel});
        I.report.outcome =
            uOut > 0.8 * u
                ? fmt("Ripped the window out of its frame; the laminate split round it as it wrapped: on at %.0f m/s "
                      "in a %.0f cm disc of it, the rest of the window right behind at %.0f m/s",
                      u, 200 * disc, uOut)
                : fmt("Ripped the window out of its frame and tore through it: on at %.0f m/s wrapped in a %.0f cm "
                      "disc of laminate, the rest of the window after it at %.0f m/s",
                      u, 200 * disc, uOut);
    }
    bulges_.erase(std::remove_if(bulges_.begin(), bulges_.end(), [k](const Bulge& b) { return b.impactor == k; }),
                  bulges_.end());
    lastDynamic_ = time_;
    log(fmt("%.0f kg of glazing out of the frame; dragging it needs %.0f kN/m at the rim, the polymer carries %.0f",
            sheetMass, need / 1e3, strength / 1e3));
    log(I.report.outcome);
}

// ---- Fragments ----------------------------------------------------------------------------------

void Pane::releaseFragment(int index) {
    Fragment& f = frags_.fragments()[index];
    f.state = FragmentState::Free;
    if (f.chip) return;
    for (auto& G : glass_) {
        if (G.ply != f.ply) continue;
        const auto& map = frags_.maps[f.ply];
        // Only the piece's bounding box can hold its cells.
        const Real r = f.radius + 2 * map.cell;
        const int i0 = std::max(0, static_cast<int>((f.origin.x - r) / map.cell));
        const int i1 = std::min(map.nx - 1, static_cast<int>((f.origin.x + r) / map.cell));
        const int j0 = std::max(0, static_cast<int>((f.origin.y - r) / map.cell));
        const int j1 = std::min(map.ny - 1, static_cast<int>((f.origin.y + r) / map.cell));
        std::vector<std::uint32_t> cells;
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const std::size_t c = static_cast<std::size_t>(j) * map.nx + i;
                if (map.label[c] == index) cells.push_back(static_cast<std::uint32_t>(c));
            }
        G.net.markRemovedCells(cells);
    }
}

void Pane::freezeCheck() {
    if (!monolithic_) return;
    for (auto& G : glass_) {
        CrackNetwork& net = G.net;
        if (net.segments().empty() || net.activeTips() > 0) continue;
        if (time_ - net.lastActivity() < kQuiet) continue;
        if (net.topology() == G.builtRevision) continue;
        bool pressing = false;
        for (const auto& I : imp_) pressing = pressing || I.phase == ImpactorPhase::Contact;
        if (pressing) continue;
        net.killDormant(time_);
        const std::size_t before = frags_.fragments().size();
        // A blast still pushing on broken glass that is not tempered keeps
        // breaking it (below): keep part of the piece budget for that, the
        // finest chips round the origins merging into their neighbours instead.
        const Real pBlast = G.dicing || blastOnset_ < 0 ? 0.0
            : std::max(std::fabs(blastPressure(time_)), std::fabs(blastPressure(std::max(G.originTime, blastOnset_))));
        const int cap = frags_.maxFragments;
        if (pBlast > kBlastHold) frags_.maxFragments = cap - (cap - static_cast<int>(before)) / 4;
        int added = frags_.build(net, G.ply, 0.5 * (G.z0 + G.z1), G.z1 - G.z0, G.mat->density, time_);
        frags_.maxFragments = cap;
        if (pBlast > kBlastHold) {
            const Real p = pBlast;
            for (int pass = 0; pass < 10 && p > kBlastHold && breakUnderBlast(static_cast<int>(&G - glass_.data()), p, before); ++pass)
                frags_.build(net, G.ply, 0.5 * (G.z0 + G.z1), G.z1 - G.z0, G.mat->density, time_);
            added = 0;
            for (std::size_t i = before; i < frags_.fragments().size(); ++i)
                added += !frags_.fragments()[i].retired && !frags_.fragments()[i].chip;
        }
        G.builtRevision = net.topology();
        G.fragmented = true;
        // Crushed glass falls out as grit.
        std::vector<std::uint32_t> crushed;
        for (std::size_t c = 0; c < net.grid().size(); ++c)
            if ((net.grid()[c] & CrackNetwork::kCrush) && !(net.grid()[c] & CrackNetwork::kRemoved))
                crushed.push_back(static_cast<std::uint32_t>(c));
        const int chips = std::min<int>(40, static_cast<int>(crushed.size() / 3));
        for (int c = 0; c < chips; ++c) {
            const std::uint32_t idx = crushed[static_cast<std::size_t>(rng_.uniform() * static_cast<Real>(crushed.size())) % crushed.size()];
            const Vec2 at = net.cellCentre(static_cast<int>(idx % net.nx()), static_cast<int>(idx / net.nx()));
            Vec3 v{rng_.uniform(-0.5, 0.5), rng_.uniform(-0.5, 0.5), plate_.velocity(at.x, at.y)};
            for (const auto& hole : holes_)
                if (length(at - hole.c) < hole.r * 1.5) v = hole.vel * rng_.uniform(0.3, 0.9) + Vec3{rng_.uniform(-1, 1), rng_.uniform(-1, 1), 0};
            frags_.addChip(G.ply, at, 0.5 * (G.z0 + G.z1), rng_.uniform(0.6, 1.6) * net.cell() * 0.5, G.z1 - G.z0,
                           G.mat->density, v, time_, rng_);
        }
        net.markRemovedCells(crushed);
        int freed = 0, pending = 0, held = 0;
        // A blast still pressing on the pane tears out whatever its hold cannot keep.
        const Real pNow = std::fabs(blastPressure(time_));
        const bool blown = pNow > kBlastHold;
        std::vector<int> sheetDice, sheetGone;
        std::vector<char> sheetFrame;
        for (std::size_t i = before; i < frags_.fragments().size(); ++i) {
            Fragment& f = frags_.fragments()[i];
            if (f.chip || f.retired || f.bornAt != time_) continue;
            const Vec2 c = f.origin;
            const Real wd = plate_.velocity(c.x, c.y);
            const Vec2 gr = plate_.velocityGradient(c.x, c.y);
            f.vel = {0, 0, wd};
            f.omega = {gr.y, -gr.x, 0};
            bool pushed = false;
            // Pieces under an object still driving into the glass go with it
            // (it can craze the pane before it has punched its hole).
            for (const auto& I : imp_) {
                if (!I.touched || I.side * I.vel.z <= 0.05) continue;
                // The ring of dice bordering what it crushed is shoved aside too.
                // Only pieces small beside the object go with it: a BB through
                // a pane does not carry the rest of the pane.
                const Real reach = kPushRadius * std::max(I.def.radius, I.contactRadius) + (G.dicing ? G.diceSpacing : 0.0);
                if (f.radius < 4 * reach && length(c - I.hit) < reach + std::min(0.5 * f.radius, reach)) {
                    pushed = true;
                    const Vec2 out = normalize(c - I.hit);
                    f.vel = I.vel * rng_.uniform(0.6, 1.0) + Vec3{out.x, out.y, 0} * (0.1 * length(I.vel));
                    f.omega = {rng_.uniform(-40, 40), rng_.uniform(-40, 40), rng_.uniform(-20, 20)};
                }
            }
            for (const auto& hole : holes_) {
                // Pieces in the impactor's path go with it.
                if (f.radius < 4 * hole.r && length(c - hole.c) < hole.r + std::min(0.5 * f.radius, hole.r)) {
                    pushed = true;
                    const Vec2 out = normalize(c - hole.c);
                    f.vel = hole.vel * rng_.uniform(0.6, 1.0) + Vec3{out.x, out.y, 0} * (0.1 * length(hole.vel));
                    f.omega = {rng_.uniform(-40, 40), rng_.uniform(-40, 40), rng_.uniform(-20, 20)};
                }
            }
            if (G.dicing) {
                f.diceSize = G.diceSize;
                if (pushed) {
                    // Released strain energy pops the dice in the impactor's path.
                    f.vel += Vec3{rng_.uniform(-0.15, 0.15), rng_.uniform(-0.15, 0.15), rng_.uniform(-0.5, 0.5)};
                    f.omega += Vec3{rng_.uniform(-6, 6), rng_.uniform(-6, 6), rng_.uniform(-3, 3)};
                }
            }
            bool supported = false;
            // A piece the frame holds stays whatever the plate is doing under it
            // (its centroid can sit right on the impact); a loose one only if
            // it is not already flying.
            if (!pushed && (f.edges || std::fabs(f.vel.z) < 1.0)) {
                int edgesTouched = 0;
                for (int b = 0; b < 4; ++b) edgesTouched += (f.edges >> b) & 1;
                if ((f.edges & 4) && f.edgeContact >= 0.01) supported = true;      // sits on the setting blocks
                else if (edgesTouched >= 2) supported = true;                      // wedged across a corner
                else if (f.edges && def_.frameGrip * f.edgeContact >= f.mass * kGravity) supported = true;
            }
            if (blown && pNow * f.area > def_.frameGrip * f.edgeContact) supported = false;
            if (blown && !pushed && !supported) {
                f.vel += Vec3{rng_.uniform(-0.4, 0.4), rng_.uniform(-0.4, 0.4), 0};
                f.omega += Vec3{rng_.uniform(-15, 15), rng_.uniform(-15, 15), rng_.uniform(-8, 8)};
            }
            // The dice at the break origin are pulverised and leave a hole.
            if (G.dicing && !pushed && length(c - G.origin) < std::max(1.2 * G.diceSpacing, 2 * net.cell())) {
                pushed = true;
                f.vel = Vec3{rng_.uniform(-0.3, 0.3), rng_.uniform(-0.3, 0.3), rng_.uniform(-0.6, 0.6)};
            }
            if (G.dicing && !pushed && !blown) {
                // Crazed but still in the frame: the sheet decides when it goes.
                f.state = FragmentState::Held;
                f.vel = {};
                f.omega = {};
                sheetDice.push_back(static_cast<int>(i));
                sheetFrame.push_back(supported ? 1 : 0);
                ++pending;
            } else if (supported) {
                f.state = FragmentState::Held;
                f.vel = {};
                f.omega = {};
                ++held;
            } else {
                releaseFragment(static_cast<int>(i));
                if (G.dicing) sheetGone.push_back(static_cast<int>(i));
                ++freed;
            }
        }
        if (G.dicing && sheetDice.size() > 1) {
            // Everything released so far counts as the hole the sheet starts from.
            for (const int k : sheetGone) sheetDice.push_back(k), sheetFrame.push_back(0);
            CrazedSheet sheet;
            sheet.begin(frags_, sheetDice, sheetFrame, sheetGone, G.origin, def_.frameGrip, time_, rng_);
            sheets_.push_back(std::move(sheet));
        }
        if (G.dicing)
            log(fmt("tempered ply crazed into %.0f dice: %.0f fall at once, %.0f hang in the frame", added, freed,
                    pending));
        else
            log(fmt("ply cut into %.0f pieces: %.0f fall, %.0f stay in the frame", added, freed, held));
    }
}

// Glass a blast is still pushing on keeps breaking after its first cracks: a
// piece held by the frame or by the pieces round it bends as a cantilever
// under the pressure, sigma = 3 p L^2 / h^2, so none much longer than
// L = h sqrt(sigma_f / 3p) holds together. Each piece cut since `from` that is
// longer is cracked across its long axis, through its centroid, the crack
// wandering a little until it meets another or an edge. Returns whether it
// cracked any (the ply is then cut into pieces again).
bool Pane::breakUnderBlast(int g, Real p, std::size_t from) {
    GlassPly& G = glass_[g];
    CrackNetwork& net = G.net;
    const Real h = G.z1 - G.z0, cell = net.cell();
    const Real strength = G.mat->weibullScale * rateFactor(time_ - blastOnset_) + G.surfaceCompression;
    const Real L = std::max(6 * cell, h * std::sqrt(strength / (3 * p)));
    const std::uint32_t group = net.newGroup();
    int cracks = 0;
    for (std::size_t i = from; i < frags_.fragments().size(); ++i) {
        const Fragment& f = frags_.fragments()[i];
        if (f.retired || f.chip || f.ply != G.ply || f.bornAt != time_) continue;
        // Its long axis, from the outline's spread about the centroid.
        Real sxx = 0, syy = 0, sxy = 0;
        for (const Vec2& q : f.outline) sxx += q.x * q.x, syy += q.y * q.y, sxy += q.x * q.y;
        const Real ang = 0.5 * std::atan2(2 * sxy, sxx - syy);
        Real lo = 0, hi = 0;
        for (const Vec2& q : f.outline) {
            const Real s = q.x * std::cos(ang) + q.y * std::sin(ang);
            lo = std::min(lo, s), hi = std::max(hi, s);
        }
        if (hi - lo <= L) continue;
        // Across the long axis, from a cell well inside the piece in the middle
        // of its length (a bent piece's middle can lie outside it, and a cut
        // started at its rim may not cross it).
        const auto& m = frags_.maps[G.ply];
        const Vec2 ax{std::cos(ang), std::sin(ang)};
        const int i0 = std::max(0, static_cast<int>((f.origin.x - f.radius) / m.cell)), i1 = std::min(m.nx - 1, static_cast<int>((f.origin.x + f.radius) / m.cell));
        const int j0 = std::max(0, static_cast<int>((f.origin.y - f.radius) / m.cell)), j1 = std::min(m.ny - 1, static_cast<int>((f.origin.y + f.radius) / m.cell));
        Vec2 at;
        int seen = 0;
        for (const Real band : {0.2, 0.5}) {
            for (int jj = j0; jj <= j1; ++jj)
                for (int ii = i0; ii <= i1; ++ii) {
                    if (m.label[static_cast<std::size_t>(jj) * m.nx + ii] != static_cast<int>(i)) continue;
                    const Vec2 c{(ii + 0.5) * m.cell, (jj + 0.5) * m.cell};
                    const Real u = (dot(c - f.origin, ax) - lo) / (hi - lo);
                    if (std::fabs(u - 0.5) > band || !net.glassAt(c) || net.crackedNear(c, 2 * cell)) continue;
                    if (rng_.uniform() * ++seen < 1) at = c;  // one at random
                }
            if (seen) break;
        }
        if (!seen) continue;
        const Real a = ang + 0.5 * kPi + rng_.uniform(-0.25, 0.25);
        for (const Real sgn : {1.0, -1.0}) {
            // Across to the piece's edge: a crack it meets that only runs into
            // the piece (a dead end) does not stop the cut, which carries on
            // beyond it while the glass there is still this piece.
            Vec2 start = at, dir = Vec2{std::cos(a), std::sin(a)} * sgn;
            for (int leg = 0; leg < 24; ++leg) {
                const int t = net.spawn(start, dir, time_, CrackKind::Concentric, group, 0, G.mat->crackSpeed);
                if (t < 0) break;
                for (int k = 0; k < 20000 && net.tips()[t].alive; ++k) {
                    CrackTip& tip = net.tips()[t];
                    tip.dir = rotate(tip.dir, rng_.normal() * 0.035);
                    if (!net.advance(t, tip.pos + tip.dir * cell, time_)) break;
                }
                if (net.tips()[t].alive) net.kill(t, time_);
                const Vec2 end = net.tips()[t].pos;
                dir = net.tips()[t].dir;
                // Past what stopped it (a dead-end crack, or another piece the
                // sheet wraps round): wherever the line is in this piece again.
                bool on = false;
                const int reach = static_cast<int>((hi - lo) / cell) + 2;
                for (int s = 1; s <= reach && !on; ++s) {
                    const Vec2 q = end + dir * (s * cell);
                    if (q.x < 0 || q.y < 0 || q.x > def_.width || q.y > def_.height) break;
                    if (frags_.fragmentAt(G.ply, q) == static_cast<int>(i) && net.glassAt(q) && !net.crackedNear(q, 0.1 * cell))
                        start = q, on = true;
                }
                if (!on) break;
            }
        }
        ++cracks;
    }
    net.compactTips();
    if (cracks)
        log(fmt("the blast keeps breaking ply %.0f: %.0f pieces longer than %.0f cm cracked across", g + 1.0,
                static_cast<double>(cracks), 100 * L));
    return cracks > 0;
}

// ---- Outputs --------------------------------------------------------------------------------------

Real Pane::deflection(Vec2 p) const {
    Real w = plate_.deflection(p.x, p.y);
    for (const auto& b : bulges_) {
        const Real r = length(p - b.c);
        if (r < b.outer) {
            const Impactor& I = imp_[b.impactor];
            Real d = b.depth;
            if (I.phase == ImpactorPhase::Membrane)
                d = std::max(0.0, I.side * (I.pos.z + I.side * I.nose - I.membraneZ0));
            Membrane m;
            m.configure({}, b.rim, b.outer, 1, 1);
            w += I.side * m.shape(r, d);
        }
    }
    return w;
}

void Pane::sampleFields(int g, int nx, int ny, std::vector<float>& w, std::vector<float>& front,
                        std::vector<float>& back) const {
    const std::size_t n = static_cast<std::size_t>(nx) * ny;
    std::vector<Real> ww(n);
    std::vector<Curvature> kk(n);
    std::vector<PlaneStrain> ee(n);
    plate_.sampleGrid(nx, ny, ww.data(), nullptr, kk.data(), ee.data());
    w.resize(n);
    front.resize(n);
    back.resize(n);
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const std::size_t o = static_cast<std::size_t>(j) * nx + i;
            const Vec2 p{def_.width * i / (nx - 1), def_.height * j / (ny - 1)};
            Real extra = 0;
            for (const auto& b : bulges_) {
                const Real r = length(p - b.c);
                if (r >= b.outer) continue;
                const Impactor& I = imp_[b.impactor];
                Real d = b.depth;
                if (I.phase == ImpactorPhase::Membrane)
                    d = std::max(0.0, I.side * (I.pos.z + I.side * I.nose - I.membraneZ0));
                Membrane m;
                m.configure({}, b.rim, b.outer, 1, 1);
                extra += I.side * m.shape(r, d);
            }
            w[o] = static_cast<float>(ww[o] + extra);
            if (g >= 0 && g < static_cast<int>(glass_.size())) {
                const Real nf = nearFieldAt(p);
                Real s1, s2, a;
                stressFromCurvature(g, false, kk[o], nf, false, ee[o]).principal(s1, s2, a);
                front[o] = static_cast<float>(s1);
                stressFromCurvature(g, true, kk[o], nf, false, ee[o]).principal(s1, s2, a);
                back[o] = static_cast<float>(s1);
            }
        }
    }
}

PaneStatus Pane::status() const {
    PaneStatus s;
    Real maxRemoved = 0, maxCracked = 0, maxCrushed = 0, haze = 1;
    for (const auto& G : glass_) {
        const auto& grid = G.net.grid();
        std::size_t removed = 0, cracked = 0, crushed = 0;
        for (auto c : grid) {
            if (c & CrackNetwork::kRemoved) ++removed;
            else if (c & (CrackNetwork::kIdMask | CrackNetwork::kCrush)) ++cracked;
            if ((c & CrackNetwork::kCrush) && !(c & CrackNetwork::kRemoved)) ++crushed;
        }
        const Real cellA = sq(G.net.cell());
        maxRemoved = std::max(maxRemoved, static_cast<Real>(removed) * cellA);
        maxCrushed = std::max(maxCrushed, static_cast<Real>(crushed) * cellA);
        // A crack scatters light over a band about 6 mm wide; cracks lie at
        // random, so a sightline clears one ply with probability exp(-L w / A),
        // and it crosses every ply.
        haze *= std::exp(-G.net.totalLength() * 0.006 / (def_.width * def_.height));
        maxCracked = std::max(maxCracked, static_cast<Real>(cracked) / static_cast<Real>(std::max<std::size_t>(1, grid.size())));
        s.cracked = s.cracked || !G.net.segments().empty();
    }
    const Real area = def_.width * def_.height;
    // Laminates keep their interlayer: only torn holes are open.
    Real holeArea = 0;
    for (const auto& h : holes_) holeArea += kPi * h.r * h.r;
    s.openArea = monolithic_ ? maxRemoved : pulledOut_ ? area : std::min(area, holeArea);
    for (const auto& I : imp_) s.penetrated = s.penetrated || I.report.penetrated;
    s.barrierIntact = s.openArea < 1e-5 && !s.penetrated;
    s.crackedFraction = std::min(1.0, maxCracked * 8);
    // Through the hole the view is clear; crushed (white) glass is opaque;
    // through the rest, hazed by the cracks.
    const Real open = std::min(1.0, s.openArea / area);
    const Real white = std::min(1.0 - open, maxCrushed / area);
    s.visibility = open + (1 - open - white) * haze;
    for (const auto& b : bulges_) s.maxBulge = std::max(s.maxBulge, b.depth);
    for (const auto& f : frags_.fragments())
        if (!f.retired && f.state != FragmentState::Held) ++s.fragments;
    return s;
}

int Pane::fragmentsIn(int g, Vec2 c, Real size) const {
    if (g < 0 || g >= static_cast<int>(glass_.size())) return 0;
    const int ply = glass_[g].ply;
    int n = 0;
    for (const auto& f : frags_.fragments()) {
        if (f.retired || f.chip || f.ply != ply) continue;
        if (std::fabs(f.origin.x - c.x) <= 0.5 * size && std::fabs(f.origin.y - c.y) <= 0.5 * size) ++n;
    }
    return n;
}

}  // namespace gls
