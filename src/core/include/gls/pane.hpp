// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// One framed pane and everything that hits it. The pane frame: x along the
// width, y up (the pane's bottom edge at y = 0), z through the thickness from
// the front face (z = 0) toward the back face; the plies are stacked front
// first. Time advances in whole internal steps whose size follows what is
// happening (0.05 us inside a bullet strike, 0.5 ms for falling fragments),
// so the result does not depend on how a host chops its frames.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gls/contact.hpp"
#include "gls/cracks.hpp"
#include "gls/fragments.hpp"
#include "gls/library.hpp"
#include "gls/plate.hpp"
#include "gls/rng.hpp"

namespace gls {

// Work limits. A low tier keeps the physics and coarsens its resolution: fewer
// plate modes, a coarser crack grid, fewer crack tips and fragments.
struct Budget {
    const char* name = "high";
    int modes = 32;                 // plate modes along the longer side
    Real crackCell = 0.002;         // m, crack grid cell (raised to fit maxCells)
    int maxCells = 500000;          // per glass ply
    int maxTips = 768;
    std::size_t maxSegments = 250000;
    int maxFragments = 6000;
    Real probeSpacing = 0.02;       // m, flaw probe lattice
    Real minDiceSpacing = 0.005;    // m, coarsest tempered fragment the tier draws
};
Budget budgetForTier(int tier);  // 0 potato, 1 low, 2 medium, 3 high, 4 ultra
inline constexpr int kTierCount = 5;

enum class ImpactorPhase : std::uint8_t {
    Flight = 0,      // approaching (or leaving) the pane
    Contact = 1,     // pressing on the glass (Hertz, sharp, flat, soft body)
    Ballistic = 2,   // penetrating the layup
    Membrane = 3,    // pulling the interlayer of a broken laminate
    Done = 4,        // at rest or gone
};

struct ImpactReport {
    Real speedIn = 0, speedOut = 0;   // m/s along the travel
    Real energyIn = 0, absorbed = 0;  // J
    Real peakForce = 0;               // N
    Real contactStart = -1, contactTime = 0;  // s
    Real maxStress = 0;               // Pa, peak principal stress seen at the load point
    Real depth = 0;                   // m, deepest point reached into the layup
    Real bulge = 0;                   // m, interlayer / backing deflection
    bool broke = false, penetrated = false, rebounded = false, embedded = false, diced = false;
    std::string outcome;
};

struct Impactor {
    ImpactorDef def;
    Vec3 pos, vel;                    // centre of mass, pane frame
    Vec3 axis{0, 0, 1};               // travel direction at launch (renderers orient by it)
    Real nose = 0;                    // m from the centre to the contact point along the travel
    int side = 1;                     // +1 strikes the front face, -1 the back face
    ImpactorPhase phase = ImpactorPhase::Flight;
    Vec2 hit;                         // pane point of contact
    Real force = 0, contactRadius = 0;
    Real squash = 0;                  // m, soft-body flattening (rendering)
    bool touched = false, through = false;
    ImpactReport report;

    // Contact models and progress.
    SphereContact sphere;
    SharpContact sharp;
    FlatContact flat;
    SoftBodyLoad soft;
    Ballistic ballistic;
    Membrane membrane;
    Real membraneZ0 = 0, membraneMax = 0, membraneSet = 0;
    // Pushing through a broken monolithic pane: the force decays while the
    // impactor shoves the broken glass ahead of it.
    bool pushing = false, wedge = false;
    Real pushStart = 0, pushForce = 0, pushTau = 0, pushZ0 = 0;
    Real loadOnset = 0;
    Real contactStep = 0.2e-6;        // s, from the contact stiffness at the expected peak
    Real localW = 0;                  // m, plate motion under the load the modes are too coarse to carry
    std::vector<char> brokenAtLoad;   // per ply
    std::vector<Real> strengthAtLoad; // per ply-face, drawn at contact
    Real coneStrength = 0;
    bool cone = false;
    Real coneDepth = 0;               // m, contact damage (cone / median crack) under the struck face
    std::size_t ballisticEventsSeen = 0;
};

// The crack star at a load point: the wedges between radial cracks bend as
// cantilevers, so the face nearest the impact breaks in rings at their roots.
struct Star {
    int ply = 0;
    int impactor = 0;
    Vec2 centre;
    Real reach = 0, lastRing = 0, born = 0, nextStrength = 0;
    int rings = 0;
    std::uint32_t group = 0;
};

struct Probe {
    Vec2 p;
    Real strength[2] = {0, 0};  // Pa: front face, back face
    bool fired[2] = {false, false};
    bool edge = false;
};

struct GlassPly {
    int ply = 0;           // index into the glazing's plies
    int layer = 0;         // index into the laminate's layers
    Real z0 = 0, z1 = 0;
    const Material* mat = nullptr;
    Real centralTension = 0, surfaceCompression = 0, compressionDepth = 0;
    bool dicing = false;   // residual tension forks running cracks (fully tempered)
    Real diceSpacing = 0;  // m, branch spacing drawn (energy balance, coarsened to the budget)
    Real diceSize = 0;     // m, energy-balance fragment size of the real glass
    CrackNetwork net;
    std::vector<Probe> probes;
    int probeNx = 0, probeNy = 0;
    std::uint64_t builtRevision = 0;
    bool fragmented = false;
    Vec2 origin;           // first fracture origin
    Real originTime = -1;
};

// What a host (vehicle, aircraft) needs back from its glazing.
struct PaneStatus {
    bool cracked = false;
    bool penetrated = false;        // something passed through
    bool barrierIntact = true;      // still separates the two sides (pressure, wind)
    Real openArea = 0;              // m^2 of missing glazing
    Real crackedFraction = 0;       // of the area, behind cracks
    Real visibility = 1;            // 1 clear .. 0 opaque (crack density)
    Real maxBulge = 0;              // m, out-of-plane permanent deformation
    int fragments = 0;              // free pieces
};

class Pane {
public:
    Pane(const GlazingDef& def, const Budget& budget, std::uint64_t seed = 1);

    // An impactor at pane-frame centre `pos` moving at `vel`. Returns its index.
    int launch(const ImpactorDef& def, const Vec3& pos, const Vec3& vel);
    // Aims at pane point `at` from `lead` metres away at `speed`, with the
    // travel tilted by yaw (about y) and pitch (about x). fromBack strikes
    // the back face.
    int fire(const ImpactorDef& def, Vec2 at, Real speed, Real yaw = 0, Real pitch = 0, bool fromBack = false,
             Real lead = 0.25);

    // Whole internal steps until time() >= t, at most maxSteps. Returns the
    // number of steps taken.
    int advance(Real t, int maxSteps = 1 << 30);
    Real time() const { return time_; }
    Real lastStep() const { return lastDt_; }
    bool quiet() const;  // nothing moving and nothing left to happen

    const GlazingDef& glazing() const { return def_; }
    const Laminate& laminate() const { return lam_; }
    const ModalPlate& plate() const { return plate_; }
    const Budget& budget() const { return budget_; }
    std::vector<GlassPly>& glass() { return glass_; }
    const std::vector<GlassPly>& glass() const { return glass_; }
    FragmentSystem& fragments() { return frags_; }
    const FragmentSystem& fragments() const { return frags_; }
    std::vector<Impactor>& impactors() { return imp_; }
    const std::vector<Impactor>& impactors() const { return imp_; }
    const std::vector<Star>& stars() const { return stars_; }

    // Bending plus residual stress on a face of glass ply `g` (index into glass()).
    PlaneStress stress(int g, bool backFace, Vec2 p) const;
    // Out-of-plane deflection (m) including interlayer bulges.
    Real deflection(Vec2 p) const;
    // Fields on an nx x ny lattice (x fastest): deflection (m) and the
    // largest principal stress of glass ply g's front and back faces (Pa).
    void sampleFields(int g, int nx, int ny, std::vector<float>& w, std::vector<float>& front,
                      std::vector<float>& back) const;

    PaneStatus status() const;
    // Fragments whose centroid lies in the square of side `size` centred at c
    // (ECE R43 / EN 12150 count), for glass ply g.
    int fragmentsIn(int g, Vec2 c, Real size) const;
    // Text log of fracture events, time-stamped.
    const std::vector<std::string>& events() const { return events_; }

    Real floorY() const { return frags_.floorY; }
    Real fragmentTime() const { return fragTime_; }  // when the pieces were last stepped
    void setFloor(Real y) { frags_.floorY = y; }

private:
    Real chooseStep() const;
    void step(Real dt);
    void stepImpactor(int k, Real dt);
    void beginContact(int k);
    void contactStepFor(Impactor& I);
    void endContact(int k, const char* why);
    void loadPointChecks(int k);
    void breakAtLoad(int k, int g, Real sigma, Real strength, int radials);
    void startRadials(int g, Vec2 c, Real r0, int n, std::uint32_t group, CrackKind kind, Real speed);
    void startMembrane(int k);
    void passThrough(int k, Real massShare);
    void ballisticEvents(int k);
    void growCracks(Real dt);
    void ringCracks();
    void spawnRing(Star& s, Real radius, Real stress);
    void probeChecks();
    void freezeCheck();
    void releaseFragment(int index);
    Real rateFactor(Real loadTime) const;
    Real nearFieldAt(Vec2 p) const;
    PlaneStress stressFromCurvature(int g, bool back, const Curvature& k, Real extraIso, bool layered = false) const;
    // Curvature at p from a lattice refreshed every few microseconds (crack
    // tips sample it; the plate's modes are band-limited, so bilinear
    // interpolation on a lattice finer than the shortest mode loses nothing).
    Curvature curvatureAt(Vec2 p);
    void log(const std::string& s);
    bool monolithic() const { return monolithic_; }

    GlazingDef def_;
    std::vector<std::unique_ptr<Material>> mats_;
    Budget budget_;
    Laminate lam_;
    ModalPlate plate_;
    std::vector<GlassPly> glass_;
    FragmentSystem frags_;
    std::vector<Impactor> imp_;
    std::vector<Star> stars_;
    std::vector<std::string> events_;
    Rng rng_;
    Real time_ = 0, lastDt_ = 0, lastProbe_ = -1, lastDynamic_ = -1e9, lastLoaded_ = -1e9;
    bool monolithic_ = true;
    std::vector<MembranePly> membranePlies_;
    Real fragTime_ = 0;  // pieces step at their own fixed rate
    std::vector<Curvature> kGrid_;
    int kNx_ = 0, kNy_ = 0;
    Real kGridTime_ = -1;
    struct Bulge {
        Vec2 c;
        Real rim = 0, outer = 0, depth = 0;
        int impactor = -1;
    };
    std::vector<Bulge> bulges_;
    struct Hole {
        Vec2 c;
        Real r = 0;
        Vec3 vel;  // what punched it, as it left
    };
    std::vector<Hole> holes_;
};

}  // namespace gls
