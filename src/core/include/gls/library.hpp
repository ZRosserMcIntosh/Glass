// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
//
// Definitions: materials, glazing layups and impactors, loaded from JSON with
// a provenance note per field (published / calibrated / estimated). The
// loader converts the unit-suffixed JSON fields (`thickness_mm`, `youngs_GPa`)
// to SI once; everything after this header is SI.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "gls/math.hpp"

namespace gls {

enum class MaterialKind { Glass, Interlayer, Polymer };

struct Material {
    std::string id, name;
    MaterialKind kind = MaterialKind::Glass;
    Real density = 2500;   // kg/m^3
    Real youngs = 72e9;    // Pa (interlayers: secant modulus at impact strain rates)
    Real poisson = 0.22;
    Real shear = 0;        // Pa; interlayer shear modulus at impact rates (0: from E and nu)

    // Glass.
    Real toughness = 0.75e6;          // K_Ic, Pa*sqrt(m)
    Real hardness = 5.5e9;            // Vickers hardness, Pa
    Real branching = 2.1e6;           // A_b, Pa*sqrt(m): sigma * sqrt(r_branch)
    Real crackSpeed = 1520;           // terminal crack velocity, m/s
    Real weibullModulus = 7;          // m
    Real weibullScale = 70e6;         // sigma_theta, Pa: surface, 1 m^2, 60 s load
    Real edgeFactor = 0.8;            // cut-edge strength / surface strength
    Real corrosionN = 16;             // stress-corrosion exponent n (dynamic fatigue)
    Real comminution = 0;             // J/m^3 absorbed crushing the ballistic cone
    Real coneAngle = deg(68);         // Hertzian / ballistic cone half-angle from the load axis

    // Interlayers and polymers.
    Real tensile = 0;                 // Pa, strength at impact rates
    Real failureStrain = 0;           // engineering strain at break
    Real flowStress = 0;              // Pa; interlayer membrane flow (yield) stress

    std::map<std::string, std::string> provenance;  // field -> "published: source"
    Real shearModulus() const { return shear > 0 ? shear : youngs / (2 * (1 + poisson)); }
    Real plateModulus() const { return youngs / (1 - poisson * poisson); }
};

// Residual-stress state of a glass ply.
enum class Treatment { Annealed, HeatStrengthened, FullyTempered, Chemical };

struct Ply {
    std::string material;
    Real thickness = 0;             // m
    Treatment treatment = Treatment::Annealed;
    Real surfaceCompression = 0;    // Pa, positive = compressive (residual)
    Real layerDepth = 0;            // m, chemical ion-exchange depth of layer
    const Material* mat = nullptr;  // resolved by Library

    bool isGlass() const { return mat && mat->kind == MaterialKind::Glass; }
    // Mid-plane tension balancing the surface compression.
    Real centralTension() const;
    // Depth of the compressive layer from each surface.
    Real compressionDepth() const;
    // Residual stress at depth d below a surface (negative = compression).
    Real residualAt(Real depth) const;
};

struct GlazingDef {
    std::string id, name, category, description, standard;
    Real width = 1, height = 1;     // m (pane x along width, y up)
    std::vector<Ply> plies;         // front (impact) face first
    Real damping = 0.01;            // modal damping ratio
    Real pressure = 0;              // Pa, static differential load (+ pushes toward the back face)
    Real frameGrip = 40;            // N/m of edge contact holding loose fragments
    Real bite = 0.012;              // m, frame rebate covering the pane edges
    std::map<std::string, std::string> provenance;

    Real thickness() const;
    Real massPerArea() const;
    int glassPlies() const;
};

enum class ImpactorKind { Sphere, Sharp, Flat, Projectile, SoftBody };

struct ImpactorDef {
    std::string id, name, category, description;
    ImpactorKind kind = ImpactorKind::Sphere;
    Real mass = 0.227;              // kg
    Real radius = 0.019;            // m, body radius (rendering, momentum footprint)
    Real length = 0;                // m (projectiles, birds); 0: sphere
    Real contactRadius = 0;         // m, curvature at the contact (sphere: = radius; sharp: tip)
    Real youngs = 210e9, poisson = 0.3, density = 7850;
    Real yield = 0;                 // Pa: flow / crush stress (0 = elastic)
    Real restitution = 1;           // < 1 adds Hunt-Crossley damping (balls, bags)
    Real tipAngle = deg(60);        // sharp tips: cone half-angle
    Real speed = 5, minSpeed = 0.5, maxSpeed = 50;  // m/s, the demo's range
    Real porosity = 0;              // soft bodies (bird substitute)
    std::map<std::string, std::string> provenance;

    Real contactCurvature() const { return contactRadius > 0 ? contactRadius : radius; }
};

class Library {
public:
    std::vector<Material> materials;
    std::vector<GlazingDef> glazings;
    std::vector<ImpactorDef> impactors;

    const Material* material(const std::string& id) const;
    const GlazingDef* glazing(const std::string& id) const;
    const ImpactorDef* impactor(const std::string& id) const;

    // Adds (or replaces, by id) the definitions in a JSON document with any
    // of the top-level arrays "materials", "glazings", "impactors". Returns
    // the errors; definitions that fail validation are not added.
    std::vector<std::string> load(const std::string& jsonText);
    // Resolves ply material pointers (after all files are loaded).
    std::vector<std::string> resolve();
};

}  // namespace gls
