// Glass Dynamics — engine-independent glazing fracture core.
// SPDX-License-Identifier: MIT
#include "gls/library.hpp"

#include <cmath>
#include <functional>

#include "gls/json.hpp"

namespace gls {

// ---- Ply residual stress ----------------------------------------------------
//
// Thermal tempering leaves a parabolic profile through the thickness,
// sigma(zeta) = CT * (1 - 3 zeta^2) with zeta = 2z/t in [-1, 1]: surface
// compression 2*CT, zero crossing 0.211 t below each surface. Ion exchange
// leaves a thin compressive layer, taken as linear over the depth of layer,
// balanced by a uniform central tension.

Real Ply::centralTension() const {
    if (surfaceCompression <= 0) return 0;
    if (treatment == Treatment::Chemical) {
        const Real core = std::max(thickness - 2 * layerDepth, 1e-6);
        return surfaceCompression * layerDepth / core;
    }
    return surfaceCompression / 2;
}

Real Ply::compressionDepth() const {
    if (surfaceCompression <= 0) return 0;
    if (treatment == Treatment::Chemical) return layerDepth;
    return thickness * 0.5 * (1 - 1 / std::sqrt(3.0));
}

Real Ply::residualAt(Real depth) const {
    if (surfaceCompression <= 0) return 0;
    depth = clampr(depth, 0, thickness);
    if (treatment == Treatment::Chemical) {
        if (depth < layerDepth) {
            const Real ct = centralTension();
            return lerp(-surfaceCompression, ct, depth / layerDepth);
        }
        if (depth > thickness - layerDepth) return residualAt(thickness - depth);
        return centralTension();
    }
    const Real zeta = 1 - 2 * depth / thickness;
    return centralTension() * (1 - 3 * zeta * zeta);
}

Real GlazingDef::thickness() const {
    Real t = 0;
    for (const auto& p : plies) t += p.thickness;
    return t;
}

Real GlazingDef::massPerArea() const {
    Real m = 0;
    for (const auto& p : plies) m += p.thickness * (p.mat ? p.mat->density : 2500);
    return m;
}

int GlazingDef::glassPlies() const {
    int n = 0;
    for (const auto& p : plies) n += p.isGlass() ? 1 : 0;
    return n;
}

// ---- Lookup ----------------------------------------------------------------

const Material* Library::material(const std::string& id) const {
    for (const auto& m : materials)
        if (m.id == id) return &m;
    return nullptr;
}
const GlazingDef* Library::glazing(const std::string& id) const {
    for (const auto& g : glazings)
        if (g.id == id) return &g;
    return nullptr;
}
const ImpactorDef* Library::impactor(const std::string& id) const {
    for (const auto& i : impactors)
        if (i.id == id) return &i;
    return nullptr;
}

// ---- JSON loading -------------------------------------------------------------

namespace {

// Reads the members of one definition object; every key must be known.
class Reader {
public:
    Reader(const json::Value& obj, std::string where, std::vector<std::string>& errors)
        : obj_(obj), where_(std::move(where)), errors_(errors) {}

    void num(const char* key, Real scale, Real& out, Real lo, Real hi) {
        const json::Value* v = take(key);
        if (!v) return;
        if (!v->isNumber()) return error(std::string(key) + " must be a number");
        const Real x = v->asNumber() * scale;
        if (!(x >= lo * scale && x <= hi * scale))
            return error(std::string(key) + " = " + std::to_string(v->asNumber()) + " is out of range [" +
                         std::to_string(lo) + ", " + std::to_string(hi) + "]");
        out = x;
    }
    void str(const char* key, std::string& out) {
        const json::Value* v = take(key);
        if (!v) return;
        if (!v->isString()) return error(std::string(key) + " must be a string");
        out = v->asString();
    }
    void provenance(std::map<std::string, std::string>& out) {
        const json::Value* v = take("provenance");
        if (!v) return;
        if (!v->isObject()) return error("provenance must be an object");
        for (const auto& [k, s] : v->members()) {
            if (!s.isString()) { error("provenance." + k + " must be a string"); continue; }
            const std::string& t = s.asString();
            if (t.rfind("published", 0) != 0 && t.rfind("calibrated", 0) != 0 && t.rfind("estimated", 0) != 0 &&
                t.rfind("gameplay", 0) != 0)
                error("provenance." + k + " must start with published, calibrated, estimated or gameplay");
            out[k] = t;
        }
    }
    const json::Value* take(const char* key) {
        used_.push_back(key);
        return obj_.find(key);
    }
    void finish() {
        for (const auto& [k, v] : obj_.members()) {
            bool known = false;
            for (const auto& u : used_) known = known || u == k;
            if (!known) error("unknown field \"" + k + "\"");
        }
    }
    void error(const std::string& m) { errors_.push_back(where_ + ": " + m); ++count_; }
    int errorCount() const { return count_; }

private:
    const json::Value& obj_;
    std::string where_;
    std::vector<std::string>& errors_;
    std::vector<std::string> used_;
    int count_ = 0;
};

template <class T>
void upsert(std::vector<T>& list, T item) {
    for (auto& x : list)
        if (x.id == item.id) { x = std::move(item); return; }
    list.push_back(std::move(item));
}

void readMaterial(const json::Value& o, std::vector<Material>& out, std::vector<std::string>& errors) {
    Material m;
    const json::Value* idv = o.find("id");
    Reader r(o, "material " + (idv && idv->isString() ? idv->asString() : std::string("?")), errors);
    r.str("id", m.id);
    r.str("name", m.name);
    std::string kind = "glass";
    r.str("kind", kind);
    if (kind == "glass") m.kind = MaterialKind::Glass;
    else if (kind == "interlayer") m.kind = MaterialKind::Interlayer;
    else if (kind == "polymer") m.kind = MaterialKind::Polymer;
    else r.error("kind must be glass, interlayer or polymer");
    r.num("density_kgm3", 1, m.density, 100, 20000);
    r.num("youngs_GPa", 1e9, m.youngs, 1e-4, 1000);
    r.num("poisson", 1, m.poisson, 0, 0.499);
    r.num("shear_MPa", 1e6, m.shear, 0, 1e5);
    r.num("toughness_MPam", 1e6, m.toughness, 0.1, 20);
    r.num("hardness_GPa", 1e9, m.hardness, 0.01, 50);
    r.num("branching_MPam", 1e6, m.branching, 0.1, 20);
    r.num("crack_speed_ms", 1, m.crackSpeed, 100, 5000);
    r.num("weibull_modulus", 1, m.weibullModulus, 1, 60);
    r.num("weibull_scale_MPa", 1e6, m.weibullScale, 1, 5000);
    r.num("edge_factor", 1, m.edgeFactor, 0.1, 1.5);
    r.num("corrosion_n", 1, m.corrosionN, 2, 200);
    r.num("comminution_MJm3", 1e6, m.comminution, 0, 1e4);
    Real cone = 68;
    r.num("cone_angle_deg", 1, cone, 10, 85);
    m.coneAngle = deg(cone);
    r.num("tensile_MPa", 1e6, m.tensile, 0, 1e4);
    r.num("failure_strain", 1, m.failureStrain, 0, 20);
    r.num("flow_MPa", 1e6, m.flowStress, 0, 1e4);
    r.provenance(m.provenance);
    r.finish();
    if (m.id.empty()) r.error("missing id");
    if (m.kind != MaterialKind::Glass && m.failureStrain <= 0) r.error("failure_strain is required for polymers");
    if (r.errorCount() == 0) upsert(out, std::move(m));
}

void readGlazing(const json::Value& o, std::vector<GlazingDef>& out, std::vector<std::string>& errors) {
    GlazingDef g;
    const json::Value* idv = o.find("id");
    Reader r(o, "glazing " + (idv && idv->isString() ? idv->asString() : std::string("?")), errors);
    r.str("id", g.id);
    r.str("name", g.name);
    r.str("category", g.category);
    r.str("description", g.description);
    r.str("standard", g.standard);
    r.num("width_m", 1, g.width, 0.02, 6);
    r.num("height_m", 1, g.height, 0.02, 6);
    r.num("damping", 1, g.damping, 0, 0.5);
    r.num("pressure_kPa", 1e3, g.pressure, -200, 200);
    r.num("frame_grip_Nm", 1, g.frameGrip, 0, 1e5);
    r.num("bite_mm", 1e-3, g.bite, 0, 50);
    r.provenance(g.provenance);
    const json::Value* plies = r.take("plies");
    if (!plies || !plies->isArray() || plies->items().empty()) {
        r.error("plies must be a non-empty array");
    } else {
        int k = 0;
        for (const auto& pv : plies->items()) {
            Ply p;
            Reader pr(pv, "glazing " + g.id + " ply " + std::to_string(k++), errors);
            if (!pv.isObject()) { pr.error("must be an object"); continue; }
            pr.str("material", p.material);
            pr.num("thickness_mm", 1e-3, p.thickness, 0.01, 100);
            std::string t = "annealed";
            pr.str("treatment", t);
            if (t == "annealed") p.treatment = Treatment::Annealed;
            else if (t == "heat_strengthened") p.treatment = Treatment::HeatStrengthened;
            else if (t == "fully_tempered") p.treatment = Treatment::FullyTempered;
            else if (t == "chemical") p.treatment = Treatment::Chemical;
            else pr.error("treatment must be annealed, heat_strengthened, fully_tempered or chemical");
            pr.num("surface_compression_MPa", 1e6, p.surfaceCompression, 0, 1500);
            pr.num("layer_depth_um", 1e-6, p.layerDepth, 0, 2000);
            pr.finish();
            if (p.treatment == Treatment::Chemical && p.layerDepth <= 0) pr.error("chemical plies need layer_depth_um");
            if (p.material.empty() || p.thickness <= 0) pr.error("needs material and thickness_mm");
            if (pr.errorCount()) r.error("bad ply");
            g.plies.push_back(p);
        }
    }
    r.finish();
    if (g.id.empty()) r.error("missing id");
    if (r.errorCount() == 0) upsert(out, std::move(g));
}

void readImpactor(const json::Value& o, std::vector<ImpactorDef>& out, std::vector<std::string>& errors) {
    ImpactorDef d;
    const json::Value* idv = o.find("id");
    Reader r(o, "impactor " + (idv && idv->isString() ? idv->asString() : std::string("?")), errors);
    r.str("id", d.id);
    r.str("name", d.name);
    r.str("category", d.category);
    r.str("description", d.description);
    std::string kind = "sphere";
    r.str("kind", kind);
    if (kind == "sphere") d.kind = ImpactorKind::Sphere;
    else if (kind == "sharp") d.kind = ImpactorKind::Sharp;
    else if (kind == "flat") d.kind = ImpactorKind::Flat;
    else if (kind == "projectile") d.kind = ImpactorKind::Projectile;
    else if (kind == "soft") d.kind = ImpactorKind::SoftBody;
    else r.error("kind must be sphere, sharp, flat, projectile or soft");
    r.num("mass_g", 1e-3, d.mass, 0.01, 1e6);
    r.num("radius_mm", 1e-3, d.radius, 0.1, 2000);
    r.num("length_mm", 1e-3, d.length, 0, 5000);
    r.num("contact_radius_mm", 1e-3, d.contactRadius, 0, 2000);
    r.num("youngs_GPa", 1e9, d.youngs, 1e-6, 1200);
    r.num("poisson", 1, d.poisson, 0, 0.499);
    r.num("density_kgm3", 1, d.density, 10, 25000);
    r.num("yield_MPa", 1e6, d.yield, 0, 1e4);
    r.num("restitution", 1, d.restitution, 0.05, 1);
    Real tip = 60;
    r.num("tip_angle_deg", 1, tip, 5, 89);
    d.tipAngle = deg(tip);
    r.num("speed_ms", 1, d.speed, 0, 3000);
    r.num("min_speed_ms", 1, d.minSpeed, 0, 3000);
    r.num("max_speed_ms", 1, d.maxSpeed, 0, 3000);
    r.num("porosity", 1, d.porosity, 0, 0.9);
    r.provenance(d.provenance);
    r.finish();
    if (d.id.empty()) r.error("missing id");
    if (r.errorCount() == 0) upsert(out, std::move(d));
}

}  // namespace

std::vector<std::string> Library::load(const std::string& jsonText) {
    std::vector<std::string> errors;
    const json::ParseResult pr = json::parse(jsonText);
    if (!pr.ok()) {
        errors.push_back("JSON " + pr.error);
        return errors;
    }
    if (!pr.value.isObject()) {
        errors.push_back("top level must be an object");
        return errors;
    }
    for (const auto& [key, v] : pr.value.members()) {
        if (key == "comment" || key == "schema") continue;
        if (!v.isArray()) { errors.push_back(key + " must be an array"); continue; }
        for (const auto& item : v.items()) {
            if (!item.isObject()) { errors.push_back(key + ": entries must be objects"); continue; }
            if (key == "materials") readMaterial(item, materials, errors);
            else if (key == "glazings") readGlazing(item, glazings, errors);
            else if (key == "impactors") readImpactor(item, impactors, errors);
            else { errors.push_back("unknown top-level array \"" + key + "\""); break; }
        }
    }
    return errors;
}

std::vector<std::string> Library::resolve() {
    std::vector<std::string> errors;
    for (auto& g : glazings) {
        for (auto& p : g.plies) {
            p.mat = material(p.material);
            if (!p.mat) errors.push_back("glazing " + g.id + ": unknown material " + p.material);
            else if (p.mat->kind != MaterialKind::Glass && p.surfaceCompression > 0)
                errors.push_back("glazing " + g.id + ": residual stress on a non-glass ply");
        }
        if (g.glassPlies() == 0) errors.push_back("glazing " + g.id + ": needs at least one glass ply");
    }
    return errors;
}

}  // namespace gls
