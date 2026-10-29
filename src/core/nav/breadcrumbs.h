/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"

// ============================================================================
// Breadcrumbs - the path this flight took, kept so that the return home can fly it backwards (SmartRTL). ArduPilot
// AP_SmartRTL (libraries/AP_SmartRTL), written anew and smaller:
//
//   record     the first point at arming (home), then one whenever the craft has moved accuracy_m (SRTL_ACCURACY, 2 m) from
//              the last one (add_point)
//   loops      when the newest segment passes closer than 0.99 x accuracy_m (SMARTRTL_PRUNING_DELTA) to an older one that
//              does not touch it, everything flown in between is a loop: the points of it are replaced by the middle of
//              where the two segments came closest (detect_loops / segment_segment_dist / add_loop). ArduPilot searches in
//              a background thread and cuts when the store runs low or the return starts; here the newest segment is
//              checked as it comes (the same cuts, a little work per point, no thread)
//   simplify   when the store is full: Ramer-Douglas-Peucker, points closer than 0.5 x accuracy_m (SMARTRTL_SIMPLIFY_EPSILON)
//              to the line through their neighbours go (detect_simplifications). Still full: record() returns false and the
//              caller gives the path up after 15 s of that (SMARTRTL_TIMEOUT) - the return then flies straight home
//
// Fixed memory (N points of 12 bytes), no recursion, no allocation. Units m.
// ============================================================================
namespace fc {
namespace nav {

struct Crumb { float n = 0, e = 0, z = 0; };

// closest approach of the segments p1-p2 and q1-q2 (3-D); the squared distance, and the two closest points (Dan Sunday)
inline WB_FAST float segment_gap2(const Crumb& p1, const Crumb& p2, const Crumb& q1, const Crumb& q2, Crumb& cp, Crumb& cq) {
    const float u[3] = {p2.n - p1.n, p2.e - p1.e, p2.z - p1.z}, v[3] = {q2.n - q1.n, q2.e - q1.e, q2.z - q1.z}, w[3] = {p1.n - q1.n, p1.e - q1.e, p1.z - q1.z};
    const float a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2], b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2], c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    const float d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2], e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
    const float den = a * c - b * b;
    float sn, sd = den, tn, td = den;
    if (den < 1e-6f) { sn = 0.0f; sd = 1.0f; tn = e; td = c; }                 // parallel: any point of p against q
    else {
        sn = b * e - c * d; tn = a * e - b * d;
        if (sn < 0.0f) { sn = 0.0f; tn = e; td = c; } else if (sn > sd) { sn = sd; tn = e + b; td = c; }
    }
    if (tn < 0.0f) { tn = 0.0f; if (-d < 0.0f) sn = 0.0f; else if (-d > a) sn = sd; else { sn = -d; sd = a; } }
    else if (tn > td) { tn = td; if (-d + b < 0.0f) sn = 0.0f; else if (-d + b > a) sn = sd; else { sn = -d + b; sd = a; } }
    const float s = (sd > 1e-9f && fabsf(sn) > 1e-9f) ? sn / sd : 0.0f, t = (td > 1e-9f && fabsf(tn) > 1e-9f) ? tn / td : 0.0f;
    cp.n = p1.n + s * u[0]; cp.e = p1.e + s * u[1]; cp.z = p1.z + s * u[2];
    cq.n = q1.n + t * v[0]; cq.e = q1.e + t * v[1]; cq.z = q1.z + t * v[2];
    const float dn = cp.n - cq.n, de = cp.e - cq.e, dz = cp.z - cq.z;
    return dn * dn + de * de + dz * dz;
}

// squared distance of x from the segment a-b (3-D)
inline WB_FAST float point_segment2(const Crumb& x, const Crumb& a, const Crumb& b) {
    const float u[3] = {b.n - a.n, b.e - a.e, b.z - a.z}, w[3] = {x.n - a.n, x.e - a.e, x.z - a.z};
    const float l2 = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
    float t = l2 > 1e-9f ? (u[0] * w[0] + u[1] * w[1] + u[2] * w[2]) / l2 : 0.0f;
    t = clampf(t, 0.0f, 1.0f);
    const float dn = w[0] - t * u[0], de = w[1] - t * u[1], dz = w[2] - t * u[2];
    return dn * dn + de * de + dz * dz;
}

template <int N>
class Breadcrumbs {
    static_assert(N >= 8, "a path of a few points at least");
public:
    float accuracy_m = 2.0f;                         // SRTL_ACCURACY

    void reset() { n_ = 0; ok_ = true; loops_ = 0; simplified_ = 0; }
    void give_up() { ok_ = false; }                  // (the caller: the store stayed full, or the position was lost, for too long)
    int size() const { return n_; }
    const Crumb& at(int i) const { return pts_[i]; }
    bool ok() const { return ok_; }
    uint16_t loops_cut() const { return loops_; }
    uint16_t simplified() const { return simplified_; }

    // where the craft is now; kept if it has moved far enough from the last point. false: the store is full, it was not kept
    WB_FAST bool record(float n, float e, float z) {
        if (!ok_) return true;
        Crumb p; p.n = n; p.e = e; p.z = z;
        if (n_ > 0) {
            const Crumb& l = pts_[n_ - 1];
            const float dn = n - l.n, de = e - l.e, dz = z - l.z;
            if (dn * dn + de * de + dz * dz < accuracy_m * accuracy_m) return true;
        }
        if (n_ >= 3) cut_loop(p);
        if (n_ >= N) simplify(0.5f * accuracy_m);
        if (n_ >= N) return false;
        pts_[n_++] = p;
        return true;
    }

private:
    // the newest segment (last point -> p) against every older one that does not touch it
    void cut_loop(const Crumb& p) {
        const Crumb& last = pts_[n_ - 1];
        const float lim2 = (0.99f * accuracy_m) * (0.99f * accuracy_m);
        for (int i = 0; i + 2 < n_; i++) {                       // segment i .. i+1; i+1 < n_-1 keeps it from sharing a point
            Crumb a, b;
            if (segment_gap2(last, p, pts_[i], pts_[i + 1], a, b) < lim2) {
                Crumb m; m.n = 0.5f * (a.n + b.n); m.e = 0.5f * (a.e + b.e); m.z = 0.5f * (a.z + b.z);
                n_ = i + 1;                                      // keep 0 .. i, then the meeting point (p follows)
                pts_[n_++] = m;
                loops_++;
                return;
            }
        }
    }
    // Ramer-Douglas-Peucker over the whole store, an explicit stack instead of recursion
    void simplify(float tol) {
        bool keep[N];
        for (int i = 0; i < n_; i++) keep[i] = false;
        keep[0] = keep[n_ - 1] = true;
        int stack[2 * N], sp = 0;
        stack[sp++] = 0; stack[sp++] = n_ - 1;
        const float tol2 = tol * tol;
        while (sp > 0) {
            const int b = stack[--sp], a = stack[--sp];
            float worst = 0; int at = -1;
            for (int i = a + 1; i < b; i++) { const float d = point_segment2(pts_[i], pts_[a], pts_[b]); if (d > worst) { worst = d; at = i; } }
            if (at >= 0 && worst > tol2) { keep[at] = true; if (sp + 4 <= 2 * N) { stack[sp++] = a; stack[sp++] = at; stack[sp++] = at; stack[sp++] = b; } }
        }
        int m = 0;
        for (int i = 0; i < n_; i++) if (keep[i]) pts_[m++] = pts_[i];
        if (m < n_) simplified_++;
        n_ = m;
    }

    Crumb pts_[N];
    int n_ = 0; bool ok_ = true; uint16_t loops_ = 0, simplified_ = 0;
};

}  // namespace nav
}  // namespace fc
