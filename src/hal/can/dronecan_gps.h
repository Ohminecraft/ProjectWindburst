/*
 * This file is part of Windburst Project
 */

#pragma once

#include <math.h>
#include "dronecan_msgs.h"
#include "../gps/gps_types.h"

namespace dcan {

// A DroneCAN fix as the GPS layer keeps it (what the u-blox driver fills from NAV-PVT). aux: the module's dilutions of
// precision, when it sends them (gnss.Auxiliary); without them the PDOP stands in for the HDOP (it is never smaller).
inline gps::Solution to_solution(const Fix2& f, const GnssAux* aux) {
    gps::Solution s;
    s.lat = (int32_t)(f.lat_1e8 / 10); s.lon = (int32_t)(f.lon_1e8 / 10); s.alt_cm = f.height_msl_mm / 10;
    s.fix_type = f.status == GNSS_3D ? 3 : f.status == GNSS_2D ? 2 : f.status == GNSS_TIME_ONLY ? 5 : 0;
    s.fix = f.status == GNSS_3D; s.num_sat = f.sats_used;
    const float vn = f.ned_velocity[0], ve = f.ned_velocity[1], vd = f.ned_velocity[2];
    auto cms = [](float v) { const float c = v * 100.0f; return (int16_t)(c > 32767.0f ? 32767 : c < -32767.0f ? -32767 : lroundf(c)); };
    s.vel_n_cms = cms(vn); s.vel_e_cms = cms(ve); s.vel_d_cms = cms(vd); s.has_vel_d = true;
    const float gs = sqrtf(vn * vn + ve * ve), s3 = sqrtf(vn * vn + ve * ve + vd * vd);
    s.ground_speed_cms = (uint16_t)fminf(gs * 100.0f, 65535.0f); s.speed3d_cms = (uint16_t)fminf(s3 * 100.0f, 65535.0f);
    float course = atan2f(ve, vn) * 57.29578f; if (course < 0) course += 360.0f;
    s.course_ddeg = (uint16_t)((int)lroundf(course * 10.0f) % 3600);
    s.pdop = (uint16_t)fminf(f.pdop * 100.0f, 9999.0f);
    s.hdop = aux && aux->hdop > 0 ? (uint16_t)fminf(aux->hdop * 100.0f, 9999.0f) : s.pdop;
    s.vdop = aux && aux->vdop > 0 ? (uint16_t)fminf(aux->vdop * 100.0f, 9999.0f) : 0;
    // accuracies from the covariance: 6 values (the diagonal: position N E D, velocity N E D) or the full 6 x 6 matrix
    if (f.cov_len == 6 || f.cov_len == 36) {
        const int st = f.cov_len == 36 ? 7 : 1;
        auto sd_mm = [](float var) { return var > 0 ? (uint32_t)fminf(sqrtf(var) * 1000.0f, 4e9f) : 0u; };
        s.h_acc_mm = sd_mm(fmaxf(f.covariance[0], f.covariance[st])); s.v_acc_mm = sd_mm(f.covariance[2 * st]); s.s_acc_mm = sd_mm(f.covariance[3 * st]);
    }
    if (f.time_standard == 3 && f.gnss_timestamp_us) s.time_ms = (uint32_t)((f.gnss_timestamp_us / 1000ull) % 604800000ull);   // GPS time: of the week
    return s;
}

} // namespace dcan
