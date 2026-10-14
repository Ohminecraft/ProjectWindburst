// Replays a flight log (the CSV export of a .wblog: the fast file and its _slow file) through the EKF (src/core/ekf.h) and compares
// it with what the firmware itself estimated in that flight.
//
//   build/ekf_replay  flight.csv  flight_slow.csv  [out.csv]  [--gps-delay 0.15] [--declination 0] [--no-mag]
//
// Uses from the fast file: t, gyro (filtered, deg/s, pilot convention: pitch rate negated), acc (g, FRD); from the slow file: armed, baro_alt,
// mag (calibrated, the core's frame), gps_fix, gps_speed, gps_course, lat / lon (only if the CSV kept their resolution: the export
// before 0.9.0 rounded them to ~11 m, then the GPS position is left out), vel_n / vel_e / pos_n / pos_e / est_z (the firmware's own).
// Prints a summary; out.csv gets one row per slow record with both estimates side by side.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>
#include <map>
#include "../src/core/ekf.h"

using namespace fc;

struct Csv {
    std::vector<std::string> head; std::vector<std::vector<double>> rows;
    bool load(const char* path) {
        FILE* f = fopen(path, "r"); if (!f) return false;
        char* line = nullptr; size_t cap = 0; ssize_t n; bool first = true;
        while ((n = getline(&line, &cap, f)) > 0) {
            std::vector<std::string> cells; std::string cur;
            for (ssize_t i = 0; i < n; i++) { const char c = line[i]; if (c == ',' || c == '\n' || c == '\r') { if (c == ',' || !cur.empty() || !cells.empty()) cells.push_back(cur); cur.clear(); if (c != ',') break; } else cur += c; }
            if (first) { head = cells; first = false; continue; }
            std::vector<double> r(head.size(), NAN); for (size_t i = 0; i < cells.size() && i < r.size(); i++) r[i] = cells[i].empty() ? NAN : atof(cells[i].c_str());
            rows.push_back(r);
        }
        free(line); fclose(f); return !head.empty();
    }
    int col(const char* name) const {                      // "gyro[0]" matches "gyro[0] (deg/s)"
        const size_t n = strlen(name);
        for (size_t i = 0; i < head.size(); i++) if (head[i].compare(0, n, name) == 0 && (head[i].size() == n || head[i][n] == ' ')) return (int)i;
        return -1;
    }
};
static float wrapd(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: ekf_replay fast.csv slow.csv [out.csv] [--gps-delay s] [--declination deg] [--no-mag]\n"); return 2; }
    const char* out_path = nullptr; float delay = 0.15f, decl = 0; bool use_mag = true; float acc_noise = -1, gyro_noise = -1, mag_sigma = -1;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--gps-delay") && i + 1 < argc) delay = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--declination") && i + 1 < argc) decl = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--no-mag")) use_mag = false;
        else if (!strcmp(argv[i], "--acc-noise") && i + 1 < argc) acc_noise = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--gyro-noise") && i + 1 < argc) gyro_noise = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--mag-sigma") && i + 1 < argc) mag_sigma = (float)atof(argv[++i]);
        else out_path = argv[i];
    }
    Csv F, S; if (!F.load(argv[1]) || !S.load(argv[2])) { fprintf(stderr, "cannot read the CSV files\n"); return 1; }
    const int ft = F.col("t"), gx = F.col("gyro[0]"), ax = F.col("acc[0]"), fy = F.col("yaw"), fr = F.col("roll"), fp = F.col("pitch");
    const int st = S.col("t"), sarm = S.col("armed"), sbaro = S.col("baro_alt"), sfix = S.col("gps_fix"), sspd = S.col("gps_speed"), scrs = S.col("gps_course");
    const int slat = S.col("lat"), slon = S.col("lon"), smag = S.col("mag[0]"), svn = S.col("vel_n"), sve = S.col("vel_e"), spn = S.col("pos_n"), spe = S.col("pos_e"), sz = S.col("est_z");
    if (ft < 0 || gx < 0 || ax < 0 || st < 0) { fprintf(stderr, "the fast file needs t, gyro[0..2], acc[0..2]\n"); return 1; }
    printf("fast: %zu records, slow: %zu records\n", F.rows.size(), S.rows.size());

    // Do the coordinates have their resolution? (pre-0.9.0 exports: 4 - 5 significant decimals)
    bool pos_usable = false;
    if (slat >= 0 && slon >= 0) { int fine = 0, n = 0; for (auto& r : S.rows) { if (isnan(r[slat]) || r[slat] == 0) continue; n++; const double f = fabs(r[slat] * 1e5 - floor(r[slat] * 1e5 + 0.5)); if (f > 1e-3) fine++; }
        pos_usable = n > 0 && fine > n / 4; }
    printf("GPS position: %s\n", pos_usable ? "used" : "NOT used (the CSV rounded the coordinates; GPS velocity only)");

    Ekf ekf; ekf.cfg.gps_delay_s = delay;
    if (acc_noise > 0) ekf.cfg.accel_noise = acc_noise;
    if (gyro_noise > 0) ekf.cfg.gyro_noise = gyro_noise;
    if (mag_sigma > 0) ekf.cfg.mag_sigma = mag_sigma;
    // initial alignment from the first records: accelerometer tilt, compass heading
    { const auto& r = F.rows[0]; Vec3 a{(float)r[ax], (float)r[ax + 1], (float)r[ax + 2]};
      float yaw0 = 0; bool known = false;
      if (smag >= 0 && use_mag) {
          const auto& q = S.rows[0]; const float mx = (float)q[smag], my = (float)q[smag + 1], mz = (float)q[smag + 2];
          // the field rotated into the earth frame with the tilt and a heading of 0: its angle is (declination - heading)
          const float roll = atan2f(-a.y, -a.z), pitch = atan2f(a.x, sqrtf(a.y * a.y + a.z * a.z)), cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
          const float mn = cp * mx + sr * sp * my + cr * sp * mz, me = cr * my - sr * mz;
          yaw0 = decl * kDeg2Rad - atan2f(me, mn); known = true; }
      ekf.init(a, yaw0, known); }

    FILE* out = out_path ? fopen(out_path, "w") : nullptr;
    if (out) fprintf(out, "t,armed,ekf_roll,ekf_pitch,ekf_yaw,log_roll,log_pitch,log_yaw,ekf_vn,ekf_ve,ekf_vd,log_vn,log_ve,gps_vn,gps_ve,ekf_pn,ekf_pe,log_pn,log_pe,ekf_z,log_z,baro,yaw_sigma,test_vel,test_mag,mag_rejects,gps_rejects\n");
    size_t si = 0; double last_gps[4] = {NAN, NAN, NAN, NAN}; double lat0 = NAN, lon0 = NAN;
    double prev_t = F.rows[0][ft];
    // statistics
    double yaw_d2 = 0, roll_d2 = 0, pitch_d2 = 0; int nd = 0; double vres_ekf = 0, vres_log = 0; int nv = 0; double max_ekf_z = 0, max_log_z = 0;
    float log_vn = 0, log_ve = 0, log_pn = 0, log_pe = 0, log_z = 0; float gps_vn = 0, gps_ve = 0; bool armed = false;
    for (const auto& r : F.rows) {
        const double t = r[ft]; const float dt = (float)(t - prev_t); prev_t = t;
        // (the log's gyro is in the pilot convention like its angles - pitch rate negated -, its accelerometer in FRD)
        const Vec3 g{(float)(r[gx] * kDeg2Rad), (float)(-r[gx + 1] * kDeg2Rad), (float)(r[gx + 2] * kDeg2Rad)};
        const Vec3 a{(float)(r[ax] * kG0), (float)(r[ax + 1] * kG0), (float)(r[ax + 2] * kG0)};
        if (dt > 0 && dt < 0.1f) ekf.predict({g.x * dt, g.y * dt, g.z * dt}, {a.x * dt, a.y * dt, a.z * dt}, dt);
        if (!armed && fabsf(g.x) + fabsf(g.y) + fabsf(g.z) < 0.1f) ekf.fuse_still();
        while (si < S.rows.size() && S.rows[si][st] <= t) {
            const auto& q = S.rows[si++];
            armed = sarm >= 0 && q[sarm] > 0.5;
            if (sbaro >= 0 && !isnan(q[sbaro])) ekf.fuse_baro((float)q[sbaro]);
            if (smag >= 0 && use_mag) ekf.fuse_mag({(float)q[smag], (float)q[smag + 1], (float)q[smag + 2]}, decl);
            const float prev_log_vn = log_vn, prev_log_ve = log_ve;           // (the firmware's estimate BEFORE this record: before it fused the fix)
            if (svn >= 0) { log_vn = (float)q[svn]; log_ve = (float)q[sve]; }
            if (spn >= 0) { log_pn = (float)q[spn]; log_pe = (float)q[spe]; }
            if (sz >= 0) log_z = (float)q[sz];
            const bool fix = sfix >= 0 && q[sfix] >= 3;
            if (fix && sspd >= 0 && scrs >= 0) {
                const double cur[4] = {q[sspd], q[scrs], slat >= 0 ? q[slat] : 0, slon >= 0 ? q[slon] : 0};
                bool fresh = false; for (int k = 0; k < 4; k++) if (cur[k] != last_gps[k]) fresh = true;
                if (fresh) {
                    for (int k = 0; k < 4; k++) last_gps[k] = cur[k];
                    const float spd = (float)q[sspd], crs = (float)(q[scrs] * kDeg2Rad);
                    gps_vn = spd * cosf(crs); gps_ve = spd * sinf(crs);
                    // residual of each estimate against this raw GPS velocity (both estimates at the moment it arrives)
                    if (armed) { vres_ekf += hypot(ekf.vel()[0] - gps_vn, ekf.vel()[1] - gps_ve); vres_log += hypot(prev_log_vn - gps_vn, prev_log_ve - gps_ve); nv++; }
                    if (pos_usable) {
                        if (isnan(lat0)) { lat0 = q[slat]; lon0 = q[slon]; }
                        const float pn = (float)((q[slat] - lat0) * 111319.5), pe = (float)((q[slon] - lon0) * 111319.5 * cos(lat0 * M_PI / 180.0));
                        ekf.fuse_gps(pn, pe, gps_vn, gps_ve, 0, false, 2.0f, 0.3f);
                    } else ekf.fuse_gps_vel(gps_vn, gps_ve, 0, false, 0.3f);
                }
            }
            float er, ep, ey; ekf.euler_rad(er, ep, ey);
            const float lr = fr >= 0 ? (float)r[fr] : 0, lp = fp >= 0 ? (float)r[fp] : 0, ly = fy >= 0 ? (float)r[fy] : 0;
            // (the log's pitch is in the pilot convention, nose up negative: the aero pitch is its negative)
            if (armed) { yaw_d2 += pow(wrapd(ey * kRad2Deg - ly), 2); roll_d2 += pow(er * kRad2Deg - lr, 2); pitch_d2 += pow(ep * kRad2Deg + lp, 2); nd++;
                         max_ekf_z = fmax(max_ekf_z, fabs(ekf.pos()[2])); max_log_z = fmax(max_log_z, fabs(log_z)); }
            const EkfStatus& stt = ekf.status();
            if (out) fprintf(out, "%.3f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u\n",
                             q[st], armed ? 1 : 0, er * kRad2Deg, ep * kRad2Deg, ey * kRad2Deg, lr, -lp, ly, ekf.vel()[0], ekf.vel()[1], ekf.vel()[2], log_vn, log_ve, gps_vn, gps_ve,
                             ekf.pos()[0], ekf.pos()[1], log_pn, log_pe, -ekf.pos()[2], log_z, sbaro >= 0 ? q[sbaro] : 0.0, ekf.yaw_sigma() * kRad2Deg, stt.test_vel, stt.test_mag, (unsigned)stt.mag_rejects, (unsigned)stt.gps_rejects);
        }
    }
    const EkfStatus& stt = ekf.status();
    printf("armed: EKF vs the firmware's attitude: roll %.2f deg RMS, pitch %.2f, heading %.2f\n", sqrt(roll_d2 / fmax(nd, 1)), sqrt(pitch_d2 / fmax(nd, 1)), sqrt(yaw_d2 / fmax(nd, 1)));
    printf("armed: velocity minus the raw GPS velocity just before each fix is used: EKF %.2f m/s, firmware %.2f m/s (%d fixes)\n", vres_ekf / fmax(nv, 1), vres_log / fmax(nv, 1), nv);
    printf("largest |height|: EKF %.1f m, firmware %.1f m\n", max_ekf_z, max_log_z);
    printf("EKF: gyro bias %.2f %.2f %.2f deg/s, accel bias %.2f %.2f %.2f m/s^2, compass readings refused %u, GPS refused %u, resets %u\n",
           ekf.gyro_bias()[0] * kRad2Deg, ekf.gyro_bias()[1] * kRad2Deg, ekf.gyro_bias()[2] * kRad2Deg, ekf.accel_bias()[0], ekf.accel_bias()[1], ekf.accel_bias()[2],
           (unsigned)stt.mag_rejects, (unsigned)stt.gps_rejects, (unsigned)stt.resets);
    if (out) fclose(out);
    return 0;
}
