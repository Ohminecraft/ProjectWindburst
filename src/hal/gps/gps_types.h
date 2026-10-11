/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// GPS data + configuration. Units follow Betaflight (io/gps.h) so values map 1:1 to its telemetry / app fields:
//   lat/lon 1e-7 deg, alt cm (MSL), speeds cm/s, course deci-degrees, DOP x100, accuracies mm.
// ============================================================================
namespace gps {

constexpr int kMaxSats = 40;

enum Provider : uint8_t { PROV_NMEA, PROV_UBLOX, PROV_MSP, PROV_VIRTUAL };       // DroneCAN (BF has it) is not implemented
enum SbasMode : uint8_t { SBAS_AUTO, SBAS_EGNOS, SBAS_WAAS, SBAS_MSAS, SBAS_GAGAN, SBAS_NONE };
enum AcquireModel : uint8_t { MODEL_PORTABLE, MODEL_STATIONARY, MODEL_PEDESTRIAN, MODEL_AUTOMOTIVE, MODEL_AT_SEA, MODEL_AIRBORNE_1G, MODEL_AIRBORNE_2G, MODEL_AIRBORNE_4G };
enum UtcStandard : uint8_t { UTC_AUTO, UTC_USNO, UTC_EU, UTC_SU, UTC_NTSC };

struct Config {
    Provider provider = PROV_UBLOX;
    SbasMode sbas = SBAS_NONE;
    bool sbas_integrity = false;
    bool auto_config = true;                 // BF gps_auto_config
    bool auto_baud = true;                   // BF gps_auto_baud default is OFF but Betaflight 4.5+ u-blox code sweeps baud rates regardless; we keep it switchable
    uint32_t baud = 115200;                  // the rate we want to run at (BF default 57600; 115200 leaves room for 10 Hz NAV-PVT + NAV-SAT)
    uint8_t update_rate_hz = 10;             // BF gps_update_rate_hz
    AcquireModel acquire_model = MODEL_STATIONARY;    // until first 3D fix (BF: gps_ublox_acquire_model)
    AcquireModel flight_model = MODEL_AIRBORNE_4G;    // after the first 3D fix (BF: gps_ublox_flight_model)
    bool use_galileo = false;
    UtcStandard utc_standard = UTC_AUTO;
    bool set_home_point_once = false;        // BF gps_set_home_point_once
    bool use_3d_speed = false;               // BF gps_use_3d_speed
    uint8_t min_sats = 8;                    // for the home fix (BF gpsRescue minSats default 8)
    char nmea_custom_commands[64] = "";      // BF nmeaCustomCommands (sent verbatim, one per line, e.g. to switch a NMEA module's rate)
};

struct DateTime { uint16_t year = 0; uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0; uint16_t millis = 0; bool valid = false; };

struct Solution {
    int32_t lat = 0, lon = 0;                // 1e-7 deg
    int32_t alt_cm = 0;                      // MSL
    uint8_t fix_type = 0;                    // 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
    bool fix = false;                        // valid 3D fix (what BF calls STATE(GPS_FIX))
    uint8_t num_sat = 0;
    uint16_t ground_speed_cms = 0, speed3d_cms = 0;
    uint16_t course_ddeg = 0;                // 0..3599
    int16_t vel_n_cms = 0, vel_e_cms = 0, vel_d_cms = 0;   // NED
    bool has_vel_d = false;                  // vel_d_cms is measured (u-blox NAV-PVT / VELNED); NMEA has no vertical velocity
    uint16_t pdop = 0, hdop = 0, vdop = 0;   // x100
    uint32_t h_acc_mm = 0, v_acc_mm = 0, s_acc_mm = 0, head_acc = 0;
    uint32_t time_ms = 0;                    // GPS time of week
    DateTime date;
};

struct SatInfo { uint8_t gnss_id = 255, sv_id = 0, cno = 0; int8_t elev = 0; int16_t azim = 0; uint32_t flags = 0; };

enum State : uint8_t { ST_UNKNOWN, ST_INITIALIZED, ST_DETECT_BAUD, ST_CHANGE_BAUD, ST_CONFIGURE, ST_RECEIVING_DATA, ST_LOST_COMMUNICATION };
enum UbxVersion : uint8_t { UBX_UNDEF, UBX_M5, UBX_M6, UBX_M7, UBX_M8, UBX_M9, UBX_M10 };

} // namespace gps
