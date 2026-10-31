#!/usr/bin/env python3
"""Reference frames for test/test_mavlink.cpp, made by pymavlink (pip install pymavlink): the C++ encoder (src/telemetry/mavlink.h)
must produce them byte for byte. The values here and the ones the test builds are the same - change both together.
Run once after changing what is covered:  python3 tools/mavlink_vectors.py > test/mavlink_vectors.h"""
import sys
import pymavlink
from pymavlink.dialects.v20 import common as m

sys.stdout.reconfigure(newline="\n")      # (LF also on Windows)
cases = []


def add(name, msg, seq, sysid=1, compid=1):
    mav = m.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    mav.seq = seq
    cases.append((name, seq, bytes(msg.pack(mav))))


add("heartbeat", m.MAVLink_heartbeat_message(type=2, autopilot=0, base_mode=1 | 64 | 128 | 16, custom_mode=2, system_status=4, mavlink_version=3), 0)
add("heartbeat_disarmed_acro", m.MAVLink_heartbeat_message(type=13, autopilot=0, base_mode=1 | 64, custom_mode=1, system_status=3, mavlink_version=3), 255, sysid=7)
add("sys_status", m.MAVLink_sys_status_message(onboard_control_sensors_present=0x0221802B, onboard_control_sensors_enabled=0x0221802B, onboard_control_sensors_health=0x02218023,
                                               load=473, voltage_battery=15872, current_battery=1234, battery_remaining=61, drop_rate_comm=0, errors_comm=3,
                                               errors_count1=0, errors_count2=0, errors_count3=0, errors_count4=0), 17)
add("gps_raw_int", m.MAVLink_gps_raw_int_message(time_usec=123456789000, fix_type=3, lat=473977418, lon=85455939, alt=488123, eph=87, epv=65535, vel=1234, cog=27050,
                                                 satellites_visible=17, alt_ellipsoid=0, h_acc=1250, v_acc=0, vel_acc=0, hdg_acc=0, yaw=0), 18)
add("attitude", m.MAVLink_attitude_message(time_boot_ms=654321, roll=0.125, pitch=-0.25, yaw=2.5, rollspeed=1.5, pitchspeed=-0.75, yawspeed=0.0625), 19)
add("global_position_int", m.MAVLink_global_position_int_message(time_boot_ms=654321, lat=-337000000, lon=-1223950000, alt=12345, relative_alt=-2500, vx=-150, vy=275, vz=-40, hdg=35999), 20)
add("rc_channels", m.MAVLink_rc_channels_message(time_boot_ms=1000, chancount=16, chan1_raw=1500, chan2_raw=1000, chan3_raw=2000, chan4_raw=1234, chan5_raw=1800,
                                                 chan6_raw=1100, chan7_raw=1000, chan8_raw=1000, chan9_raw=1000, chan10_raw=1000, chan11_raw=1000, chan12_raw=1000,
                                                 chan13_raw=1000, chan14_raw=1000, chan15_raw=1000, chan16_raw=1999, chan17_raw=65535, chan18_raw=65535, rssi=203), 21)
add("rc_channels_no_link", m.MAVLink_rc_channels_message(time_boot_ms=0, chancount=0, chan1_raw=0, chan2_raw=0, chan3_raw=0, chan4_raw=0, chan5_raw=0, chan6_raw=0,
                                                         chan7_raw=0, chan8_raw=0, chan9_raw=0, chan10_raw=0, chan11_raw=0, chan12_raw=0, chan13_raw=0, chan14_raw=0,
                                                         chan15_raw=0, chan16_raw=0, chan17_raw=0, chan18_raw=0, rssi=0), 22)     # all zero: truncated to one byte
add("vfr_hud", m.MAVLink_vfr_hud_message(airspeed=3.5, groundspeed=3.5, heading=271, throttle=42, alt=12.25, climb=-0.5), 23)
add("battery_status", m.MAVLink_battery_status_message(id=0, battery_function=0, type=0, temperature=32767, voltages=[3968, 3968, 3968, 3968] + [65535] * 6,
                                                       current_battery=1234, current_consumed=567, energy_consumed=-1, battery_remaining=61, time_remaining=0,
                                                       charge_state=0, voltages_ext=[0, 0, 0, 0], mode=0, fault_bitmask=0), 24)
add("statustext", m.MAVLink_statustext_message(severity=6, text=b"imu: SPI ICM42688P, gyro 4000 Hz", id=0, chunk_seq=0), 25)
long_text = b"motors: FAILED - DShot / ProShot drive at most 4 motors on the ESP32-S3 (4 RMT transmitters)"
add("statustext_chunk0", m.MAVLink_statustext_message(severity=4, text=long_text[:50], id=1, chunk_seq=0), 26)
add("statustext_chunk1", m.MAVLink_statustext_message(severity=4, text=long_text[50:100], id=1, chunk_seq=1), 27)
# what an ExpressLRS receiver in MAVLink mode sends (SerialMavlink.cpp): system 255, component 68 (TELEMETRY_RADIO), to system 1, all components
add("rc_channels_override", m.MAVLink_rc_channels_override_message(target_system=1, target_component=0, chan1_raw=1500, chan2_raw=1250, chan3_raw=988, chan4_raw=1750,
                                                                   chan5_raw=2012, chan6_raw=1000, chan7_raw=1500, chan8_raw=2000, chan9_raw=1100, chan10_raw=1200,
                                                                   chan11_raw=1300, chan12_raw=1400, chan13_raw=1600, chan14_raw=1700, chan15_raw=1800, chan16_raw=1900,
                                                                   chan17_raw=0, chan18_raw=0), 40, sysid=255, compid=68)
add("radio_status", m.MAVLink_radio_status_message(rxerrors=0, fixed=0, rssi=204, remrssi=70, txbuf=87, noise=8, remnoise=0), 41, sysid=255, compid=68)
# 0.40: the ground station's side (system 255, component 190 = MAV_COMP_ID_MISSIONPLANNER) and the craft's answers
from pymavlink.dialects.v20 import development as d
GCS = dict(sysid=255, compid=190)
add("command_long", m.MAVLink_command_long_message(target_system=1, target_component=1, command=176, confirmation=0, param1=1.0, param2=16.0, param3=0.0,
                                                   param4=0.0, param5=0.0, param6=0.0, param7=0.0), 50, **GCS)
add("command_int", m.MAVLink_command_int_message(target_system=1, target_component=0, frame=6, command=179, current=0, autocontinue=0, param1=1.0, param2=0.0,
                                                 param3=0.0, param4=0.0, x=473977418, y=85455939, z=12.5), 51, **GCS)
add("command_ack", m.MAVLink_command_ack_message(command=400, result=1, progress=0, result_param2=0, target_system=255, target_component=190), 52)
add("autopilot_version", m.MAVLink_autopilot_version_message(capabilities=8192 | 4 | 8, flight_sw_version=(0 << 24) | (40 << 16) | (0 << 8) | 255, middleware_sw_version=0,
                                                             os_sw_version=0, board_version=0, flight_custom_version=[0] * 8, middleware_custom_version=[0] * 8,
                                                             os_custom_version=[0] * 8, vendor_id=0, product_id=0, uid=0, uid2=[0] * 18), 53)
add("home_position", m.MAVLink_home_position_message(latitude=473977418, longitude=85455939, altitude=488123, x=0.0, y=0.0, z=0.0, q=[1.0, 0.0, 0.0, 0.0],
                                                     approach_x=0.0, approach_y=0.0, approach_z=0.0, time_usec=0), 54)
add("gps_global_origin", m.MAVLink_gps_global_origin_message(latitude=473977418, longitude=85455939, altitude=488123, time_usec=0), 55)
add("available_modes", d.MAVLink_available_modes_message(number_modes=8, mode_index=4, standard_mode=1, custom_mode=16, properties=0, mode_name=b"Pos Hold"), 56)
add("available_modes_monitor", d.MAVLink_available_modes_monitor_message(seq=1), 57)
add("mission_request_list", m.MAVLink_mission_request_list_message(target_system=1, target_component=1, mission_type=0), 58, **GCS)
add("mission_count", m.MAVLink_mission_count_message(target_system=1, target_component=1, count=3, mission_type=0), 59, **GCS)
add("mission_request_int", m.MAVLink_mission_request_int_message(target_system=255, target_component=190, seq=2, mission_type=0), 60)
add("mission_item_int", m.MAVLink_mission_item_int_message(target_system=1, target_component=1, seq=1, frame=3, command=16, current=0, autocontinue=1,
                                                           param1=5.0, param2=0.0, param3=0.0, param4=float("nan"), x=473980000, y=85460000, z=25.5,
                                                           mission_type=0), 61, **GCS)
add("mission_ack", m.MAVLink_mission_ack_message(target_system=255, target_component=190, type=0, mission_type=0), 62)
add("mission_clear_all", m.MAVLink_mission_clear_all_message(target_system=1, target_component=1, mission_type=255), 63, **GCS)
add("mission_set_current", m.MAVLink_mission_set_current_message(target_system=1, target_component=1, seq=2), 64, **GCS)
add("mission_current", m.MAVLink_mission_current_message(seq=2, total=5, mission_state=3, mission_mode=1), 65)
add("mission_item_reached", m.MAVLink_mission_item_reached_message(seq=1), 66)
add("timesync", m.MAVLink_timesync_message(tc1=0, ts1=1234567890123), 67, **GCS)
# 0.40.1: the float forms Mission Planner falls back to when it has not seen MAV_PROTOCOL_CAPABILITY_MISSION_INT
add("mission_item", m.MAVLink_mission_item_message(target_system=1, target_component=1, seq=1, frame=3, command=16, current=0, autocontinue=1,
                                                   param1=5.0, param2=0.0, param3=0.0, param4=0.0, x=47.398, y=8.546, z=25.5, mission_type=0), 68, **GCS)
add("mission_request", m.MAVLink_mission_request_message(target_system=1, target_component=1, seq=2, mission_type=0), 69, **GCS)

print("// Made by tools/mavlink_vectors.py with pymavlink %s - do not edit." % pymavlink.__version__)
print("#pragma once\n#include <stdint.h>\nstruct MavVector { const char* name; uint8_t seq; int n; uint8_t bytes[280]; };")
print("static const MavVector kMavVectors[] = {")
for name, seq, b in cases:
    print('    {"%s", %d, %d, {%s}},' % (name, seq, len(b), ", ".join("0x%02X" % x for x in b)))
print("};")
