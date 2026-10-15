#!/usr/bin/env python3
"""Reference frames for test/test_can.cpp, made by pydronecan (pip install dronecan): the C++ encoder and decoder must agree
with them bit for bit. Run once after changing what is covered:  python3 tools/dronecan_vectors.py > test/dronecan_vectors.h"""
import dronecan
from dronecan.transport import Transfer

U = dronecan.uavcan
TS = dronecan.TYPENAMES["uavcan.Timestamp"]
cases = []

def add(name, payload, src, tid, prio, dst=None, request=False, service=False, discriminator=None):
    t = Transfer(transfer_id=tid, source_node_id=src, dest_node_id=dst, payload=payload, transfer_priority=prio,
                 request_not_response=request, service_not_message=service, discriminator=discriminator)
    cases.append((name, t.to_frames()))

add("node_status", U.protocol.NodeStatus(uptime_sec=123456, health=1, mode=2, sub_mode=5, vendor_specific_status_code=0xBEEF), 10, 7, 16)
fix = U.equipment.gnss.Fix2(timestamp=TS(usec=1234567890123), gnss_timestamp=TS(usec=987654321), gnss_time_standard=3, num_leap_seconds=18,
                             longitude_deg_1e8=854559380, latitude_deg_1e8=4739774190, height_ellipsoid_mm=488123, height_msl_mm=440555,
                             ned_velocity=[1.25, -0.5, 0.125], sats_used=17, status=3, mode=1, sub_mode=1, covariance=[0.25, 0.25, 0.5, 0.01, 0.01, 0.02], pdop=1.5)
add("fix2", fix, 125, 21, 16)
fix2 = U.equipment.gnss.Fix2(longitude_deg_1e8=-1223950000 * 10, latitude_deg_1e8=-337000000 * 10, height_ellipsoid_mm=-50000, height_msl_mm=-12345,
                              ned_velocity=[-3.0, 2.0, -1.0], sats_used=6, status=2, covariance=[], pdop=3.25)
add("fix2_south_west", fix2, 42, 31, 20)
add("gnss_aux", U.equipment.gnss.Auxiliary(gdop=1.5, pdop=1.25, hdop=0.75, vdop=1.0, tdop=0.5, ndop=0.5, edop=0.5, sats_visible=24, sats_used=17), 125, 3, 20)
add("mag2", U.equipment.ahrs.MagneticFieldStrength2(sensor_id=1, magnetic_field_ga=[0.3, -0.05, 0.38]), 125, 9, 20)
alloc = U.protocol.dynamic_node_id.Allocation(node_id=0, first_part_of_unique_id=True, unique_id=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66])
add("alloc_request", alloc, 0, 0, 30, discriminator=0x1A5)
alloc_resp = U.protocol.dynamic_node_id.Allocation(node_id=125, first_part_of_unique_id=False, unique_id=list(range(1, 17)))
add("alloc_response", alloc_resp, 10, 4, 30)
info = U.protocol.GetNodeInfo.Response(status=U.protocol.NodeStatus(uptime_sec=77, health=0, mode=0),
                                        software_version=U.protocol.SoftwareVersion(major=0, minor=15, optional_field_flags=0, vcs_commit=0, image_crc=0),
                                        hardware_version=U.protocol.HardwareVersion(major=1, minor=2, unique_id=list(range(16)), certificate_of_authenticity=[]),
                                        name="org.windburst.fc")
add("node_info_response", info, 10, 5, 30, dst=125, request=False, service=True)
add("node_info_request", U.protocol.GetNodeInfo.Request(), 10, 6, 30, dst=125, request=True, service=True)

add("pressure", U.equipment.air_data.StaticPressure(static_pressure=97123.5, static_pressure_variance=4.0), 77, 2, 16)
add("temperature", U.equipment.air_data.StaticTemperature(static_temperature=293.5, static_temperature_variance=1.0), 77, 3, 20)
add("battery", U.equipment.power.BatteryInfo(temperature=300.0, voltage=15.75, current=12.5, average_power_10sec=190.0, remaining_capacity_wh=20.0, full_charge_capacity_wh=33.0,
    hours_to_full_charge=0, status_flags=0x41, state_of_health_pct=97, state_of_charge_pct=61, state_of_charge_pct_stdev=2, battery_id=0, model_instance_id=12345, model_name="4S 2200"), 78, 9, 20)
add("raw_command", U.equipment.esc.RawCommand(cmd=[0, 1000, -8191, 8191]), 10, 17, 4)
add("esc_status", U.equipment.esc.Status(error_count=3, voltage=15.5, current=4.25, temperature=310.0, rpm=-21000, power_rating_pct=40, esc_index=2), 61, 11, 16)

print("// Made by tools/dronecan_vectors.py with pydronecan %s - do not edit." % getattr(dronecan, "__version__", "?"))
print("#pragma once\n#include <stdint.h>\nstruct DcVector { const char* name; int n; uint32_t id[24]; uint8_t len[24]; uint8_t data[24][8]; };")
print("static const DcVector kDcVectors[] = {")
for name, frames in cases:
    assert len(frames) <= 24
    ids = ", ".join("0x%08X" % f.message_id for f in frames)
    lens = ", ".join(str(len(f.bytes)) for f in frames)
    data = ", ".join("{" + ", ".join("0x%02X" % b for b in f.bytes) + "}" for f in frames)
    print('    {"%s", %d, {%s}, {%s}, {%s}},' % (name, len(frames), ids, lens, data))
print("};")
