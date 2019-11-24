#pragma once
#include <stdint.h>

const struct tPacketProp{
	uint8_t pad_len = 3;
	uint8_t hdr_len = 2;
	uint8_t crc_len = 2;
	uint8_t pad_data[3] = { 0xFF, 0x00, 0xAA };
	uint8_t pad_EOH = 0xFF;
	uint8_t pad_EOP = 0xFF;
}PacketProp;

enum class msg_type {optitrack, position, attitude, control_system, controller, float_msg, switcher, vector3D_msg, external_reference, ack,internal_msg_start,internal_ros };
enum class msg_type_optitrack {position, attitude};
enum class control_system {roll, pitch, yaw, x, y, z};
enum class block_type {controller, provider, reference};
enum class switcher_type {controller, provider, reference};
enum class controller_type {pid, mrft};
enum class reference_type {process_variable_ref, restricted_process_variable_ref};
enum class internal_switcher_type {position_provider, attitude_provider, reference};
enum class controller_msg_type {data, change_settings, command};
enum class control_system_msg_type {switch_in_out, add_block, change_PID_settings};
enum class ack_msg_type { raw_packet, payload, nozzle, obstacle, mission,ack,internal_msg_start,internal_ros };
enum class ros_msg_type {ros_obstacle_distance,ros_aircraft_attitude};

