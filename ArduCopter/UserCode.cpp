/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
    int RX;
	int buffer;
    RX = read("/dev/ttyS6",&buffer,10);
	hal.console->println(RX);
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required

    switch (msg->msgid)
	case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
		{
			// decode packet
			mavlink_set_position_target_global_int_t packet;
			mavlink_msg_set_position_target_global_int_decode(msg, &packet);
	
			// exit if vehicle is not in Guided mode or Auto-Guided mode
			if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.auto_mode == Auto_NavGuided)) {
				break;
			}
	
			// check for supported coordinate frames
			if (packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
				packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT && // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
				packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
				packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
				break;
			}
	
			bool pos_ignore 	 = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
			bool vel_ignore 	 = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
			bool acc_ignore 	 = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
	
			/*
			 * for future use:
			 * bool force			= packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
			 * bool yaw_ignore		= packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
			 * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
			 */
	
			Vector3f pos_ned;
	
			if(!pos_ignore) {
				// sanity check location
				if (!check_latlng(packet.lat_int, packet.lon_int)) {
					result = MAV_RESULT_FAILED;
					break;
				}
				Location loc;
				loc.lat = packet.lat_int;
				loc.lng = packet.lon_int;
				loc.alt = packet.alt*100;
				switch (packet.coordinate_frame) {
					case MAV_FRAME_GLOBAL_RELATIVE_ALT: // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
					case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
						loc.flags.relative_alt = true;
						loc.flags.terrain_alt = false;
						break;
					case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
						loc.flags.relative_alt = true;
						loc.flags.terrain_alt = true;
						break;
					case MAV_FRAME_GLOBAL_INT:
					default:
						// Copter does not support navigation to absolute altitudes. This convert the WGS84 altitude
						// to a home-relative altitude before passing it to the navigation controller
						loc.alt -= copter.ahrs.get_home().alt;
						loc.flags.relative_alt = true;
						loc.flags.terrain_alt = false;
						break;
				}
				pos_ned = copter.pv_location_to_vector(loc);
			}
	
			if (!pos_ignore && !vel_ignore && acc_ignore) {
				copter.guided_set_destination_posvel(pos_ned, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
			} else if (pos_ignore && !vel_ignore && acc_ignore) {
				copter.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
			} else if (!pos_ignore && vel_ignore && acc_ignore) {
				if (!copter.guided_set_destination(pos_ned)) {
					result = MAV_RESULT_FAILED;
				}
			} else {
				result = MAV_RESULT_FAILED;
			}
	
			break;
		}
    
}
#endif
