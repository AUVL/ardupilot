/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    hal.console->begin(115200);
	hal.uartE->begin(57600);
    hal.console->is_initialized();
	hal.uartE->is_initialized();
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
    mavlink_follow_target_t follow_target_msg;
	follow_target_s follow_target_topic = { };

	mavlink_msg_follow_target_decode(msg, &follow_target_msg);

	follow_target_topic.timestamp = hrt_absolute_time();

	follow_target_topic.lat = follow_target_msg.lat*1e-7;
	follow_target_topic.lon = follow_target_msg.lon*1e-7;
	follow_target_topic.alt = follow_target_msg.alt;

	if (_follow_target_pub == nullptr) {
		_follow_target_pub = orb_advertise(ORB_ID(follow_target), &follow_target_topic);
	} else {
		orb_publish(ORB_ID(follow_target), _follow_target_pub, &follow_target_topic);
	}
}
#endif
