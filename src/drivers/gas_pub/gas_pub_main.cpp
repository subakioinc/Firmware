#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gas.h>

extern "C" __EXPORT int gas_pub_main(int argc, char *argv[]);

int gas_pub_main(int argc, char *argv[])
{
	PX4_INFO("GasSensor publish start!");

	struct sensor_gas_s raw;
	memset(&raw, 0, sizeof(raw));
	orb_advert_t gas_pub = orb_advertise(ORB_ID(sensor_gas), &raw);
	raw.temp_high = 100;
	raw.humid_high = 30;

	for (int i = 0; i < 1000; i++) {
		 orb_publish(ORB_ID(sensor_gas), gas_pub, &raw);
		usleep(500000);
			PX4_INFO("0.5 sec passed!");
	}

	PX4_INFO("exiting");

	return 0;
}
