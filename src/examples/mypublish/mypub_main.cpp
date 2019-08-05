#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission_subak.h>

extern "C" __EXPORT int mypub_main(int argc, char *argv[]);

int mypub_main(int argc, char *argv[])
{
	PX4_INFO("mypub start!");

	struct mission_subak_s raw;
	memset(&raw, 0, sizeof(raw));
	orb_advert_t subak_pub = orb_advertise(ORB_ID(mission_subak), &raw);

	for (int i = 0; i < 1000; i++) {
		 raw.x++;
		 raw.y++;
		 orb_publish(ORB_ID(mission_subak), subak_pub, &raw);
		usleep(500000);
			PX4_INFO("0.5 sec passed!");
	}

	PX4_INFO("exiting");

	return 0;
}
