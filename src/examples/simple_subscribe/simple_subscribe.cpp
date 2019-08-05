#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

extern "C" __EXPORT int simple_subscribe_main(int argc, char *argv[]);

int simple_subscribe_main(int argc, char *argv[])
{
	PX4_INFO("Start mysub!");

    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(sensor_sub_fd, 200);

	px4_pollfd_struct_t fds{};
	
	fds.fd = sensor_sub_fd;
	fds.events = POLLIN;

	for (int i = 0; i < 5; i++) {
		int poll_ret = px4_poll(&fds, 1, 1000);

		if (poll_ret == 0) {
			PX4_ERR("Got no data within a second");
		} else if (poll_ret < 0) {
			PX4_ERR("ERROR return value from poll()");
		} else {
			if (fds.revents & POLLIN) {
                struct sensor_combined_s raw;
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
                PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                     (double)raw.accelerometer_m_s2[0],
                     (double)raw.accelerometer_m_s2[1],
                     (double)raw.accelerometer_m_s2[2]);
			}
		}	
	}

	PX4_INFO("exiting mysubscribe module");

	return 0;

}
