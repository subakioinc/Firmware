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

extern "C" __EXPORT int gas_sub_main(int argc, char *argv[]);

int gas_sub_main(int argc, char *argv[])
{
    PX4_INFO("Start GAS's sub!");

    int gassub_fd = orb_subscribe(ORB_ID(sensor_gas));

    px4_pollfd_struct_t fds{};
	fds.fd = gassub_fd;
	fds.events = POLLIN;

    for (int i = 0; i < 1000; i++) {
		int poll_ret = px4_poll(&fds, 1, 1000);

		if (poll_ret == 0) {
			PX4_ERR("Got no data within a second");
		} else if (poll_ret < 0) {
			PX4_ERR("ERROR return value from poll()");
		} else {
			if (fds.revents & POLLIN) {
				struct sensor_gas_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_gas), gassub_fd, &raw);

				PX4_INFO("Get gas info : %d, %d", raw.temp_high, raw.humid_high);

			}
		}
    }

    PX4_INFO("Exiting");

    return 0;
}
