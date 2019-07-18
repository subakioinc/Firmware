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

int binaryToDecimal(int n);

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

				PX4_INFO("Sensor ID: %d", raw.sensor_id);
				PX4_INFO("Sensor State: %d", raw.sensor_state);
				PX4_INFO("Sensor Temperature: %f, Humidity: %f", (double)raw.temp, (double)raw.humid);
				PX4_INFO("Sensor1: ID: %d, State: %d, cdc_low: %d, cdc_high: %d", raw.id[0], raw.state[0],raw.cdc_low[0], raw.cdc_high[0]);
				PX4_INFO("Sensor2: ID: %d, State: %d, cdc_low: %d, cdc_high: %d", raw.id[1], raw.state[1],raw.cdc_low[1], raw.cdc_high[1]);
				PX4_INFO("Sensor3: ID: %d, State: %d, cdc_low: %d, cdc_high: %d", raw.id[2], raw.state[2],raw.cdc_low[2], raw.cdc_high[2]);
				PX4_INFO("Sensor4: ID: %d, State: %d, cdc_low: %d, cdc_high: %d", raw.id[3], raw.state[3],raw.cdc_low[3], raw.cdc_high[3]);

			}
		}
    }

    PX4_INFO("Exiting");

    return 0;
}


