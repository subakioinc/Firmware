/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gas.cpp
 * @author Jaeeun Kim <jaeeun@subak.io>
 *
 * Driver for gas sensor.
 * on the serial port you connect the sensor,i.e TELEM2.
 *
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include "gas_parser.h"
#include <systemlib/mavlink_log.h>


//
#define GAS_BASE_DEVICE_PATH	"/dev/gas"
#define GAS0_DEVICE_PATH	"/dev/gas0"


// designated serial port on Pixhawk (TELEM2)
#define GAS_DEFAULT_PORT		"/dev/ttyS2" // Its baudrate is 115200

// normal conversion wait time
#define GAS_CONVERSION_INTERVAL 50*1000UL/* 50ms */

/* Mavlink log uORB handle */
static orb_advert_t mavlink_log_pub = nullptr;


class GAS : public cdev::CDev, public px4::ScheduledWorkItem
{
public:

	// Constructor
	GAS(const char *port = GAS_DEFAULT_PORT);

	// Virtual destructor
	virtual ~GAS() override;

	virtual int  init() override;
	virtual int  ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

private:

	char 				             _port[20];
	int         	             _conversion_interval;
	ringbuffer::RingBuffer	  *_reports;
	int				               _fd;
	uint8_t			             _linebuf[100];
	uint8_t                   _cycle_counter;

	unsigned char             _frame_data[4];
	uint16_t                  _crc16;

	int				               _class_instance;
	int				               _orb_class_instance;

	perf_counter_t			       _sample_perf;
	perf_counter_t			       _comms_errors;

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();


	/**
	* Perform a reading cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				Run() override;

	int				collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gas_main(int argc, char *argv[]);

/**
* Method : Constructor
*
* @note This method initializes the class variables
*/

GAS::GAS(const char *port) :
	CDev(GAS0_DEVICE_PATH),
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_conversion_interval(GAS_CONVERSION_INTERVAL),
	_reports(nullptr),
	_fd(-1),
	_cycle_counter(0),
	_frame_data{0xA5, 0x5A, 0, 0},
	_crc16(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "gas_read")),
	_comms_errors(perf_alloc(PC_COUNT, "gas_com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

// Destructor
GAS::~GAS()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(GAS_BASE_DEVICE_PATH, _class_instance);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

/**
* Method : init()
*
* This method setup the general driver for a range finder sensor.
*/

int
GAS::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(int));

		if (_reports == nullptr) {
			PX4_ERR("alloc failed");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(GAS_BASE_DEVICE_PATH);

		/* Todo :get a publish handle on the gas topic */


	} while (0);

	return ret;
}



int
GAS::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					start();
					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {

					/* convert hz to tick interval via microseconds */
					int interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < _conversion_interval) {
						return -EINVAL;
					}

					start();


					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

int

/*
 * Method: collect()
 *
 * This method reads data from serial UART and places it into a buffer
*/
GAS::collect()
{
	int bytes_read = 0;
	bool crc_valid = false;
	Parser parser = Parser();




	perf_begin(_sample_perf);




	/* read from the sensor (uart buffer) */
	bytes_read = ::read(_fd, &_linebuf[0], sizeof(_linebuf));



	if (bytes_read < 0) {
		PX4_DEBUG("read err: %d \n", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

	} else if (bytes_read > 0) {
		PX4_ERR("gas read! : %d", bytes_read);

		for(int i=0; i<bytes_read; i++){
			printf("%X ", _linebuf[i]);
			parser.Parse(_linebuf[i]);


		}
		PX4_ERR("\n");

	}

	if (!crc_valid) {
		return -EAGAIN;
	}


	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	bytes_read = OK;

	perf_end(_sample_perf);

	return OK;

}

void
GAS::start()
{
	PX4_INFO("driver started");
	PX4_ERR("gas start실행됐음");
	mavlink_log_critical(&mavlink_log_pub, "GAS Detected!");

	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
GAS::stop()
{
	ScheduleClear();
}

void
GAS::Run()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		//NL 문자를 CR-NL 문자로 맵핑하지 않습니다.
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		//stop bit : 1, partiy : none으로
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

		unsigned speed = B115200; // Baud rate

		/* set baud rate */
		//uart_cofig의 터미널 입력 속도를 speed값으로 변경 return 0,-1(fail)
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		//uart_config 터미널의 출력속도를 speed 값으로 변경 return 0,-1(fail)
		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		//fd에 대한 터미널 속성을 바로 변경한다
		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}
	}

	/* perform collection */
	int collect_ret = collect();

	if (collect_ret == -EAGAIN) {
		_cycle_counter++;
	}

	/* schedule a fresh cycle call when a complete packet has been received */
	ScheduleDelayed(_conversion_interval);
	_cycle_counter = 0;
}

void
GAS::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace gas
{

GAS	*g_dev;

int	start(const char *port);
int	stop();
int	test();
int	reset();
int	info();

/**
 * Start the driver.
 */
int
start(const char *port)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	/* create the driver */
	g_dev = new GAS(port);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(GAS0_DEVICE_PATH, 0);

	if (fd < 0) {
		PX4_ERR("device open fail (%i)", errno);
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("failed to set baudrate %d", B115200);
		goto fail;
	}

	PX4_DEBUG("gas::start() succeeded");
	return 0;

fail:
	PX4_DEBUG("gas::start() failed");

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return -1;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		return -1;
	}

	return 0;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	return 0;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = open(GAS0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return -1;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return -1;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return -1;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

} // namespace

int
gas_main(int argc, char *argv[])
{
	const char *device_path = GAS_DEFAULT_PORT;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return -1;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		return gas::start(device_path);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return gas::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return gas::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return gas::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return gas::info();
	}

out_error:
	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return -1;
}
