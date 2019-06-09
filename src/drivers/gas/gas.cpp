/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file gps.cpp
 * Driver for the gas on a serial port
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif


#include <termios.h>

#ifndef __PX4_QURT
#include <poll.h>
#endif


#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_cli.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <uORB/uORB.h>

#include <uORB/topics/gas_dump.h>

#include <board_config.h>

#ifdef __PX4_LINUX
#include <linux/spi/spidev.h>
#endif /* __PX4_LINUX */

#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000


class GAS : public ModuleBase<GAS>
{
public:

	GAS(const char *path, unsigned configured_baudrate);
	virtual ~GAS();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** spawn task */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static GAS *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);


	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	int print_status() override;

	/**
	 * Schedule reset of the GAS device
	 */
	void schedule_reset(GASRestartType restart_type);

	/**
	 * Reset device if reset was scheduled
	 */
	void reset_if_scheduled();

private:
	int				_serial_fd{-1};					///< serial interface to GAS
	unsigned			_baudrate{0};					///< current baudrate
	const unsigned			_configured_baudrate{0};			///< configured baudrate (0=auto-detect)
	char				_port[20] {};					///< device / serial port path
	/**
	 * Publish the gps struct
	 */
	void 				publish();

	/**
	 * This is an abstraction for the poll on serial used.
	 *
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout in ms
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int pollOrRead(uint8_t *buf, size_t buf_length, int timeout);

	/**
	 * set the Baudrate
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(unsigned baud);
};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);


GAS::GAS(const char *path, unsigned configured_baudrate) :
	_configured_baudrate(configured_baudrate)
{
	/* store port name */
	strncpy(_port, path, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

GAS::~GAS()
{
	// wait for it to exit
	unsigned int i = 0;

	do {
		px4_usleep(20000); // 20 ms
		++i;
	} while (i < 100);
}

int GAS::callback(GASCallbackType type, void *data1, int data2, void *user)
{
	GAS *gps = (GAS *)user;

	switch (type) {
	case GASCallbackType::readDeviceData: {
			int num_read = gps->pollOrRead((uint8_t *)data1, data2, *((int *)data1));

			if (num_read > 0) {
				gps->dumpGpsData((uint8_t *)data1, (size_t)num_read, false);
			}

			return num_read;
		}

	case GASCallbackType::writeDeviceData:
		gps->dumpGpsData((uint8_t *)data1, (size_t)data2, true);

		return write(gps->_serial_fd, data1, (size_t)data2);

	case GASCallbackType::setBaudrate:
		return gps->setBaudrate(data2);

	case GASCallbackType::gotRTCMMessage:
		/* not used */
		break;

	case GASCallbackType::surveyInStatus:
		/* not used */
		break;

	case GASCallbackType::setClock:
		px4_clock_settime(CLOCK_REALTIME, (timespec *)data1);
		break;
	}

	return 0;
}

int GAS::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
#if !defined(__PX4_QURT)

	/* For non QURT, use the usual polling. */

	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	//FIXME: add a unified poll() API
	const int max_timeout = 50;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

	if (ret > 0) {
		/* if we have new data from GAS, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If we have all requested data available, read it without waiting.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			const unsigned character_count = 32; // minimum bytes that we want to read
			unsigned baudrate = _baudrate == 0 ? 115200 : _baudrate;
			const unsigned sleeptime = character_count * 1000000 / (baudrate / 10);

#ifdef __PX4_NUTTX
			int err = 0;
			int bytes_available = 0;
			err = ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

			if (err != 0 || bytes_available < (int)character_count) {
				px4_usleep(sleeptime);
			}

#else
			px4_usleep(sleeptime);
#endif

			ret = ::read(_serial_fd, buf, buf_length);

		} else {
			ret = -1;
		}
	}

	return ret;

#else
	/* For QURT, just use read for now, since this doesn't block, we need to slow it down
	 * just a bit. */
	px4_usleep(10000);
	return ::read(_serial_fd, buf, buf_length);
#endif
}

int GAS::setBaudrate(unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		GAS_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		GAS_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		GAS_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

	return 0;
}


void
GAS::run()
{
	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (_serial_fd < 0) {
		PX4_ERR("GAS: failed to open serial port: %s err: %d", _port, errno);
		return;
	}

	initializeCommunicationDump();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!should_exit()) {
		{

			if (_helper != nullptr) {
				delete (_helper);
				_helper = nullptr;
			}


			_baudrate = _configured_baudrate;

			if (_helper && _helper->configure(_baudrate, GASHelper::OutputMode::GAS) == 0) {

				/* reset report */
				memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));
				_report_gps_pos.heading = NAN;
				_report_gps_pos.heading_offset = heading_offset;

				if (_mode == GAS_DRIVER_MODE_UBX) {

					/* GAS is obviously detected successfully, reset statistics */
					_helper->resetUpdateRates();
				}

				int helper_ret;

				while ((helper_ret = _helper->receive(TIMEOUT_5HZ)) > 0 && !should_exit()) {

					if (helper_ret & 1) {
						publish();

						last_rate_count++;
					}

					reset_if_scheduled();

					/* measure update rate every 5 seconds */
					if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
						float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
						_rate = last_rate_count / dt;
						_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
						last_rate_measurement = hrt_absolute_time();
						last_rate_count = 0;
						_last_rate_rtcm_injection_count = 0;
						_helper->storeUpdateRates();
						_helper->resetUpdateRates();
					}

					if (!_healthy) {
						_healthy = true;
					}
				}

				if (_healthy) {
					_healthy = false;
					_rate = 0.0f;
					_rate_rtcm_injection = 0.0f;
				}
			}
		}
	}

	PX4_INFO("exiting");

	if (_serial_fd >= 0) {
		::close(_serial_fd);
		_serial_fd = -1;
	}

}

int
GAS::print_status()
{
	PX4_INFO("Main GAS");

	return 0;
}

void
GAS::schedule_reset(GASRestartType restart_type)
{
	_scheduled_reset = restart_type;
}

void
GAS::reset_if_scheduled()
{
	GASRestartType restart_type = _scheduled_reset;

	if (restart_type != GASRestartType::None) {
		_scheduled_reset = GASRestartType::None;
		int res = _helper->reset(restart_type);

		if (res == -1) {
			PX4_INFO("Reset is not supported on this device.");

		} else if (res < 0) {
			PX4_INFO("Reset failed.");

		} else {
			PX4_INFO("Reset succeeded.");
		}
	}
}

void
GAS::publish()
{

}


int
GAS::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	GAS *_instance = get_instance();

	bool res = false;

	if (argc == 2 && !strcmp(argv[0], "reset")) {

		if (!strcmp(argv[1], "hot")) {
			res = true;
			_instance->schedule_reset(GASRestartType::Hot);

		} else if (!strcmp(argv[1], "cold")) {
			res = true;
			_instance->schedule_reset(GASRestartType::Cold);

		} else if (!strcmp(argv[1], "warm")) {
			res = true;
			_instance->schedule_reset(GASRestartType::Warm);
		}
	}

	if (res) {
		PX4_INFO("Resetting GAS - %s", argv[1]);
		return 0;
	}

	return (res) ? 0 : print_usage("unknown command");
}

int GAS::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GAS driver module that handles the communication with the device and publishes the position via uORB.
It supports multiple protocols (device vendors) and by default automatically selects the correct one.

### Implementation
There is a thread for each device polling for data. The GAS protocol classes are implemented with callbacks
so that they can be used in other projects as well (eg. QGroundControl uses them too).

### Examples
For testing it can be useful to fake a GAS signal (it will signal the system that it has a valid position):
$ gas stop
$ gas start -f

Starting 2 GAS devices (the main GAS on /dev/ttyS3 and the secondary on /dev/ttyS4):
$ gas start -d /dev/ttyS3 -e /dev/ttyS4

Initiate warm restart of GAS device
$ gas reset warm
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gas", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	//ToDo : tel2 포트 사용을 위해 path 확인하기 : /dev/ttyS3 -> ??
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "GAS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('e', nullptr, "<file:dev>", "Optional secondary GAS device", true);

//	PRINT_MODULE_USAGE_PARAM_STRING('i', "uart", "spi|uart", "GAS interface", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset GAS device");
	PRINT_MODULE_USAGE_ARG("cold|warm|hot", "Specify reset type", false);

	return 0;
}

int GAS::task_spawn(int argc, char *argv[])
{
	return task_spawn(argc, argv, Instance::Main);
}

int GAS::task_spawn(int argc, char *argv[], Instance instance)
{
	px4_main_t entry_point;
	if (instance == Instance::Main) {
		entry_point = (px4_main_t)&run_trampoline;
	}

	int task_id = px4_task_spawn_cmd("gps", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, 1600,
				   entry_point, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		return -errno;
	}

	if (instance == Instance::Main) {
		_task_id = task_id;
	}

	return 0;
}

GAS *GAS::instantiate(int argc, char *argv[])
{
	return instantiate(argc, argv, Instance::Main);
}

GAS *GAS::instantiate(int argc, char *argv[], Instance instance)
{
	const char *device_name = "/dev/ttyS3";
	int baudrate_main = 0;
	int baudrate_secondary = 0;
	bool fake_gps = false;
	bool enable_sat_info = false;
	GASHelper::Interface interface = GASHelper::Interface::UART;
	gps_driver_mode_t mode = GAS_DRIVER_MODE_NONE;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:e:fg:si:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate_main) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;
		case 'g':
			if (px4_get_parameter_value(myoptarg, baudrate_secondary) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;

		case 'd':
			device_name = myoptarg;
			break;

		case 'i':
			// uart only
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	GAS *gas;
	gas = new GAS(device_name, mode, instance, baudrate_main);

	return gps;
}

int
gas_main(int argc, char *argv[])
{
	return GAS::main(argc, argv);
}
