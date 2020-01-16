/**
 * @file simple_cpp_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author jeyong <jeyong@subak.io>
 */

#include <px4_log.h>

extern "C" __EXPORT int simple_cpp_app_main(int argc, char *argv[]);

int simple_cpp_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello World!");
    return OK;
}
