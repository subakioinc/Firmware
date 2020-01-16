/**
 * @file simple_c_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author jeyong <jeyong@subak.io>
 */

#include <px4_log.h>

__EXPORT int simple_c_app_main(int argc, char *argv[]);

int simple_c_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello World!");
    return OK;
}
