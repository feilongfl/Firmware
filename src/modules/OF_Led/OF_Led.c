/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file OF_Led_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <poll.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/output_pwm.h>

static bool thread_should_exit = false; /**< daemon exit flag */
static bool thread_running     = false; /**< daemon status flag */
static int  daemon_task; /**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int OF_Led_main(int argc, char* argv[]);

/**
 * Mainloop of daemon.
 */
int OF_Led_thread_main(int argc, char* argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char* reason);

static void
usage(const char* reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int OF_Led_main(int argc, char* argv[])
{

    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("daemon already running\n");
            /* this is not an error */
            return 0;
        }

        thread_should_exit = false;
        daemon_task        = px4_task_spawn_cmd("daemon",
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT,
            2000,
            OF_Led_thread_main,
            (argv) ? (char* const*)&argv[2] : (char* const*)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\trunning\n");

        } else {
            warnx("\tnot started\n");
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}

int OF_Led_thread_main(int argc, char* argv[])
{

    warnx("[daemon] starting\n");

    thread_running = true;

    /********/
    //running

    int of_sub_fd = orb_subscribe(ORB_ID(optical_flow));

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        {.fd = of_sub_fd, .events = POLLIN },
    };
    int error_counter = 0;
    //pre pwm
    // const char* dev  = PWM_OUTPUT0_DEVICE_PATH;
    struct output_pwm_s pwm;
    memset(&pwm, 0, sizeof(pwm));
    orb_advert_t pwm_pub = orb_advertise(ORB_ID(output_pwm), &pwm);
    /********/

    while (!thread_should_exit) {
        //warnx("Hello daemon!\n");

        /***************************/
        //loop
        //of read
        int poll_ret = px4_poll(fds, 1, 1000);

        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("[OF_Led] Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("[OF_Led] ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct optical_flow_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(optical_flow), of_sub_fd, &raw);
                // printf("[OF_Led] qulity:\t%d\n",raw.quality);

                pwm.values[5]     = 1200;
                pwm.channel_count = 5;
                orb_publish(ORB_ID(output_pwm), pwm_pub, &pwm);
            }
        }

        /*****************************/
        //pwm out

        /***************************/
        sleep(0.1);
    }

    /***************************/
    //exit

    /***************************/

    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}
