/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_log.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/quad_flot.h>

__EXPORT int px4_recpt_app_main(int argc, char *argv[]);

bool thread_running = false;     /**< Deamon status flag */

int px4_recpt_app_main(int argc, char *argv[])
{
    PX4_INFO("The receiver !");
	thread_running = true;

    int quad_flot_df = orb_subscribe(ORB_ID(quad_flot));

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = quad_flot_df,   .events = POLLIN },
    };


    while (true) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);
 
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			// PX4_ERR("Got no data within a second");
            continue;
		} else if (poll_ret < 0) {
            PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            continue;
        }
       
        if (fds[0].revents & POLLIN) {
            /* obtained data for the first file descriptor */
            struct quad_flot_s raw;
            /* copy sensors raw data into local buffer */
            orb_copy(ORB_ID(quad_flot), quad_flot_df, &raw);
            PX4_INFO(
                "Essai:\t%8.4f\t%8.4f\t%8.4f",
                (double)raw.rotor_0[0],
                (double)raw.rotor_0[1],
                (double)raw.rotor_0[2],
                (double)raw.rotor_0[3]
            );
        }
    }

	thread_running = false;
    return OK;
}
