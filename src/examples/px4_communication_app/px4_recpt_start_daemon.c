/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file publisher_start_nuttx.cpp
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//#include <systemlib/err.h>

extern bool thread_running;
int daemon_task;             /**< Handle of deamon task / thread */

static bool task_should_exit = false;

extern int px4_recpt_app_main(int argc, char *argv[]);

__EXPORT int px4_recpt_start_daemon_main(int argc, char *argv[]);

int px4_recpt_start_daemon_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: px4_recpt_start_daemon {start|stop|status}");
	    return ERROR;
    }
    
	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
		    PX4_INFO("already running");
	        return ERROR;
		}

		task_should_exit = false;

		daemon_task = px4_task_spawn_cmd(
            "px4_recpt_app",
             SCHED_DEFAULT,
             SCHED_PRIORITY_MAX - 5,
             2000,
             px4_recpt_app_main,
             (char * const *)&argv[0]
        );
						 // (argv) ? (char *const *)&argv[2] : (char *const *)0);

		return OK;
	}

	if (!strcmp(argv[1], "stop")) {
		task_should_exit = true;
		return OK;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
		    PX4_INFO("is running");

		} else {
		    PX4_INFO("not started");
		}

		return OK;
	}

    PX4_INFO("unrecognized command");
	return 1;
}
