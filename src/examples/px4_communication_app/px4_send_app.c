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

__EXPORT int px4_send_app_main(int argc, char *argv[]);


int px4_send_app_main(int argc, char *argv[])
{
    PX4_INFO("The sender !");

    struct quad_flot_s att;
    memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(quad_flot), &att);

  
    char* endptr; 
    double coef = 1.0;
    if(argc>=2){
        coef = strtof(argv[1], &endptr);
        if( endptr == argv[1] ){
            coef = 1.0;
        }
    }

    att.rotor_0[0] = coef* -0.495383;
    att.rotor_0[1] = coef* 0.707107;
    att.rotor_0[2] = coef* 0.765306;
    att.rotor_0[3] = coef* 1.000000;

    att.rotor_1[0] = coef* 0.495383;
    att.rotor_1[1] = coef* -0.707107;
    att.rotor_1[2] = coef* 1.000000;
    att.rotor_1[3] = coef* 1.000000;

    att.rotor_2[0] = coef* 0.495383;
    att.rotor_2[1] = coef* 0.707107;
    att.rotor_2[2] = coef* -0.765306;
    att.rotor_2[3] = coef* 1.000000;

    att.rotor_3[0] = coef* -0.495383;
    att.rotor_3[1] = coef* -0.707107;
    att.rotor_3[2] = coef* -1.000000;
    att.rotor_3[3] = coef* 1.000000;

    orb_publish(ORB_ID(quad_flot), att_pub, &att);

    return OK;
}
