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
 * @file sbp.h
 *
 * @author Sarika Nagaraj <sarika@swift-nav.com>
 * @author
 */

#include "sbp.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <cmath>

GPSDriverSBP::GPSDriverSBP(GPSCallbackPtr callback,
                           void *callback_user,
                           struct vehicle_gps_position_s *gps_position) :
               GPSHelper(callback, callback_user),
               _gps_position(gps_position)
{
   //Nothing
}

int
GPSDriverSBP::configure(unsigned &baudrate, OutputMode output_mode)
{
    if (output_mode != OutputMode::GPS) {
        GPS_WARN("SBP: Unsupported Output Mode %i", (int)output_mode);
        return -1;
    }
   baudrate = 115200;

   return 0;

}

int
GPSDriverSBP::receive(unsigned timeout)
{
    uint8_t buf[GPS_READ_BUFFER_SIZE];

    /* timeout additional to poll */
    gps_abstime time_started = gps_absolute_time();
    int j = 0;
    while (true) {
        int ret = read(buf, sizeof(buf), timeout);
         printf("This is ret %d \n", ret);
         printf("This is timeout %d \n", timeout);

        if (ret > 0) {
            /* first read whatever is left */
            printf("Reading: %d\t Ret: %d\n %", buf[j], ret);

            if (j < ret) {
                /* pass received bytes to the packet decoder */
                while (j < ret) {
                    printf("Reading: %d\t Ret: %d\n %", buf[j], ret);
                    _gps_position->lat = (int32_t)57.378301e7f;
                    _gps_position->lon = (int32_t)108.538777e7f;
                    _gps_position->alt = (int32_t)1200e3f;
                    _gps_position->alt_ellipsoid = 10000;
                    _gps_position->s_variance_m_s = 0.5f;
                    _gps_position->c_variance_rad = 0.1f;
                    _gps_position->fix_type = 3;
                    _gps_position->eph = 0.8f;
                    _gps_position->epv = 1.2f;
                    _gps_position->hdop = 0.9f;
                    _gps_position->vdop = 0.9f;
                    _gps_position->vel_n_m_s = 0.0f;
                    _gps_position->vel_e_m_s = 0.0f;
                    _gps_position->vel_d_m_s = 0.0f;
                    _gps_position->vel_m_s = 0.0f;
                    _gps_position->cog_rad = 0.0f;
                    _gps_position->vel_ned_valid = true;
                    _gps_position->satellites_used = 9;
                    _gps_position->heading = NAN;
                    _gps_position->heading_offset = NAN;
                    j++;
                }
                /* everything is read */
                j = 0;


                /* no time and satellite information simulated */

                printf("Everything is read\n");
                return 1;
            }
        } else {
            gps_usleep(20000);
            return -1;
        }
        /* abort after timeout if no useful packets received */
        if (time_started + timeout * 1000 < gps_absolute_time()) {
            GPS_WARN("timed out, returning");
            return -1;
        }

      }
}
