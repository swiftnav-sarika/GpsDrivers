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
 * @file sbp.h
 *
 * @author Sarika Nagaraj <sarika@swift-nav.com>
 * @author
 */


#include "gps_helper.h"
#include "../../definitions.h"


#define SBP_TIMEOUT_10HZ 200
#define SBP_BAUDRATE 9800

typedef struct {
    bool sys_error_flag; //1
    bool io_error_flag; //1
    bool nap_error_flag; //1
    uint8_t res; //5
    uint8_t protocol_minor; //8
    uint8_t protocol_major; //8
    uint8_t res2; //6
    bool ext_antenna_short; //1
    bool ext_antenna_present; //1
} sbp_heartbeat_packet_t; //4 bytes


class GPSDriverSBP : public GPSHelper
{
public:
    GPSDriverSBP(GPSCallbackPtr callback,
                 void *callback_user,
                 struct vehicle_gps_position_s *gps_position);
    virtual ~GPSDriverSBP() = default;

    int receive(unsigned timeout) override;
    int configure(unsigned &baudrate, OutputMode output_mode) override;

private:
    /**
     * Parse the binary SBP packet
     */
    int parseChar(uint8_t b);

    struct vehicle_gps_position_s *_gps_position {nullptr};


};
