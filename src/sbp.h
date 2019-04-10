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

#pragma once

#include "gps_helper.h"
#include "edc.h"
#include "../../definitions.h"

#define SBP_TIMEOUT_10HZ    200
#define SBP_BAUDRATE        115200
#define SBP_PREAMBLE        0x55

// Message types supported by this driver
#define SBP_STARTUP_MSGTYPE         0xFF00
#define SBP_HEARTBEAT_MSGTYPE       0xFFFF
#define SBP_GPS_TIME_MSGTYPE        0x0102
#define SBP_DOPS_MSGTYPE            0x0208
#define SBP_POS_LLH_MSGTYPE         0x020A
#define SBP_BASELINE_NED_MSGTYPE    0x020C
#define SBP_VEL_NED_MSGTYPE         0x020E
#define SBP_TRACKING_STATE_MSGTYPE  0x0013
#define SBP_EXT_EVENT_MSGTYPE       0x0101

typedef enum {
    SBP_DECODE_PREAMBLE = 0,
    SBP_DECODE_MESSAGEID,
    SBP_DECODE_SENDER,
    SBP_DECODE_LENGTH,
    SBP_DECODE_PAYLOAD,
    SBP_DECODE_CRC,

} sbp_decode_state_t;


// Swift SBP message definition
#pragma pack(push, 1)
//Heartbeat
typedef struct {
    bool sys_error_flag : 1;        //1
    bool io_error_flag : 1;         //1
    bool nap_error_flag : 1;        //1
    uint8_t reserved : 1;           //5
    uint8_t protocol_minor : 5;     //8
    uint8_t protocol_major : 8;     //8
    uint8_t reserved2 : 6;          //6
    bool ext_antenna_short : 1;     //1
    bool ext_antenna_present : 1;   //1
} sbp_heartbeat_packet_t;       //4 bytes

//GPS Time
typedef struct {
    uint16_t wn;     //< GPS week number (unit: weeks)
    uint32_t tow;    //< GPS Time of Week rounded to the nearest ms (unit: ms)
    int32_t ns;      //< Nanosecond remainder of rounded tow (unit: ns)
    struct flags {
        uint8_t time_src:3;  //< Fix mode (0: invalid, 1: GNSS Solution, 2: Propagated
        uint8_t res:5;       //< Reserved
    } flags;
} sbp_gpstime_packet_t ; // 11 bytes

//Postion LLH
typedef struct {
    uint32_t tow : 4;               //4
    double lat;                 //8
    double lon;                 //8
    double height;              //8
    uint16_t h_accuracy : 8;        //8
    uint16_t v_accuracy : 8;        //8
    uint8_t n_sats : 6;             //6
    uint8_t flags : 1;              //1
} sbp_pos_llh_packet_t;         //34 bytes

//Dilution of Precision
typedef struct {
    uint32_t tow;    //< GPS Time of Week (unit: ms)
    uint16_t gdop;   //< Geometric Dilution of Precision (unit: 0.01)
    uint16_t pdop;   //< Position Dilution of Precision (unit: 0.01)
    uint16_t tdop;   //< Time Dilution of Precision (unit: 0.01)
    uint16_t hdop;   //< Horizontal Dilution of Precision (unit: 0.01)
    uint16_t vdop;   //< Vertical Dilution of Precision (unit: 0.01)
    struct flags {
        uint8_t fix_mode:3;  //< Fix mode (0: invalid, 1: SPP, 2: DGNSS, 3: Float RTX, 4: Fixed RTX, 5: Undefined, 6: SBAS Position
        uint8_t res:4;       //< Reserved
        bool raim_repair:1;  //< Solution from RAIM?
    } flags;
} sbp_dops_packet_t ; // 15 bytes

//Velocity in North East Down(NED) coordinates.
typedef struct {
    uint32_t tow;          //< GPS Time of Week (unit: ms)
    int32_t n;             //< Velocity North coordinate (unit: mm/s)
    int32_t e;             //< Velocity East coordinate  (unit: mm/s)
    int32_t d;             //< Velocity Down coordinate  (unit: mm/s)
    uint16_t h_accuracy;   //< Horizontal velocity accuracy estimate (unit: mm/s)
    uint16_t v_accuracy;   //< Vertical velocity accuracy estimate (unit: mm/s)
    uint8_t n_sats;        //< Number of satellites used in solution
    struct flags {
        uint8_t vel_mode:3;  //< Velocity mode (0: Invalid, 1: Measured Doppler derived, 2: Computed Doppler derived, 3: Dead reckoning)
        uint8_t ins_mode:2;  //< Inertial navigation mode (0: none, 1: INS used)
        uint8_t res:3;       //< Reserved
    } flags;
} sbp_vel_ned_packet_t; // 22 bytes

// Timestamped external events
typedef struct {
    uint16_t wn;           //< GPS week number (unit: weeks)
    uint32_t tow;          //< GPS Time of Week (unit: ms)
    int32_t ns_residual;   //< Nanosecond residual of millisecond-rounded TOW (ranges from -500000 to 500000)
    struct flags {
        uint8_t level:1;       //< New level of pin values (0: Low (falling edge), 1: High (rising edge))
        uint8_t quality:1;     //< Time quality values (0: Unknown - don't have nav solution, 1: Good (< 1 microsecond))
        uint8_t res:6;         //< Reserved
    } flags;
    uint8_t pin;           //< Pin number (0-9)
} sbp_ext_event_packet_t ; // 12 bytes

#pragma pack(pop)


typedef union {
    sbp_heartbeat_packet_t		sbp_heartbeat;
    sbp_pos_llh_packet_t		sbp_pos_llh_payload;
} sbp_buf_t;


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
    int _rx_buff_count;
    uint8_t _rx_buff[256];
    uint16_t _rx_msgtype;
    uint16_t _rx_send_id;
    uint8_t _rx_payload_len;
    uint16_t _rx_crc;
    uint16_t _crc;
    sbp_decode_state_t _decode_state{};

    sbp_buf_t		_buf{};

    struct vehicle_gps_position_s *_gps_position {nullptr};
    /**
     * Parse the binary SBP packet
     */
    int parseChar(const uint8_t b);
    void crc_add(const uint8_t value);

    /**
     * Reset the parse state machine for a fresh start
     */
    void decodeInit(void);

};
