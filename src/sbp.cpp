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

    if (baudrate > 0 && baudrate != SBP_BAUDRATE) {
        return -1;
    }

    /* set baudrate first */
    if (GPSHelper::setBaudrate(SBP_BAUDRATE) != 0) {
        return -1;
    }

    baudrate = SBP_BAUDRATE;

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

        if (ret > 0) {
            /* first read whatever is left */
            if (j < ret) {
                /* pass received bytes to the packet decoder */
                while (j < ret) {
                    if (parseChar(buf[j]) > 0) {
                        printf("Character Parsed");
                        return 1;
                    }
                    j++;
                }
                /* everything is read */
                j = 0;
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

int
GPSDriverSBP::parseChar(const uint8_t b)
{
    int ret = 0;
    uint16_t crc = 0;
    switch (_decode_state) {

    /* Expecting Preamble */
    case SBP_DECODE_PREAMBLE:
        if (b == 0x55) {
            printf("Preamble Found!!!!\n");
            _rx_buff_count = 0;
            _decode_state = SBP_DECODE_MESSAGEID;
        } else {
            decodeInit();
        }
        break;

        /* Expecting Message ID */
    case SBP_DECODE_MESSAGEID:
        *((uint8_t*)&(_rx_msgtype) + _rx_buff_count) = b;
        _rx_buff_count++;
        if(_rx_buff_count >= 2){
            _rx_buff_count = 0;
            printf("Message ID = %d\n",_rx_msgtype);
            _decode_state = SBP_DECODE_SENDER;
        }
        break;

        /* Expecting Sender */
    case SBP_DECODE_SENDER:
        *((uint8_t*)&(_rx_send_id) + _rx_buff_count) = b;
        _rx_buff_count++;
        if(_rx_buff_count >= 2){
            _rx_buff_count = 0;
            printf("Sender ID = %d\n",_rx_send_id);
            _decode_state = SBP_DECODE_LENGTH;
        }
        break;

        /* Expecting Length */
    case SBP_DECODE_LENGTH:
        _rx_payload_len = b;
        _rx_buff_count = 0;
        printf("Payload Length = %d\n",_rx_payload_len);
        _decode_state = SBP_DECODE_PAYLOAD;
        break;

        /* Expecting payload */
    case SBP_DECODE_PAYLOAD:
        *((uint8_t*)&(_rx_buff) + _rx_buff_count) = b;
        _rx_buff_count++;
        //printf("Payload %d\n",b);
        if(_rx_buff_count >= _rx_payload_len){
            _rx_buff_count = 0;
            for(int k=0; k >= _rx_payload_len;k++){
                printf("Buffer %d\n",_rx_buff[k]);

            }


            //            if (ret < 0) {
            //                // payload not handled, discard message
            //                decodeInit();

            //            } else if (ret > 0) {
            //                // payload complete, expecting checksum
            //                _decode_state = SBP_DECODE_CRC;

            //            } else {
            //                // expecting more payload, stay in state UBX_DECODE_PAYLOAD
            //            }
            _decode_state = SBP_DECODE_CRC;
        }

        break;

        /* Expecting checksum byte */
    case SBP_DECODE_CRC:
        *((uint8_t*)&(_rx_crc) + _rx_buff_count) = b;
        _rx_buff_count++;
        if(_rx_buff_count >= 2) {
            crc = crc16_ccitt((uint8_t*)&(_rx_msgtype), 2, 0);
            crc = crc16_ccitt((uint8_t*)&(_rx_send_id), 2, crc);
            crc = crc16_ccitt(&_rx_payload_len, 1, crc);
            crc = crc16_ccitt(_rx_buff, _rx_payload_len, crc);
            printf("crc = %d\n", crc);
            printf("_rx_crc %d\n",_rx_crc);
            if (_rx_crc == crc){
                printf("Checksum!!\n");
                processPayload();
                decodeInit();
                break;
            } else {
                printf("Checksum failed!\n");
                decodeInit();
                ret = 0;
            }

        }

    default:
        break;
    }
    return ret;
}
void
GPSDriverSBP::processPayload()
{

    switch (_rx_msgtype) {
    case SBP_HEARTBEAT_MSGTYPE:
        printf("I'm a heartbeat message!\n");
        memcpy((uint8_t *)&(_sbp_buf.sbp_heartbeat),(uint8_t*)&(_rx_buff), sizeof(sbp_heartbeat_packet_t));
        printf("sys_error_flag %d\n",_sbp_buf.sbp_heartbeat.sys_error_flag);
        printf("io_error_flag %d\n",_sbp_buf.sbp_heartbeat.io_error_flag);
        printf("nap_error_flag %d\n",_sbp_buf.sbp_heartbeat.nap_error_flag);
        printf("protocol_minor %d\n",_sbp_buf.sbp_heartbeat.protocol_minor);
        printf("protocol_major %d\n",_sbp_buf.sbp_heartbeat.protocol_major);
        printf("ext_antenna_short %d\n",_sbp_buf.sbp_heartbeat.ext_antenna_short);
        printf("ext_antenna_present %d\n",_sbp_buf.sbp_heartbeat.ext_antenna_present);
        break;

    case SBP_GPS_TIME_MSGTYPE:
        printf("I'm a GPS TIME message!\n");
        memcpy((uint8_t *)&(_sbp_buf.sbp_gps_time),(uint8_t*)&(_rx_buff), sizeof(sbp_gpstime_packet_t));
        printf("wn %d\n",_sbp_buf.sbp_gps_time.wn);
        printf("tow %d\n",_sbp_buf.sbp_gps_time.tow);
        printf("ns %d\n",_sbp_buf.sbp_gps_time.ns);
        printf("time_src %d\n",_sbp_buf.sbp_gps_time.flags.time_src);

        break;

    case SBP_POS_LLH_MSGTYPE:
        printf("I'm a POS LLH message!\n");
        memcpy((uint8_t *)&(_sbp_buf.sbp_pos_llh),(uint8_t*)&(_rx_buff), sizeof(sbp_pos_llh_packet_t));
        printf("tow %d\n",_sbp_buf.sbp_pos_llh.tow);
        printf("lat %f\n",_sbp_buf.sbp_pos_llh.lat);
        printf("lon %f\n",_sbp_buf.sbp_pos_llh.lon);
        printf("height %f\n",_sbp_buf.sbp_pos_llh.height);
        printf("h_accuracy %d\n",_sbp_buf.sbp_pos_llh.h_accuracy);
        printf("v_accuracy %d\n",_sbp_buf.sbp_pos_llh.v_accuracy);
        printf("n_sats %d\n",_sbp_buf.sbp_pos_llh.n_sats);
        printf("flags %d\n",_sbp_buf.sbp_pos_llh.flags);
        _gps_position->lat = _sbp_buf.sbp_pos_llh.lat;
        _gps_position->lon = _sbp_buf.sbp_pos_llh.lon;
        _gps_position->alt = _sbp_buf.sbp_pos_llh.height;
        _gps_position->eph = _sbp_buf.sbp_pos_llh.h_accuracy;
        _gps_position->epv = _sbp_buf.sbp_pos_llh.v_accuracy;
        _gps_position->fix_type = _sbp_buf.sbp_pos_llh.flags.fix_mode;
        _gps_position->satellites_used = _sbp_buf.sbp_pos_llh.n_sats;
        break;

    case SBP_DOPS_MSGTYPE:
        printf("I'm a SBP DOPS message!\n");
        memcpy((uint8_t *)&(_sbp_buf.sbp_dops),(uint8_t*)&(_rx_buff), sizeof(sbp_dops_packet_t));

        _gps_position->hdop = _sbp_buf.sbp_dops.hdop;
        _gps_position->vdop = _sbp_buf.sbp_dops.vdop;
        break;

    case SBP_VEL_NED_MSGTYPE:
        printf("I'm a SBP VEL NED message!\n");
        memcpy((uint8_t *)&(_sbp_buf.sbp_vel_ned),(uint8_t*)&(_rx_buff), sizeof(sbp_vel_ned_packet_t));
        break;

    case SBP_EXT_EVENT_MSGTYPE:
        printf("I'm a SBP EXT_EVENT message!\n");
        memcpy((uint8_t *)&(_sbp_buf.sbp_ext_event),(uint8_t*)&(_rx_buff), sizeof(sbp_ext_event_packet_t));
        break;

    default:
        break;
    }

}

void
GPSDriverSBP::decodeInit()
{
    //_rx_buff=0;
    _rx_msgtype=0;
    _rx_send_id=0;
    _rx_payload_len=0;
    _crc = 0;
    _rx_buff_count = 0;
    _decode_state = SBP_DECODE_PREAMBLE;
}

