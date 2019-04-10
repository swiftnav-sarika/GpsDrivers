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
        if(_rx_buff_count >= _rx_payload_len){
            _rx_buff_count = 0;
            for(int k=0; k >= _rx_payload_len;k++){
                printf("Buffer %d\n",_rx_buff[k]);
            }
            switch (_rx_msgtype) {
            case SBP_HEARTBEAT_MSGTYPE:
                printf("I'm a heartbeat message!\n");
                // memcpy(sbp_buf_t.sbp_heartbeat, _rx_buff,4);
                //memcpy(&sbp_buf_t.sbp_heartbeat, _rx_buff, sizeof(struct sbp_heartbeat_packet_t));
                break;

            case SBP_GPS_TIME_MSGTYPE:
                break;

            case SBP_POS_LLH_MSGTYPE:
                //memcpy(&sbp_pos_llh_payload, _rx_buff, sizeof(struct sbp_heartbeat_packet_t));
                break;

            case SBP_DOPS_MSGTYPE:
                break;

            case SBP_VEL_NED_MSGTYPE:
                break;

            case SBP_EXT_EVENT_MSGTYPE:
                break;

            default:
                break;
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

        /* Expecting first checksum byte */
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

