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

    /* Wait for 50ms before receiving */
    decodeInit();
    receive(50);
    decodeInit();

    return 0;
}

int
GPSDriverSBP::receive(unsigned timeout)
{
    uint8_t buf[GPS_READ_BUFFER_SIZE];

    /* timeout additional to poll */
    gps_abstime time_started = gps_absolute_time();
    printf("gps_absolute_time() = %d\n", gps_absolute_time());
    printf("timeout = %f\n", timeout);
    printf("time_started = %f\n", time_started);
    int j = 0;
    while (true) {
        int ret = read(buf, sizeof(buf), timeout);
        printf("timeout = %f\n", timeout);
        printf("time_started = %f\n", time_started);

        if (ret > 0) {
            if (j < ret) {
                /* pass received bytes to the packet decoder */
                while (j < ret) {
                    if (parseChar(buf[j]) > 0) {
                        return 1;
                    }
                    j++;
                }
                /* everything is read */
                j = 0;
                //printf("Everything is read\n");
                return 1;
            }
        } else {
            gps_usleep(20000);
            //printf("I'm in else\n");
        }
        /* abort after timeout if no useful packets received */
        printf("gps_absolute_time() = %d\n", gps_absolute_time());
        printf("timeout = %f\n", timeout);
        printf("time_started = %f\n", time_started);
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
            //        } else {
            //            decodeInit();
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
            //printf("Sender ID = %d\n",_rx_send_id);
            _decode_state = SBP_DECODE_LENGTH;
        }
        break;

        /* Expecting Length */
    case SBP_DECODE_LENGTH:
        _rx_payload_len = b;
        _rx_buff_count = 0;
        //printf("Payload Length = %d\n",_rx_payload_len);
        _decode_state = SBP_DECODE_PAYLOAD;
        break;

        /* Expecting payload */
    case SBP_DECODE_PAYLOAD:
        *((uint8_t*)&(_rx_buff) + _rx_buff_count) = b;
        _rx_buff_count++;
        //printf("Payload %d\n",b);
        if(_rx_buff_count >= _rx_payload_len){
            _rx_buff_count = 0;
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

            if (_rx_crc == crc){
                processPayload();
                decodeInit();
                ret = 1;
            } else {
                printf("Checksum failed!\n");
                ret = -1;
            }
            _decode_state = SBP_DECODE_PREAMBLE;
            //decodeInit();
        }
        break;

    default:
        break;
    }
    return ret;
}

void
GPSDriverSBP::processPayload()
{
    double cog_rad;
    struct tm timeinfo;
    time_t epoch;

    switch (_rx_msgtype) {
    case SBP_HEARTBEAT_MSGTYPE:
        printf("I'm a heartbeat message!\n");
        memcpy((uint8_t *)&(_sbp_msg.sbp_heartbeat),(uint8_t*)&(_rx_buff), sizeof(sbp_heartbeat_packet_t));
        _last_heartbeat_received_us = gps_absolute_time();
        break;

    case SBP_GPS_TIME_MSGTYPE:
        printf("I'm a GPS TIME message!\n");
        memcpy((uint8_t *)&(_sbp_msg.sbp_gps_time),(uint8_t*)&(_rx_buff), sizeof(sbp_gpstime_packet_t));
        break;

    case SBP_UTC_TIME_MSGTYPE:
        printf("I'm a UTC TIME message!\n");
        memcpy((uint8_t *)&(_sbp_msg.sbp_utc_time),(uint8_t*)&(_rx_buff), sizeof(sbp_utctime_packet_t));

        /* convert to unix timestamp */
        memset(&timeinfo, 0, sizeof(timeinfo));
        timeinfo.tm_year = _sbp_msg.sbp_utc_time.year - 1900;
        timeinfo.tm_mon  = _sbp_msg.sbp_utc_time.months;
        timeinfo.tm_mday = _sbp_msg.sbp_utc_time.day;
        timeinfo.tm_hour = _sbp_msg.sbp_utc_time.hours;
        timeinfo.tm_min  = _sbp_msg.sbp_utc_time.minutes;
        timeinfo.tm_sec  = _sbp_msg.sbp_utc_time.seconds;

#ifndef NO_MKTIME

        epoch = mktime(&timeinfo);

        if (epoch > GPS_EPOCH_SECS) {
            // FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
            // and control its drift. Since we rely on the HRT for our monotonic
            // clock, updating it from time to time is safe.

            timespec ts;
            memset(&ts, 0, sizeof(ts));
            ts.tv_sec = epoch;
            ts.tv_nsec = _sbp_msg.sbp_utc_time.nanoseconds;
            setClock(ts);

            _gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
            _gps_position->time_utc_usec += _sbp_msg.sbp_utc_time.nanoseconds/1000; //Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0
        }
#else
        _gps_position->time_utc_usec = 0;

#endif
        /*the field 'timestamp' is for the position & velocity (microseconds)*/
        _gps_position->timestamp = gps_absolute_time(); // time since system start (microseconds)
        //int32 timestamp_time_relative	# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
        break;

    case SBP_POS_LLH_MSGTYPE:
        printf("I'm a POS LLH message!\n");
        memcpy((uint8_t *)&(_sbp_msg.sbp_pos_llh),(uint8_t*)&(_rx_buff), sizeof(sbp_pos_llh_packet_t));

        _gps_position->lat = static_cast<int32_t>(_sbp_msg.sbp_pos_llh.lat*1e7); // Latitude in 1E-7 degrees
        _gps_position->lon = static_cast<int32_t>(_sbp_msg.sbp_pos_llh.lon*1e7); // Longitude in 1E-7 degrees
        _gps_position->alt_ellipsoid = static_cast<int32_t>(_sbp_msg.sbp_pos_llh.height*1e3); // Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
        printf("_sbp_msg.sbp_pos_llh.height %d\n",static_cast<int32_t>(_sbp_msg.sbp_pos_llh.height*1e3));

        //        if(_last_heartbeat_received_us == 0) return -1;
        //        uint64_t time_now = gps_absolute_time();
        //        if(time_now - _last_heartbeat_received_us > SBP_TIMEOUT*1000){
        //            _gps_position->fix_type = 1;
        //            PX4_WARN("No Heartbeats from Piksi! Status to NO_FIX.");
        //            //return -1;
        //        }

        //  uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        switch (_sbp_msg.sbp_pos_llh.flags.fix_mode) {
        case 0:
            _gps_position->fix_type = 0;
            break;
        case 1:
            _gps_position->fix_type = 3;
            break;
        case 2:
            _gps_position->fix_type = 4;
            break;
        case 3:
            _gps_position->fix_type = 5;
            break;
        case 4:
            _gps_position->fix_type = 6;
            break;
        case 5:
            _gps_position->fix_type = 8;
            break;
        case 6:
            _gps_position->fix_type = 4;
            break;
        default:
            _gps_position->fix_type = 0;
            break;
        }
        _gps_position->eph = static_cast<float>(_sbp_msg.sbp_pos_llh.h_accuracy/1000.0);// GPS horizontal position accuracy (metres)
        _gps_position->epv = static_cast<float>(_sbp_msg.sbp_pos_llh.v_accuracy/1000.0);// GPS vertical position accuracy (metres)

        _gps_position->satellites_used = static_cast<uint8_t>(_sbp_msg.sbp_pos_llh.n_sats);//Number of satellites used

        _gps_position->heading = NAN;// heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
        _gps_position->heading_offset = NAN;// heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])

        break;

    case SBP_DOPS_MSGTYPE:
        printf("I'm a SBP DOPS message!\n");
        memcpy((uint8_t *)&(_sbp_msg.sbp_dops),(uint8_t*)&(_rx_buff), sizeof(sbp_dops_packet_t));

        _gps_position->hdop = static_cast<float>(_sbp_msg.sbp_dops.hdop*0.01f); // Horizontal dilution of precision
        _gps_position->vdop = static_cast<float>(_sbp_msg.sbp_dops.vdop*0.01f); // Vertical dilution of precision

        break;

    case SBP_VEL_NED_COV_MSGTYPE:
        printf("I'm a SBP VEL NED message!\n");
        memcpy((uint8_t *)&(_sbp_msg.sbp_vel_ned_cov),(uint8_t*)&(_rx_buff), sizeof(sbp_vel_ned_cov_packet_t));

        if(_sbp_msg.sbp_vel_ned_cov.flags.vel_mode > 0){

            _gps_position->vel_ned_valid = true;// True if NED velocity is valid
            _gps_position->vel_n_m_s = static_cast<float>(_sbp_msg.sbp_vel_ned_cov.n*1000);// GPS North velocity, (metres/sec)
            _gps_position->vel_e_m_s = static_cast<float>(_sbp_msg.sbp_vel_ned_cov.e*1000);// GPS East velocity, (metres/sec)
            _gps_position->vel_d_m_s = static_cast<float>(_sbp_msg.sbp_vel_ned_cov.d*1000);// GPS Down velocity, (metres/sec)
            _gps_position->vel_m_s   = static_cast<float>(sqrtf((_gps_position->vel_n_m_s * _gps_position->vel_n_m_s)
                                                                +(_gps_position->vel_e_m_s * _gps_position->vel_e_m_s)));// GPS ground speed, (metres/sec)
            cog_rad = atan2f(_gps_position->vel_e_m_s, _gps_position->vel_n_m_s);//Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
            if(cog_rad > M_PI)
                cog_rad = cog_rad - 2*M_PI;
            else if(cog_rad < -M_PI)
                cog_rad = cog_rad - 2*M_PI;
            _gps_position->cog_rad = cog_rad;

            _gps_position->s_variance_m_s = (sqrtf((_sbp_msg.sbp_vel_ned_cov.cov_n_n*_sbp_msg.sbp_vel_ned_cov.cov_n_n)
                                                  +(_sbp_msg.sbp_vel_ned_cov.cov_e_e*_sbp_msg.sbp_vel_ned_cov.cov_e_e)
                                                  +(_sbp_msg.sbp_vel_ned_cov.cov_d_d*_sbp_msg.sbp_vel_ned_cov.cov_d_d)));//GPS speed accuracy estimate, (metres/sec)

        } else {
            _gps_position->vel_ned_valid = false;
            _gps_position->cog_rad = 0;
            _gps_position->vel_n_m_s = 0;
            _gps_position->vel_e_m_s = 0;
            _gps_position->vel_d_m_s = 0;
            _gps_position->vel_m_s   = 0;
            _gps_position->s_variance_m_s = 0;

        }
        break;

    case SBP_EXT_EVENT_MSGTYPE:
        printf("I'm a SBP EXT_EVENT message!\n");
        memcpy((uint8_t *)&(_sbp_msg.sbp_ext_event),(uint8_t*)&(_rx_buff), sizeof(sbp_ext_event_packet_t));
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
    _decode_state = SBP_DECODE_PREAMBLE;
}

