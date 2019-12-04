/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//

/// @file	AP_GPS_JOYTON.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <AP_Common/AP_Common.h>

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>

#include "AP_GPS_JOYTON.h"

extern const AP_HAL::HAL& hal;

// optionally log all NMEA data for debug purposes
// #define NMEA_LOG_PATH "nmea.log"

#ifdef NMEA_LOG_PATH
#include <stdio.h>
#endif

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)        (_x - '0')
#define hexdigit(x) ((x)>9?'A'+((x)-10):'0'+(x))

AP_GPS_JOYTON::AP_GPS_JOYTON(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    // this guarantees that _term is always nul terminated
    memset(_term, 0, sizeof(_term));

    gps.send_blob_start(state.instance, JOYTON_GPS_CONFIG, sizeof(JOYTON_GPS_CONFIG));

    // this guarantees that _term is always nul terminated
    memset(_term, 0, sizeof(_term));

    init_event();  /* add by awesome */
}

bool AP_GPS_JOYTON::read(void)
{
    int16_t numc;
    bool parsed = false;

    numc = port->available();
    while (numc--) {
        char c = port->read();
#ifdef NMEA_LOG_PATH
        static FILE *logf = nullptr;
        if (logf == nullptr) {
            logf = fopen(NMEA_LOG_PATH, "wb");
        }
        if (logf != nullptr) {
            ::fwrite(&c, 1, 1, logf);
        }
#endif
        if (_decode(c)) {
            parsed = true;
        }
        /* parse event  add by awesome */
        if(parse_eventa(c))
        {
        	write_Log_mark_event();
        	GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Mark Event index: %d", eventa_data.event_index);
        	/*
        	GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "lat: %.8f lon: %.8f alt: %.2f",eventa_data.lat,eventa_data.lon,eventa_data.alt );
        	GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "pos: %d stats: %d index: %df",eventa_data.pos_type, eventa_data.stats_num, eventa_data.event_index);
        	GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "alt_error: %.2f", eventa_data.alt_error );
        	*/
        }
    }
    return parsed;
}

bool AP_GPS_JOYTON::_decode(char c)
{
    bool valid_sentence = false;

    switch (c) {
    case ',': // term terminators
        _parity ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;

    case '$': // sentence begin
        _term_number = _term_offset = 0;
        _parity = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _gps_data_good = false;
        return valid_sentence;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term)
        _parity ^= c;

    return valid_sentence;
}

//
// internal utilities
//
int16_t AP_GPS_JOYTON::_from_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

int32_t AP_GPS_JOYTON::_parse_decimal_100(const char *p)
{
    char *endptr = nullptr;
    long ret = 100 * strtol(p, &endptr, 10);
    int sign = ret < 0 ? -1 : 1;

    if (ret >= (long)INT32_MAX) {
        return INT32_MAX;
    }
    if (ret <= (long)INT32_MIN) {
        return INT32_MIN;
    }
    if (endptr == nullptr || *endptr != '.') {
        return ret;
    }

    if (isdigit(endptr[1])) {
        ret += sign * 10 * DIGIT_TO_VAL(endptr[1]);
        if (isdigit(endptr[2])) {
            ret += sign * DIGIT_TO_VAL(endptr[2]);
            if (isdigit(endptr[3])) {
                ret += sign * (DIGIT_TO_VAL(endptr[3]) >= 5);
            }
        }
    }
    return ret;
}

/*
  parse a NMEA latitude/longitude degree value. The result is in degrees*1e7
 */
uint32_t AP_GPS_JOYTON::_parse_degrees()
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;

    // scan for decimal point or end of field
    for (p = _term; *p && isdigit(*p); p++)
        ;
    q = _term;

    // convert degrees
    while ((p - q) > 2 && *q) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }

    // convert minutes
    while (p > q && *q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }

    // convert fractional minutes
    if (*p == '.') {
        q = p + 1;
        float frac_scale = 0.1f;
        while (*q && isdigit(*q)) {
            frac_min += DIGIT_TO_VAL(*q) * frac_scale;
            q++;
            frac_scale *= 0.1f;
        }
    }
    ret = (deg * (int32_t)10000000UL);
    ret += (min * (int32_t)10000000UL / 60);
    ret += (int32_t) (frac_min * (1.0e7f / 60.0f));
    return ret;
}

/*
  see if we have a new set of NMEA messages
 */
bool AP_GPS_JOYTON::_have_new_message()
{
    if (_last_RMC_ms == 0 ||
        _last_GGA_ms == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _last_RMC_ms > 150 ||
        now - _last_GGA_ms > 150) {
        return false;
    }
    if (_last_VTG_ms != 0 &&
        now - _last_VTG_ms > 150) {
        return false;
    }
    // prevent these messages being used again
    if (_last_VTG_ms != 0) {
        _last_VTG_ms = 1;
    }
    _last_GGA_ms = 1;
    _last_RMC_ms = 1;
    return true;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_JOYTON::_term_complete()
{
    // handle the last term in a message
    if (_is_checksum_term) {
        uint8_t checksum = 16 * _from_hex(_term[0]) + _from_hex(_term[1]);
        if (checksum == _parity) {
            if (_gps_data_good) {
                uint32_t now = AP_HAL::millis();
                switch (_sentence_type) {
                case _GPS_SENTENCE_RMC:
                    _last_RMC_ms = now;
                    //time                        = _new_time;
                    //date                        = _new_date;
                    state.location.lat     = _new_latitude;
                    state.location.lng     = _new_longitude;
                    state.ground_speed     = _new_speed*0.01f;
                    state.ground_course    = wrap_360(_new_course*0.01f);
                    make_gps_time(_new_date, _new_time * 10);
                    state.last_gps_time_ms = now;
                    fill_3d_velocity();
                    break;
                case _GPS_SENTENCE_GGA:
                    _last_GGA_ms = now;
                    state.location.alt  = _new_altitude;
                    state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;
                    state.num_sats      = _new_satellite_count;
                    state.hdop          = _new_hdop;
                    switch(_new_quality_indicator) {
                    case 0: // Fix not available or invalid
                        state.status = AP_GPS::NO_FIX;
                        break;
                    case 1: // GPS SPS Mode, fix valid
                        state.status = AP_GPS::GPS_OK_FIX_3D;
                        break;
                    case 2: // Differential GPS, SPS Mode, fix valid
                        state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                        break;
                    case 3: // GPS PPS Mode, fix valid
                        state.status = AP_GPS::GPS_OK_FIX_3D;
                        break;
                    case 4: // Real Time Kinematic. System used in RTK mode with fixed integers
                        state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                        break;
                    case 5: // Float RTK. Satellite system used in RTK mode, floating integers
                        state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                        break;
                    case 6: // Estimated (dead reckoning) Mode
                        state.status = AP_GPS::NO_FIX;
                        break;
                    default://to maintain compatibility with MAV_GPS_INPUT and others
                        state.status = AP_GPS::GPS_OK_FIX_3D;
                        break;
                    }
                    break;
                case _GPS_SENTENCE_VTG:
                    _last_VTG_ms = now;
                    state.ground_speed  = _new_speed*0.01f;
                    state.ground_course = wrap_360(_new_course*0.01f);
                    fill_3d_velocity();
                    // VTG has no fix indicator, can't change fix status
                    break;
                }
            } else {
                switch (_sentence_type) {
                case _GPS_SENTENCE_RMC:
                case _GPS_SENTENCE_GGA:
                    // Only these sentences give us information about
                    // fix status.
                    state.status = AP_GPS::NO_FIX;
                }
            }
            // see if we got a good message
            return _have_new_message();
        }
        // we got a bad message, ignore it
        return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        /*
          The first two letters of the NMEA term are the talker
          ID. The most common is 'GP' but there are a bunch of others
          that are valid. We accept any two characters here.
         */
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = _GPS_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "RMC") == 0) {
            _sentence_type = _GPS_SENTENCE_RMC;
        } else if (strcmp(term_type, "GGA") == 0) {
            _sentence_type = _GPS_SENTENCE_GGA;
        } else if (strcmp(term_type, "VTG") == 0) {
            _sentence_type = _GPS_SENTENCE_VTG;
            // VTG may not contain a data qualifier, presume the solution is good
            // unless it tells us otherwise.
            _gps_data_good = true;
        } else {
            _sentence_type = _GPS_SENTENCE_OTHER;
        }
        return false;
    }

    // 32 = RMC, 64 = GGA, 96 = VTG
    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
        switch (_sentence_type + _term_number) {
        // operational status
        //
        case _GPS_SENTENCE_RMC + 2: // validity (RMC)
            _gps_data_good = _term[0] == 'A';
            break;
        case _GPS_SENTENCE_GGA + 6: // Fix data (GGA)
            _gps_data_good = _term[0] > '0';
            _new_quality_indicator = _term[0] - '0';
            break;
        case _GPS_SENTENCE_VTG + 9: // validity (VTG) (we may not see this field)
            _gps_data_good = _term[0] != 'N';
            break;
        case _GPS_SENTENCE_GGA + 7: // satellite count (GGA)
            _new_satellite_count = atol(_term);
            break;
        case _GPS_SENTENCE_GGA + 8: // HDOP (GGA)
            _new_hdop = (uint16_t)_parse_decimal_100(_term);
            break;

        // time and date
        //
        case _GPS_SENTENCE_RMC + 1: // Time (RMC)
        case _GPS_SENTENCE_GGA + 1: // Time (GGA)
            _new_time = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 9: // Date (GPRMC)
            _new_date = atol(_term);
            break;

        // location
        //
        case _GPS_SENTENCE_RMC + 3: // Latitude
        case _GPS_SENTENCE_GGA + 2:
            _new_latitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 4: // N/S
        case _GPS_SENTENCE_GGA + 3:
            if (_term[0] == 'S')
                _new_latitude = -_new_latitude;
            break;
        case _GPS_SENTENCE_RMC + 5: // Longitude
        case _GPS_SENTENCE_GGA + 4:
            _new_longitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 6: // E/W
        case _GPS_SENTENCE_GGA + 5:
            if (_term[0] == 'W')
                _new_longitude = -_new_longitude;
            break;
        case _GPS_SENTENCE_GGA + 9: // Altitude (GPGGA)
            _new_altitude = _parse_decimal_100(_term);
            break;

        // course and speed
        //
        case _GPS_SENTENCE_RMC + 7: // Speed (GPRMC)
        case _GPS_SENTENCE_VTG + 5: // Speed (VTG)
            _new_speed = (_parse_decimal_100(_term) * 514) / 1000;       // knots-> m/sec, approximiates * 0.514
            break;
        case _GPS_SENTENCE_RMC + 8: // Course (GPRMC)
        case _GPS_SENTENCE_VTG + 1: // Course (VTG)
            _new_course = _parse_decimal_100(_term);
            break;
        }
    }

    return false;
}

/*
  detect a NMEA GPS. Adds one byte, and returns true if the stream
  matches a NMEA string
 */
bool
AP_GPS_JOYTON::_detect(struct NMEA_detect_state &state, uint8_t data)
{
	switch (state.step) {
	case 0:
		state.ck = 0;
		if ('$' == data) {
			state.step++;
		}
		break;
	case 1:
		if ('*' == data) {
			state.step++;
		} else {
			state.ck ^= data;
		}
		break;
	case 2:
		if (hexdigit(state.ck>>4) == data) {
			state.step++;
		} else {
			state.step = 0;
		}
		break;
	case 3:
		if (hexdigit(state.ck&0xF) == data) {
            state.step = 0;
			return true;
		}
		state.step = 0;
		break;
    }
    return false;
}

/* EVENTALLA */
void AP_GPS_JOYTON::eventa_init(eventa* event)
{
	if( (void*)0!=event )
	{
		unsigned char i = 0;
		event->index = 0;
		event->status = EVENT_NONE;

		for( ; i<EVENT_MAX_LEN; ++i )
		{
			event->buff[i] = 0;
		}
	}
}

/* add by awesome */

/*** �¼ӽ���Eventalla ���� ***/
/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long AP_GPS_JOYTON::CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for( j=8; j>0; j-- )
	{
		if( ulCRC & 1 )
			ulCRC = ( ulCRC>>1 ) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}

	return ulCRC;
}

unsigned long AP_GPS_JOYTON::CalulateSingleCRC32(unsigned char c, unsigned long crc)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;

	ulTemp1 = ( crc >> 8 ) & 0x00FFFFFFL;
	ulTemp2 = CRC32Value( ((int) crc ^ c ) & 0xff );

	return (unsigned long)(ulTemp1 ^ ulTemp2);
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long AP_GPS_JOYTON::CalculateBlockCRC32(
	unsigned long ulCount, /* Number of bytes in the data block */
	unsigned char *ucBuffer ) /* Data block */
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( ulCount-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return( ulCRC );
}

void AP_GPS_JOYTON::CRC32ToString(unsigned long crc32, unsigned char* buff)
{
	char str[] = "0123456789abcdef";
	unsigned long temp = 0;

	if(NULL==buff) return;

	temp = (crc32 & 0x0FF) & 0xFF;
	buff[7] = str[temp%16];
	buff[6] = str[temp/16];

	temp = ( (crc32 & 0x0FF00)>>8 ) & 0xFF;
	buff[4] = str[temp/16];
	buff[5] = str[temp%16];

	temp = ( (crc32 & 0x0FF0000)>>16 ) & 0xFF;
	buff[2] = str[temp/16];
	buff[3] = str[temp%16];

	temp = ( (crc32 & 0x0FF000000)>>24) & 0xFF;
	buff[0] = str[temp/16];
	buff[1] = str[temp%16];

	buff[8] = '\0';
}

void AP_GPS_JOYTON::eventa_process(const char *src, unsigned char index)
{
	if(NULL==src) return;

	switch(index)
	{
		case 12: /* pos type */
			eventa_data.pos_type = static_cast<unsigned int>(atoi(src));
			break;
		case 13:   /* lat */
			eventa_data.lat = static_cast<double>(strtod(src, NULL));
			break;
		case 14:  /* lon */
			eventa_data.lon = static_cast<double>(strtod(src, NULL));
			break;
		case 15: /* alt */
			eventa_data.alt = static_cast<float>(strtod(src, NULL));
			break;
		case 16: /* alt_error */
			eventa_data.alt_error = static_cast<float>(strtod(src, NULL));
			break;
		case 25: /* use stats num */
			eventa_data.stats_num = static_cast<unsigned int>(atoi(src));
			break;
		default: break;
	}
}

bool AP_GPS_JOYTON::parse_eventa(unsigned char c)
{
	bool error = false;
	if( '#'==c && EVENT_PARSE_NONE==eventa_status.parse_status )
	{
		eventa_status.parse_status = EVENT_PARSE_AYNC;  /* get the Aync Anscii '#' */
		eventa_status.count = 0;
		eventa_status.dot_count = 0;
		return false;
	}
	switch(eventa_status.parse_status)
	{
		case EVENT_PARSE_AYNC:
			eventa_status.crc32 = CalulateSingleCRC32(c, eventa_status.crc32);
			if(','!=c)
			{
				eventa_status.buff[eventa_status.count++] = c;  /* cache the data */
				if(eventa_status.count>=29) error = true;
			}else if( ','==c && eventa_status.count>0 )
			{
				eventa_status.buff[eventa_status.count] = '\0';
				if( 0==strcmp((const char*)eventa_status.buff, EVENTA_MSG) )
				{
					eventa_status.parse_status = EVENT_PARSE_HEADER;
					eventa_status.count = 0;
				}else error = true;
			}
			break;
		case EVENT_PARSE_HEADER:
			eventa_status.crc32 = CalulateSingleCRC32(c, eventa_status.crc32);
			if(';'!=c)
			{
				if(eventa_status.count>=50) error = true;
			}else
			{
				eventa_status.parse_status = EVENT_PARSE_DATA;
			}
			break;
		case EVENT_PARSE_DATA:
			if(','==c)
			{
				eventa_status.crc32 = CalulateSingleCRC32(c, eventa_status.crc32);
				eventa_status.buff[eventa_status.count] = '\0';
				eventa_status.count = 0;
				++eventa_status.dot_count;
				eventa_process((const char *)eventa_status.buff, eventa_status.dot_count);
			}else if('*'==c)
			{
				eventa_status.parse_status = EVENT_PARSE_CRC;
				eventa_status.count = 0;
			}else
			{
				eventa_status.crc32 = CalulateSingleCRC32(c, eventa_status.crc32);
				eventa_status.buff[eventa_status.count++] = c;
			}
			break;
		case EVENT_PARSE_CRC:
			if(eventa_status.count>=8)
			{
				unsigned char buff[10];
				CRC32ToString(eventa_status.crc32, buff);
				eventa_status.buff[eventa_status.count] = '\0';
				if(0==strcmp((const char*)buff, (const char*)eventa_status.buff))
				{
					eventa_status.parse_status = EVENT_PARSE_SUCCESS;
					++eventa_data.event_index;
				}else error = true;
			}
			eventa_status.buff[eventa_status.count++] = c;
			break;
		default: error = true; break;
	}

	/* parse error, reset the status */
	if( error )
	{
		eventa_status.count = 0;
		eventa_status.dot_count = 0;
		eventa_status.parse_status = EVENT_PARSE_NONE;
		eventa_status.crc32 = 0;
		return false;
	}

	if(EVENT_PARSE_SUCCESS==eventa_status.parse_status)
	{
		eventa_status.count = 0;
		eventa_status.dot_count = 0;
		eventa_status.parse_status = EVENT_PARSE_NONE;
		eventa_status.crc32 = 0;
		return true;
	}

	return false;
}

void AP_GPS_JOYTON::write_Log_mark_event(void)
{
    struct log_camera_mark pkt = {
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(LOG_CAMERA_MARK)),
        time_us     : AP_HAL::micros64(),
        latitude	: eventa_data.lat,
        longitude	: eventa_data.lon,
        amsl_alt    : eventa_data.alt,
        ell_alt 	: eventa_data.alt+eventa_data.alt_error ,
        pos_type	: eventa_data.pos_type,
        index		: eventa_data.event_index,
        roll        : (int16_t)0,
        pitch       : (int16_t)0,
        yaw         : (uint16_t)0,
        stats_num	: (uint8_t)eventa_data.stats_num
    };
    DataFlash_Class* pdata = DataFlash_Class::instance();
    pdata->WriteCriticalBlock(&pkt, sizeof(pkt));
}

/* end add */
