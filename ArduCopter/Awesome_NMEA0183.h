/*
 * Awesome_NMEA0183.h
 *
 *  Created on: 2019Äê9ÔÂ12ÈÕ
 *      Author: JoytonAwesome
 */

#ifndef ARDUCOPTER_AWESOME_NMEA0183_H_
#define ARDUCOPTER_AWESOME_NMEA0183_H_

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>


class Awesome_NMEA0183
{

public:
	Awesome_NMEA0183(void);

    void init(const AP_SerialManager& serial_manager);

    void update();

    void status_msg(mavlink_channel_t chan);

    void write_data(uint8_t* data, uint16_t len);

private:

    void parse_data();

    bool parse_nmea0183(char c);

    void send_data64(mavlink_channel_t chan);

    uint8_t hex_to_uint8(char c);

    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised;              // true once the driver has been initialised

    typedef enum PARSE_STATUS1{
    	NMEA0183_NONE = 0,
    	NMEA0183_HEADER = 1,
		NMEA0183_DATA,
		NEMA0183_END_DOT,
		NEMA0183_CRC
    }PARSE_STATUS1;

    struct buf{
    	uint8_t count;
    	uint8_t crc;
    	uint8_t crcCount;
    	uint8_t data[64];
    };

    uint8_t data64[64];
    uint8_t len;
    bool dataEnable;

    struct buf _buffData;
    PARSE_STATUS1 _parseStatus;
};



#endif /* ARDUCOPTER_AWESOME_NMEA0183_H_ */
