/*
 * Awesome_Camera.h
 *
 *  Created on: 2019Äê9ÔÂ12ÈÕ
 *      Author: JoytonAwesome
 */

#ifndef ARDUCOPTER_AWESOME_CAMERA_H_
#define ARDUCOPTER_AWESOME_CAMERA_H_


#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"


class Awesome_Camera
{

public:
	Awesome_Camera(void);

    void init(const AP_SerialManager& serial_manager);

    void update();

    void status_msg(mavlink_channel_t chan);

    void write_data(uint8_t* data, uint16_t len);

private:

    void parse_data();

    bool parse_camera(char c);

    void send_data64(mavlink_channel_t chan);

    uint8_t hex_to_uint8(char c);

    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised;              // true once the driver has been initialised

    typedef enum PARSE_STATUS1{
    	CAMERA_NONE = 0,
    	CAMERA_HEADER,
		CAMERA_END_DOT1,
    }PARSE_STATUS1;

    struct buf{
    	uint8_t count;
    	uint8_t data[64];
    };

    uint8_t data64[64];
    uint8_t len;
    bool dataEnable;

    struct buf _buffData;
    PARSE_STATUS1 _parseStatus;
};


#endif /* ARDUCOPTER_AWESOME_CAMERA_H_ */
