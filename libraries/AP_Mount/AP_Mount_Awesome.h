/*
  SToRM32 mount using serial protocol backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"


class AP_Mount_Awesome : public AP_Mount_Backend
{

public:
    // Constructor
	AP_Mount_Awesome(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

private:

    // read_incoming
    void parse_data();

    bool parse_nmea0183(char c);

    bool parse_camera(char c);

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

    struct buf _buffData;
    PARSE_STATUS1 _parseStatus;
};
