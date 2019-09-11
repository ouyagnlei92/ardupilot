#include "AP_Mount_Awesome.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Mount_AWesome::AP_Mount_AWesome(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _port(nullptr),
    _initialised(false),
    _parseStatus(NMEA0183_NONE),
{}

// init - performs any required initialisation for this instance
void AP_Mount_Awesome::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Awesome, 0);  // 发现串口
    if (_port) {
        _initialised = true;
        //
    }

}

// update mount position - should be called periodically
void AP_Mount_Awesome::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }


    parse_data(); // read the incoming messages from the gimbal

}

void AP_Mount_Awesome::parse_data()
{
    uint8_t data;
    int16_t numc;
    numc = _port->available();

	if (numc < 0 ){
		return;
	}

	for (int16_t i = 0; i < numc; i++) {        // Process bytes received
		data = _port->read();
		if(parse_nmea0183(data))
		{
			//打包成data64发送出去
			for(int16_t j = 0; j<_buffData.count+1; ++j ) data[j] = _buffData.data[j];
			len = _buffData.count+1;
			gcs().send_message(MSG_DATA64);
			gcs().send_text(MAV_SEVERITY_INFO, "Have NMEA0183");
		}
	}
}

bool AP_Mount_Awesome::parse_nmea0183(char c)
{
	if( c=='$' && _parseStatus==NMEA0183_NONE )
	{
		_buffData.crc = 0;
		_buffData.crcCount = 0;
		_buffData.count = 0;
		_parseStatus = NMEA0183_HEADER;
		_buffData.data[_buffData.count] = c;
		++_buffData.count;
		return false;
	}else if(_parseStatus==NMEA0183_HEADER && c!=',')
	{
		_buffData.data[_buffData.count] = c;
		_buffData.crc ^= c;
		++_buffData.count;
		return false;
	}else if(_parseStatus==NMEA0183_HEADER && c==',')
	{
		_buffData.data[_buffData.count] = c;
		_buffData.crc ^= c;
		++_buffData.count;
		_parseStatus=NMEA0183_DATA;
		return false;
	}else if(_parseStatus==NMEA0183_DATA && c!='*')
	{
		_buffData.data[_buffData.count] = c;
		_buffData.crc ^= c;
		++_buffData.count;
		return false;
	}else if(_parseStatus==NMEA0183_DATA && c=='*')
	{
		_buffData.data[_buffData.count] = c;
		++_buffData.count;
		_parseStatus=NEMA0183_CRC;
		return false;
	}else if(_parseStatus==NEMA0183_CRC)
	{
		++_buffData.crcCount;
		_buffData.data[_buffData.count]=c;
		if(_buffData.crcCount==2)
		{
			if((hex_to_uint8(_buffData.data[_buffData.count])+(hex_to_uint8(_buffData.data[_buffData.count-1])<<4))==_buffData.crc)
			{
				_parseStatus=NMEA0183_NONE;
				return true;
			}else
			{
				_buffData.count = 0;
				_parseStatus=NMEA0183_NONE;
				return false;
			}
		}
	   _buffData.count++;
	   return false;
	}else if(_buffData.count>64)
	{
		_buffData.count = 0;
		_parseStatus=NMEA0183_NONE;
	}
	return false;
}

void AP_Mount_Awesome::send_data64(mavlink_channel_t chan)
{
	mavlink_msg_data64_send(chan, 'N', len, data);
}

uint8_t AP_Mount_Awesome::hex_to_uint8(char c)
{
	if(c>='0'&&c<='9') return (uint8_t)(c-'0');
	if(c>='a'&&c<='f') return (uint8_t)(c-'a'+10);
	if(c>='A'&&c<='F') return (uint8_t)(c-'A'+10);
}

bool AP_Mount_Awesome::parse_camera(char c)
{
	return true;
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Awesome::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_Awesome::set_mode(enum MAV_MOUNT_MODE mode)
{

}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Awesome::status_msg(mavlink_channel_t chan)
{
	send_data64(chan);
}


