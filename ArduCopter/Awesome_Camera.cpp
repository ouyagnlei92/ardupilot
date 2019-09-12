/*
 * Awesome_Camera.cpp
 *
 *  Created on: 2019年9月12日
 *      Author: JoytonAwesome
 */
#include "Awesome_Camera.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

Awesome_Camera::Awesome_Camera(void) :
    _port(nullptr),
    _initialised(false),
    _parseStatus(CAMERA_NONE),
	dataEnable(false)
{}


void Awesome_Camera::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Awesome_Camera, 0);  // 发现串口
    if (_port) {
        _initialised = true;
        //
    }

}

// update mount position - should be called periodically
void Awesome_Camera::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    parse_data(); // read the incoming messages from the gimbal

}

void Awesome_Camera::write_data(uint8_t* data, uint16_t len)
{
	if (!_initialised) {
		return;
	}
	if(_port->txspace()>len) _port->write(data, len);
}

void Awesome_Camera::parse_data()
{
    uint8_t data;
    int16_t numc;
    numc = _port->available();

	if (numc < 0 ){
		return;
	}

	for (int16_t i = 0; i < numc; i++) {        // Process bytes received
		data = _port->read();
		if(parse_camera(data))
		{
			//打包成data64发送出去
			for(uint8_t j = 0; j<_buffData.count; ++j ){ data64[j] = (uint8_t)_buffData.data[j];}
			len = _buffData.count;
			gcs().send_message(MSG_DATA64);
			gcs().send_text(MAV_SEVERITY_INFO, "Have Camera");
		}
	}
}

bool Awesome_Camera::parse_camera(char c)
{
	if(c=='#' && CAMERA_NONE==_parseStatus)
	{
		_buffData.count = 0;
		_parseStatus = CAMERA_HEADER;
		_buffData.data[_buffData.count] = c;
	    ++_buffData.count;
		return false;
	}else if(CAMERA_HEADER==_parseStatus && c!='\r')
	{
		_buffData.data[_buffData.count] = c;
	     ++_buffData.count;
	     return false;
	}else if(CAMERA_HEADER==_parseStatus && c=='\r')
	{
		_buffData.data[_buffData.count] = c;
	    ++_buffData.count;
	    _parseStatus = CAMERA_END_DOT1;
	    return false;
	}else if(CAMERA_END_DOT1==_parseStatus && c=='\n')
	{
		_buffData.data[_buffData.count] = c;
		++_buffData.count;
		_parseStatus = CAMERA_NONE;
		dataEnable = true;
		return true;
	}else if(_buffData.count>64)
	{
		_buffData.count = 0;
		_parseStatus = CAMERA_NONE;
		return false;
	}else
	{
		_buffData.count = 0;
		_parseStatus = CAMERA_NONE;
		return false;
	}
}

void Awesome_Camera::send_data64(mavlink_channel_t chan)
{
	mavlink_msg_data64_send(chan, 'C', len, data64);
}

uint8_t Awesome_Camera::hex_to_uint8(char c)
{
	if(c>='0'&&c<='9') return (uint8_t)(c-'0');
	if(c>='a'&&c<='f') return (uint8_t)(c-'a'+10);
	if(c>='A'&&c<='F') return (uint8_t)(c-'A'+10);
return 0;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void Awesome_Camera::status_msg(mavlink_channel_t chan)
{
	if(dataEnable)
	{
		send_data64(chan);
		dataEnable = false;
	}
}





