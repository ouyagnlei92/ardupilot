#include "Awesome_NMEA0183.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

Awesome_NMEA0183::Awesome_NMEA0183(void) :
    _port(nullptr),
    _initialised(false),
    _parseStatus(NMEA0183_NONE),
	dataEnable(false),
{}


void Awesome_NMEA0183::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Awesome_NMEA0183, 0);  // 发现串口
    if (_port) {
        _initialised = true;
        //
    }

}

// update mount position - should be called periodically
void Awesome_NMEA0183::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    parse_data(); // read the incoming messages from the gimbal

}

void Awesome_NMEA0183::write_data(uint8_t* data, uint16_t len)
{
	if (!_initialised) {
		return;
	}
	if(_port->txspace()>len) _port->write(data, len);
}

void Awesome_NMEA0183::parse_data()
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
			for(uint8_t j = 0; j<_buffData.count+1; ++j ){ data64[j] = (uint8_t)_buffData.data[j];}
			len = _buffData.count+1;
			gcs().send_message(MSG_DATA64);
			gcs().send_text(MAV_SEVERITY_INFO, "Have NMEA0183");
		}
	}
}

bool Awesome_NMEA0183::parse_nmea0183(char c)
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
				dataEnable = true;
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

void Awesome_NMEA0183::send_data64(mavlink_channel_t chan)
{
	mavlink_msg_data64_send(chan, 'N', len, data64);
}

uint8_t Awesome_NMEA0183::hex_to_uint8(char c)
{
	if(c>='0'&&c<='9') return (uint8_t)(c-'0');
	if(c>='a'&&c<='f') return (uint8_t)(c-'a'+10);
	if(c>='A'&&c<='F') return (uint8_t)(c-'A'+10);
return 0;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void Awesome_NMEA0183::status_msg(mavlink_channel_t chan)
{
	if(dataEnable)
	{
		send_data64(chan);
		dataEnable = false;
	}
}


