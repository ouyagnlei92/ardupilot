#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Generic.h"
#include <utility>

<<<<<<< HEAD:libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Maxell.cpp
#include <GCS_MAVLink/GCS.h>

#define BATTMONITOR_SMBUS_MAXELL_NUM_CELLS 12
uint8_t maxell_cell_ids[] = { 0x3f,  // cell 1
                              0x3e,  // cell 2
                              0x3d,  // cell 3
                              0x3c,  // cell 4
                              0x3b,  // cell 5
                              0x3a,  // cell 6
                              0x39,  // cell 7
                              0x38,  // cell 8
                              0x37,  // cell 9
                              0x36,  // cell 10
                              0x35,  // cell 11
                              0x34}; // cell 12
=======
uint8_t smbus_cell_ids[] = { 0x3f,  // cell 1
                             0x3e,  // cell 2
                             0x3d,  // cell 3
                             0x3c,  // cell 4
                             0x3b,  // cell 5
                             0x3a,  // cell 6
                             0x39,  // cell 7
                             0x38,  // cell 8
                             0x37,  // cell 9
                             0x36,  // cell 10
                             0x35,  // cell 11
                             0x34}; // cell 12
>>>>>>> upstream/master:libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Generic.cpp

#define SMBUS_READ_BLOCK_MAXIMUM_TRANSFER    0x20   // A Block Read or Write is allowed to transfer a maximum of 32 data bytes.
#define SMBUS_CELL_COUNT_CHECK_TIMEOUT       15     // check cell count for up to 15 seconds

#define SMBUS_READ_BLOCK_MAXIMUM_TRANSFER    32   // A Block Read or Write is allowed to transfer a maximum of 36 data bytes.

#define BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT   0x17    // cycle count
#define BATTMONITOR_SMBUS_MAXELL_BATTERY_DASTATUS2     0x72    // DA Status
#define BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT          0x50    // safety alert
#define BATTMONITOR_SMBUS_MAXELL_PF_ALERT              0x52    // PF alert
#define BATTMONITOR_SMBUS_MAXELL_OPERATION_STATUS      0x54    // OperationStatus  Page-139
#define BATTMONITOR_SMBUS_MAXELL_CHARGING_STATUS       0x55    // ChargingStatus   Page-139
#define BATTMONITOR_SMBUS_MAXELL_GAUGING_STATUS        0x56    // GaugingStatus    Page-139

#define SMART_BATTERY_PF_ALERT		(1<<0)
#define SMART_BATTERY_SAFE_ALERT	(1<<1)
#define SMART_BATTERY_OPERATION_STATUS (1<<2)
#define SMART_BATTERY_CHARGING_STATUS  (1<<3)
#define SMART_BATTERY_GUAING_STATUS    (1<<4)
#define SMART_BATTERY_ALL_STATUS       (SMART_BATTERY_PF_ALERT | SMART_BATTERY_SAFE_ALERT | SMART_BATTERY_OPERATION_STATUS\
		                                | SMART_BATTERY_CHARGING_STATUS | SMART_BATTERY_GUAING_STATUS)

// Constructor
AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev))
{
    _battery_status = 0;
}

void AP_BattMonitor_SMBus_Generic::timer()
{
	// check if PEC is supported
    if (!check_pec_support()) {
        return;
    }

    uint16_t data;
    uint32_t tnow = AP_HAL::micros();
    static uint32_t tstime = AP_HAL::micros();

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data / 1000.0f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // assert that BATTMONITOR_SMBUS_NUM_CELLS_MAX must be no more than smbus_cell_ids
    static_assert(BATTMONITOR_SMBUS_NUM_CELLS_MAX <= ARRAY_SIZE(smbus_cell_ids), "BATTMONITOR_SMBUS_NUM_CELLS_MAX must be no more than smbus_cell_ids");

    // check cell count
    if (!_cell_count_fixed) {
        if (_state.healthy) {
            // when battery first becomes healthy, start check of cell count
            if (_cell_count_check_start_us == 0) {
                _cell_count_check_start_us = tnow;
            }
            if (tnow - _cell_count_check_start_us > (SMBUS_CELL_COUNT_CHECK_TIMEOUT * 1e6)) {
                // give up checking cell count after 15sec of continuous healthy battery reads
                _cell_count_fixed = true;
            }
        } else {
            // if battery becomes unhealthy restart cell count check
            _cell_count_check_start_us = 0;
        }
    }

    // read cell voltages
    for (uint8_t i = 0; i < (_cell_count_fixed ? _cell_count : BATTMONITOR_SMBUS_NUM_CELLS_MAX); i++) {
        if (read_word(smbus_cell_ids[i], data) && (data > 0) && (data < UINT16_MAX)) {
            _has_cell_voltages = true;
            if(_cell_health_counter[i]>0)
            {
            	--_cell_health_counter[i];
            	if(_cell_health_counter[i]>25)
            		_cell_health_counter[i] = 0;
            }
            _state.cell_voltages.cells[i] = data;
<<<<<<< HEAD:libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Maxell.cpp
        } else {
            ++_cell_health_counter[i];
        	if(_cell_health_counter[i]>=75)
        	{
        		_cell_health_counter[i] = 75;
        		_state.cell_voltages.cells[i] = UINT16_MAX;
        	}
=======
            _last_cell_update_us[i] = tnow;
            if (!_cell_count_fixed) {
                _cell_count = MAX(_cell_count, i + 1);
            }
        } else if ((tnow - _last_cell_update_us[i]) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
            _state.cell_voltages.cells[i] = UINT16_MAX;
>>>>>>> upstream/master:libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Generic.cpp
        }
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        return;
    }

    // read current (A)
    if (read_word(BATTMONITOR_SMBUS_CURRENT, data)) {
        _state.current_amps = -(float)((int16_t)data) / 100.0f;
        _state.last_time_micros = tnow;
    }

    //read cycle count
    if (read_word(BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT, data)) {
            _state.cycle_count = (uint16_t)data;
            _state.last_time_micros = tnow;
    }

    // read more information
    if(AP_HAL::millis()>=10000)
    {
    	get_battery_status();
    }

    read_full_charge_capacity();

    // FIXME: Perform current integration if the remaining capacity can't be requested
    read_remaining_capacity();

    read_temp();

    read_serial_number();

<<<<<<< HEAD:libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Maxell.cpp
    if(tnow-tstime>5000000ul)  /* 5s update temp */
    {
		uint8_t tsbuff[16]={0};
		if(read_block(BATTMONITOR_SMBUS_MAXELL_BATTERY_DASTATUS2, (uint8_t*)tsbuff, false)>0)
		{
			/* aaAA bbBB ccCC ddDD eeEE ffFF ggGG hhHH */
			/* ExtAveCellVoltage,VAUX Voltage, TS1Temp, TS2Temp, TS3Temp, CellTemp, FETTemp, internal Gauge Temp */
			//gcs().send_text(MAV_SEVERITY_WARNING, "%d,%d,%d,%d,%d,%d",tsbuff[4],tsbuff[5],tsbuff[6],tsbuff[7],tsbuff[8],tsbuff[9]);
            _state.TSx[0]=(int16_t)(( (tsbuff[4]+((uint16_t)tsbuff[5]<<8))*0.1-273.15)*100);
			_state.TSx[1]=(int16_t)(( (tsbuff[6]+((uint16_t)tsbuff[7]<<8))*0.1-273.15)*100);
			_state.TSx[2]=(int16_t)(( (tsbuff[8]+((uint16_t)tsbuff[9]<<8))*0.1-273.15)*100);
		}
		tstime = AP_HAL::micros();
    }
=======
    read_cycle_count();
>>>>>>> upstream/master:libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Generic.cpp
}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_Generic::read_block(uint8_t reg, uint8_t* data, bool append_zero) const
{
    // get length
    uint8_t bufflen;
    // read byte (first byte indicates the number of bytes in the block)
    if (!_dev->read_registers(reg, &bufflen, 1)) {   /* 读取一次，获取长度     */
        //gcs().send_text(MAV_SEVERITY_WARNING, "reg=%d,len=%d",reg, bufflen);
        return 0;
    }

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > SMBUS_READ_BLOCK_MAXIMUM_TRANSFER) {   /* 不能超过36个字节  */
        return 0;
    }

    // buffer to hold results (2 extra byte returned holding length and PEC)
    const uint8_t read_size = bufflen + 1 + (_pec_supported ? 1 : 0); /* 数据长度 + 长度标志 + PEC标志 */
    uint8_t buff[read_size];

    // read bytes
    if (!_dev->read_registers(reg, buff, read_size)) {  /* 读取块区数据 */
        return 0;
    }

    // check PEC
    if (_pec_supported) {
        uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, bufflen+1);
        if (pec != buff[bufflen+1]) {
            return 0;
        }
    }

    // copy data (excluding PEC and len)
    memcpy(data, &buff[1], bufflen); /* 不包括长度和PEC位 */

    // optionally add zero to end
    if (append_zero) {
        data[bufflen] = '\0'; /* 选择插入字符串结束标志 */
    }

    // return success
    return bufflen;
}

// check if PEC supported with the version value in SpecificationInfo() function
// returns true once PEC is confirmed as working or not working
bool AP_BattMonitor_SMBus_Generic::check_pec_support()
{
    // exit immediately if we have already confirmed pec support
    if (_pec_confirmed) {
        return true;
    }

/*
    // specification info
    uint16_t data;
    if (!read_word(BATTMONITOR_SMBUS_SPECIFICATION_INFO, data)) {
        return false;
    }


    // extract version
    uint8_t version = (data & 0xF0) >> 4;

    // version less than 0011b (i.e. 3) do not support PEC
    if (version < 3) {
        _pec_supported = false;
        _pec_confirmed = true;
        return true;
    }

    // check manufacturer name
    uint8_t buff[SMBUS_READ_BLOCK_MAXIMUM_TRANSFER + 1];
    if (read_block(BATTMONITOR_SMBUS_MANUFACTURE_NAME, buff, true)) {
        // Hitachi maxell batteries do not support PEC
        if (strcmp((char*)buff, "Hitachi maxell") == 0) {
            _pec_supported = false;
            _pec_confirmed = true;
            return true;
        }
    }
    */

    // assume all other batteries support PEC
	_pec_supported = true;
	_pec_confirmed = true;
	return true;
}

uint16_t AP_BattMonitor_SMBus_Maxell::string_to_data(char* startp, char* endp, uint8_t len)
{
	uint16_t result = 0;
        uint8_t count = 0;
	if(startp==endp) return *startp-'0';
	else if(startp<endp)
	{
		while(endp>=startp&&count<len)
		{
			result=result*10+(*startp-'0');
			++startp;
			++count;
		}
	}else{
		while(startp>=endp&&count<len)
		{
			result=result*10+(*startp-'0');
			--startp;
			++count;
		}
	}
	return result;
}

bool AP_BattMonitor_SMBus_Maxell::get_battery_status(void)
{

	if( SMART_BATTERY_ALL_STATUS==_battery_status ) return true;

	union {
		uint32_t data;
		uint8_t buff[4];
	}x;

	// get pf alert
    if(!(_battery_status&SMART_BATTERY_PF_ALERT))
    {
    	if(read_block(BATTMONITOR_SMBUS_MAXELL_PF_ALERT, (uint8_t*)x.buff, false)>0)
    	{
    		_battery_status = _battery_status | SMART_BATTERY_PF_ALERT;
    		_state.pf_alert = x.data;
    	}
    }

    // get safe alert
    if(!(_battery_status&SMART_BATTERY_SAFE_ALERT))
    {
    	if(read_block(BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT, (uint8_t*)x.buff, false)>0)
		{
			_battery_status = _battery_status | SMART_BATTERY_SAFE_ALERT;
			_state.safe_alert = x.data;
		}
    }

    // get operation status
    if(!(_battery_status&SMART_BATTERY_OPERATION_STATUS))
	{
    	if(read_block(BATTMONITOR_SMBUS_MAXELL_OPERATION_STATUS, (uint8_t*)x.buff, false)>0)
		{
			_battery_status = _battery_status | SMART_BATTERY_OPERATION_STATUS;
			_state.operation_status = x.data;
		}
	}

    // get charging status
    if(!(_battery_status&SMART_BATTERY_CHARGING_STATUS))
	{
    	if(read_block(BATTMONITOR_SMBUS_MAXELL_CHARGING_STATUS, (uint8_t*)x.buff, false)>0)
		{
			_battery_status = _battery_status | SMART_BATTERY_CHARGING_STATUS;
			_state.charging_status = (uint16_t)(x.buff[0]+(x.buff[1]<<8));
		}
	}

    // get guaing status
    if(!(_battery_status&SMART_BATTERY_GUAING_STATUS))
	{
    	if(read_block(BATTMONITOR_SMBUS_MAXELL_GAUGING_STATUS, (uint8_t*)x.buff, false)>0)
		{
			_battery_status = _battery_status | SMART_BATTERY_GUAING_STATUS;
			_state.guaing_status = (uint16_t)(x.buff[0]+(x.buff[1]<<8));
		}
	}
     return false;

}

