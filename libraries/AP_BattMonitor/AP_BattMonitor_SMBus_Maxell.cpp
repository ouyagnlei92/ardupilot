#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Maxell.h"
#include <utility>
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

#define SMBUS_READ_BLOCK_MAXIMUM_TRANSFER    32   // A Block Read or Write is allowed to transfer a maximum of 36 data bytes.

#define BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT   0x17    // cycle count
#define BATTMONITOR_SMBUS_MAXELL_BATTERY_DASTATUS2     0x72
#define BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT          0x50    // safety alert
/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_MAXELL_CHARGE_STATUS         0x0d    // relative state of charge
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_STATUS        0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT   0x17    // cycle count
 * #define BATTMONITOR_SMBUS_MAXELL_DESIGN_VOLTAGE        0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_MANUFACTURE_DATE      0x1b    // manufacturer date
 * #define BATTMONITOR_SMBUS_MAXELL_SERIALNUM             0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_MAXELL_HEALTH_STATUS         0x4f    // state of health
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT          0x50    // safety alert
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_STATUS         0x50    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_ALERT              0x52    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_STATUS             0x53    // safety status
*/

// Constructor
AP_BattMonitor_SMBus_Maxell::AP_BattMonitor_SMBus_Maxell(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev))
{
	_have_safe_data=false;
}

void AP_BattMonitor_SMBus_Maxell::timer()
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

    // read cell voltages
    for (uint8_t i = 0; i < BATTMONITOR_SMBUS_MAXELL_NUM_CELLS; i++) {
        if (read_word(maxell_cell_ids[i], data)) {
            _has_cell_voltages = true;
            if(_cell_health_counter[i]>0)
            {
            	--_cell_health_counter[i];
            	if(_cell_health_counter[i]>25)
            		_cell_health_counter[i] = 0;
            }
            _state.cell_voltages.cells[i] = data;
        } else {
        	++_cell_health_counter[i];
        	if(_cell_health_counter[i]>=75)
        	{
        		_cell_health_counter[i] = 75;
        		_state.cell_voltages.cells[i] = UINT16_MAX;
        	}
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

    read_full_charge_capacity();

    // FIXME: Preform current integration if the remaining capacity can't be requested
    read_remaining_capacity();

    read_temp();

    read_serial_number();

    // read safe alert
    if(!_have_safe_data&&(AP_HAL::millis()>=60000))
    {
    	union {
    		uint32_t data;
    		char buff[4];
    	}safe;
    	if(read_block(BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT, (uint8_t*)safe.buff, false)>0)
    	{
    		_state.safe_alert=safe.data;
    		_have_safe_data=true;
            gcs().send_text(MAV_SEVERITY_WARNING, "Have Smart SafeAlert: %d",_state.safe_alert);
    	}
    }

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

}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_Maxell::read_block(uint8_t reg, uint8_t* data, bool append_zero) const
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
    const uint8_t read_size = bufflen + 1 + (_pec_supported ? 1 : 0);   /* 数据长度 + 长度标志 + PEC标志 */
    uint8_t buff[read_size];

    // read bytes
    if (!_dev->read_registers(reg, buff, read_size)) {   /* 读取块区数据      */
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
    memcpy(data, &buff[1], bufflen);   /* 不包括长度和PEC位 */

    // optionally add zero to end
    if (append_zero) {
        data[bufflen] = '\0';    /* 选择插入字符串结束标志 */
    }

    // return success
    return bufflen;
}

// check if PEC supported with the version value in SpecificationInfo() function
// returns true once PEC is confirmed as working or not working
bool AP_BattMonitor_SMBus_Maxell::check_pec_support()
{
    // exit immediately if we have already confirmed pec support
    if (_pec_confirmed) {
        return true;
    }

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

