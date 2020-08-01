#pragma once

#include "AP_BattMonitor_SMBus_Generic.h"

class AP_BattMonitor_SMBus_Maxell : public AP_BattMonitor_SMBus_Generic
{
    using AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic;

private:

    // return a scaler that should be multiplied by the battery's reported capacity numbers to arrive at the actual capacity in mAh
    uint16_t get_capacity_scaler() const override { return 2; }

<<<<<<< HEAD
    //string convert the data
    uint16_t string_to_data(char* startp, char* endp, uint8_t len);

    // get smart battery more information
    bool get_battery_status(void);

    uint8_t _pec_confirmed; // count of the number of times PEC has been confirmed as working

    uint8_t _cell_health_counter[12]; // cell heath counter
    uint8_t _battery_status;//get smart battery status
=======
>>>>>>> upstream/master
};
