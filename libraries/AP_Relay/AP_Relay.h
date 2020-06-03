/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class
#pragma once

#include <AP_Param/AP_Param.h>

#define AP_RELAY_NUM_RELAYS 6

/// @class	AP_Relay
/// @brief	Class to manage the ArduPilot relay
class AP_Relay {
public:
    AP_Relay();

    /* Do not allow copies */
    AP_Relay(const AP_Relay &other) = delete;
    AP_Relay &operator=(const AP_Relay&) = delete;

    // setup the relay pin
    void        init();

    // activate the relay
    void        on(uint8_t instance) { set(instance, true); if(0==instance){  update_steering_gear_control(); }}

    // de-activate the relay
    void        off(uint8_t instance) { set(instance, false); }

    // see if the relay is enabled
    bool        enabled(uint8_t instance) { return instance < AP_RELAY_NUM_RELAYS && _pin[instance] != -1; }

    // toggle the relay status
    void        toggle(uint8_t instance);

    bool        steering_gear_is_enable(void);

    int8_t      get_steering_gear_rc_override(void) { return _steering_gear_rc_override; }

    int16_t     get_steering_gear_rc_value(void) { return _steering_gear_rc_value; }

    void        update_steering_gear_control(void);

    static AP_Relay *get_singleton(void) {return singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_Relay *singleton;

    AP_Int8 _pin[AP_RELAY_NUM_RELAYS];
    AP_Int8 _default;
    AP_Int8 _steering_gear_enable;
    AP_Int8 _steering_gear_rc_override;

    bool _steering_gear_open;
    int16_t _steering_gear_rc_value;

    void set(uint8_t instance, bool value);
};

namespace AP {
    AP_Relay *relay();
};
