/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  GPS driver backend class
 */
#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_GPS.h"

#define MAXLEAPS 64
static const double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */

static double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
    {2017,1,1,0,0,0,-18},
    {2015,7,1,0,0,0,-17},
    {2012,7,1,0,0,0,-16},
    {2009,1,1,0,0,0,-15},
    {2006,1,1,0,0,0,-14},
    {1999,1,1,0,0,0,-13},
    {1997,7,1,0,0,0,-12},
    {1996,1,1,0,0,0,-11},
    {1994,7,1,0,0,0,-10},
    {1993,7,1,0,0,0, -9},
    {1992,7,1,0,0,0, -8},
    {1991,1,1,0,0,0, -7},
    {1990,1,1,0,0,0, -6},
    {1988,1,1,0,0,0, -5},
    {1985,7,1,0,0,0, -4},
    {1983,7,1,0,0,0, -3},
    {1982,7,1,0,0,0, -2},
    {1981,7,1,0,0,0, -1},
    {0}
};

class AP_GPS_Backend
{
public:

    typedef struct {        /* time struct */
        time_t time;        /* time (s) expressed by standard time_t */
        __attribute__ ((aligned (8)))double sec; /* fraction of second under 1 s */
    } gtime_t;

    AP_GPS_Backend(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // we declare a virtual destructor so that GPS drivers can
    // override with a custom destructor if need be.
    virtual ~AP_GPS_Backend(void) {}

    // The read() method is the only one needed in each driver. It
    // should return true when the backend has successfully received a
    // valid packet from the GPS.
    virtual bool read() = 0;

    // Highest status supported by this GPS. 
    // Allows external system to identify type of receiver connected.
    virtual AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D; }

    virtual bool is_configured(void) { return true; }

    virtual void inject_data(const uint8_t *data, uint16_t len);

    //MAVLink methods
    virtual bool supports_mavlink_gps_rtk_message() { return false; }
    virtual void send_mavlink_gps_rtk(mavlink_channel_t chan);

    virtual void broadcast_configuration_failure_reason(void) const { return ; }

    virtual void handle_msg(const mavlink_message_t *msg) { return ; }
    virtual void handle_gnss_msg(const AP_GPS::GPS_State &msg) { return ; }

    // driver specific lag, returns true if the driver is confident in the provided lag
    virtual bool get_lag(float &lag) const { lag = 0.2f; return true; }

    // driver specific health, returns true if the driver is healthy
    virtual bool is_healthy(void) const { return true; }

    virtual const char *name() const = 0;

    void broadcast_gps_type() const;
    virtual void Write_DataFlash_Log_Startup_messages() const;

    virtual bool prepare_for_arming(void) { return true; }

protected:
    AP_HAL::UARTDriver *port;           ///< UART we are attached to
    AP_GPS &gps;                        ///< access to frontend (for parameters)
    AP_GPS::GPS_State &state;           ///< public state for this instance

    // common utility functions
    int32_t swap_int32(int32_t v) const;
    int16_t swap_int16(int16_t v) const;

    /*
      fill in 3D velocity from 2D components
     */
    void fill_3d_velocity(void);

    /*
       fill in time_week_ms and time_week from BCD date and time components
       assumes MTK19 millisecond form of bcd_time
    */
    void make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds);

    void _detection_message(char *buffer, uint8_t buflen) const;

    bool should_df_log() const;

    extern gtime_t timeadd(gtime_t t, double sec);

    extern double timediff(gtime_t t1, gtime_t t2);

    void time2epoch(gtime_t t, double *ep);

    gtime_t epoch2time(const double *ep);

    gtime_t gpst2utc(gtime_t t);

    gtime_t utc2gpst(gtime_t t);

    double time2gpst(gtime_t t, int32_t *week);

    void make_time(int32_t date, int32_t time);

};
