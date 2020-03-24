
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"

#define AP_Mount_PinLing_RESEND_MS   1500    // resend angle targets to gimbal once per second

class AP_Mount_PinLing : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_PinLing(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

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

    virtual void set_param(float p1, float p2, float p3);

private:

    const float SPPED_CONTROL_UNIT = 0.1220740379;  //deg/s
    const float ANGLE_CONTROL_UNIT = 0.02197265625; //deg

    const uint8_t AUTO_RESET[6] = {0x3E, 0x45, 0x01, 0x46, 0x12, 0x12};

    const uint8_t ZOOM_UP[6]   = { 0x81, 0x01, 0x04, 0x07, 0x37, 0xFF};
    const uint8_t ZOOM_DOWN[6] = { 0x81, 0x01, 0x04, 0x07, 0x27, 0xFF};
    const uint8_t ZOOM_STOP[6] = { 0x81, 0x01, 0x04, 0x07, 0x00, 0xFF}; 
    const uint8_t ZOOM_GET[5]  = { 0x81, 0x09, 0x04, 0x47, 0Xff };

    const uint8_t READ_DATA[5] = { 0x3e, 0X3D, 0x00, 0x3D, 0x00 };

    const uint8_t LOOK_DOWN[6] = { 0x3E, 0x45, 0x01, 0x46, 0x11, 0x11 };

    const uint8_t RECORD_START[48] = { 0x7e,0x7e,0x44,0x00,0x00,0x7c,0x05,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0xc1};
    const uint8_t RECORD_STOP[48] =  { 0x7e,0x7e,0x44,0x00,0x00,0x7c,0x04,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0xc0};
    const uint8_t TAKE_PHOTO[48] =  {  0x7e,0x7e,0x44,0x00,0x00,0x7c,0x06,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                       0x00,0x00,0x00,0x00,0xc2};

    const uint8_t TRACK_OPEN[48]  =   { 0x7e,0x7e,0x44,0x00,0x00,0x71,0xfe,0x00,0x00,0x00,0x00,\
                                        0x01,0x00,0x2c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                        0x00,0x00,0x00,0xdc};

    const uint8_t TRACK_CLOSE[48]  =   { 0x7e,0x7e,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                         0x00,0x00,0x00,0x40};

    const uint8_t PIC_COLOR_HEADER[6] = { 0x7e,0x7e,0x44,0x00,0x00,0x78 };  

    typedef enum CONTROL_MODE{
        MODE_NONE = 0,  //none mode
        MODE_SPEED,     //speed mode
        MODE_ANGLE,     //angle mode IMU
        MODE_SPEED_ANGLE,
        MODE_RC,        //pwm mode 
        MODE_REL_MOTOR_ANGLE,  //angle mode motor
        MODE_AUTO_RETURN,      //AUTO return reset
    }CONTROL_MODE;

    typedef enum Flag {
        FLAG_OFF = 0,
        FLAG_OK = 1,
    }FLAG;

    typedef enum _parse_angle_flag{
        PARSE_ANGLE_NONE = 0,
        PARSE_ANGLE_HEADER,
        PARSE_ANGLE_ID,
        PARSE_ANGLE_DATA_LEN,
        PARSE_ANGLE_CRC,
        PARSE_DATA_CRC,
        PARSE_DATA_OK,
    }PARSE_ANGLE_FLAG;

    typedef enum _parse_zoom_flag{
        PARSE_ZOOM_NONE = 0,
        PARSE_ZOOM_HEADER,
        PARSE_ZOOM_ID,
        PARSE_ZOOM_DATA,
        PARSE_ZOOM_CRC,
        PARSE_ZOOM_OK,
    }PARSE_ZOOM_FLAG;


    typedef struct _camera_flag{
        uint8_t zoom_up_falg : 1;
        uint8_t zoom_down_flag : 1;
        uint8_t zoom_stop_flag : 1;
        uint8_t color_switch_falg : 1;  
        uint8_t take_photo_flag : 1;
        uint8_t record_flag : 1;
        uint8_t record_end_flag : 1;
        uint8_t auto_reset_flag : 1;  
        uint8_t picture_switch_flag : 1; 
        uint8_t auto_trace : 1;  
        uint8_t recording : 1;
        uint8_t auto_tracing : 1;
        uint8_t color_rc_in_trim_flag : 1;
        uint8_t look_down_flag : 1;
        uint8_t color : 4;
        uint8_t pic_pic : 4;
    }CameraFlag;

    typedef union PACKED PinLingLongCmd{
        uint8_t data[20];
        struct PACKED LongCmd{
            uint8_t headers[4];   // header 4 byte
            CONTROL_MODE roll_mode;
            CONTROL_MODE pitch_mode;
            CONTROL_MODE yaw_mode;
            int16_t roll_speed;
            int16_t rool_angle;
            int16_t pitch_speed;
            union PACKED PitchControl{
                int16_t pitch_angle;
                int8_t pitch_pwm[2];
            }pitchControl;               
            int16_t yaw_speed;
            union PACKED YawControl{
                int16_t yaw_angle;
                int8_t yaw_pwm[2];
            }yawControl;            
            uint8_t crc;
        }longCmd;
    }PinLingLongCmd;

    typedef struct PACKED mount_data{
        union PACKED roll_ang{
            uint8_t data[4];
            int32_t roll_angle;
        }roll_angle;
        union PACKED pitch_ang{
            uint8_t data[4];
            int32_t pitch_angle;
        }pitch_angle;
        union PACKED yaw_ang{
            uint8_t data[4];
            int32_t yaw_angle;
        }yaw_angle;
        uint8_t zoompqrs[4];
    }MountData;

    MountData mount_data;

    PinLingLongCmd pin_ling_long_cmd;  

    CameraFlag camera_flag;

    int16_t old_record_rc_in;       // camera color switch
    int16_t old_take_photo_rc_in;  // camera take photo
    int16_t old_tra_rc_in;
    int16_t old_auto_reset_rc_in;
    uint8_t current_pic_index;
    uint8_t current_color_index;
    uint32_t camera_last_send;
    uint32_t request_read_time;
    uint32_t read_zoom_time;
    uint32_t send_angle_control_time;

    //parse flag
    PARSE_ANGLE_FLAG parse_flag;
    uint8_t parse_data_index;
    uint32_t parse_data_crc;

    PARSE_ZOOM_FLAG parse_zoom_flag;
    uint8_t parse_zoom_index;
    uint32_t parse_zoom_src_data;
    uint32_t zoom;

    bool read_rc(void);

    void set_mount_camera(void);

    // send_target_angles
    void send_target_angles(float pitch_deg, float roll_deg, float yaw_deg);

    void send_target_rc(void);

    // send read data request
    void get_angles();

    // parse angle
    bool parse_angle(uint8_t data);

    // parse zoom
    bool parse_zoom(uint8_t data);

    bool need_send_angle();

    // read_incoming
    void read_incoming(void);

    bool can_send(bool with_control, uint8_t len);

    void set_color_pic(uint8_t type);

    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised;              // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal

    // keep the last _current_angle values
    Vector3l _current_angle;
};
