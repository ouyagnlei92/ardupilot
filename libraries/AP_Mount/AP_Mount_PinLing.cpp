#include "AP_Mount_PinLing.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_HAL/utility/RingBuffer.h>

extern const AP_HAL::HAL& hal;

AP_Mount_PinLing::AP_Mount_PinLing(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _port(nullptr),
    _initialised(false),
    _last_send(0),
{}

// init - performs any required initialisation for this instance
void AP_Mount_PinLing::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_PinLing, 0);
    if (_port) {
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
    }

}

// update mount position - should be called periodically
void AP_Mount_PinLing::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    read_rc();

    set_mount_camera();

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
            /*const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);*/
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            /*const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }*/
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            /*if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
                resend_now = true;
            }*/
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    resend_now = resend_now || ((AP_HAL::millis() - _last_send) > AP_Mount_PinLing_RESEND_MS);

    if ((AP_HAL::millis() - _last_send) > AP_Mount_PinLing_RESEND_MS*2) {
    }
    if (can_send(resend_now, sizeof(PinLingLongCmd)/sizeof(uint8_t))) {
        if (resend_now) {
            send_target_angles(ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z));
            get_angles();
        } else {
            get_angles();
        }
    }
}

//set camera by serial
void AP_Mount_PinLing::set_mount_camera(void){

    uint8_t i = 0;

    if(AP_HAL::millis()-request_read_time>=2000){
        if(camera_flag.zoom_stop_flag==0 && camera_flag.zoom_down_flag==0 && camera_flag.zoom_stop_flag==0){
            //send read zoom
            for (i = 0;  i != sizeof(ZOOM_GET) ; i++) {
                _port->write(ZOOM_GET[i]);                
            }
        }

        //send read angle
        for (i = 0;  i != sizeof(READ_DATA) ; i++) {
            _port->write(READ_DATA[i]);                
        }
    }

    //auto reset
    if(camera_flag.auto_reset_flag){
        if(can_send(true, sizeof(AUTO_RESET))){
            for (i = 0;  i != sizeof(AUTO_RESET) ; i++) {
                _port->write(AUTO_RESET[i]);                
            }
            camera_flag.auto_reset_flag = 0;
        }
    }

    if(AP_HAL::millis()-camera_last_send>=1000){
            //zoom up and down
        if(camera_flag.zoom_up_falg){
            if(can_send(true, sizeof(ZOOM_UP))){
                for (i = 0;  i != sizeof(ZOOM_UP) ; i++) {
                    _port->write(ZOOM_UP[i]);                
                }
            }
        }

        if(camera_flag.zoom_down_flag){
            if(can_send(true, sizeof(ZOOM_DOWN))){
                for (i = 0;  i != sizeof(ZOOM_DOWN) ; i++) {
                    _port->write(ZOOM_DOWN[i]);                
                }
            }
        }

        if(camera_flag.zoom_stop_flag){
            camera_flag.zoom_up_falg = 0;
            camera_flag.zoom_down_flag = 0;
            if(can_send(true, sizeof(ZOOM_STOP))){
                for (i = 0;  i != sizeof(ZOOM_STOP) ; i++) {
                    _port->write(ZOOM_STOP[i]); 
                    camera_flag.zoom_stop_flag = 0;               
                }
            }
        }

        //take photo and record
        if(camera_flag.take_photo_flag){
            if(can_send(true, sizeof(TAKE_PHOTO))){
                for (i = 0;  i != sizeof(TAKE_PHOTO) ; i++) {
                    _port->write(TAKE_PHOTO[i]); 
                    camera_flag.take_photo_flag = 0;               
                }
            }
        }

        if(camera_flag.record_flag){
            if(can_send(true, sizeof(RECORD_START))){
                for (i = 0;  i != sizeof(RECORD_START) ; i++) {
                    _port->write(RECORD_START[i]); 
                    camera_flag.record_end_flag = 0;               
                }
            }
        }

        if(camera_flag.record_end_flag){
            if(can_send(true, sizeof(RECORD_STOP))){
                for (i = 0;  i != sizeof(RECORD_STOP) ; i++) {
                    _port->write(RECORD_STOP[i]); 
                    camera_flag.record_flag = 0;               
                }
            }
        }

        //switch picture and color
        if(camera_flag.color_switch_falg){
            if(can_send(true, 48)){
                for (i = 0;  i != 48 ; i++) {
                    _port->write(COLOR_SWITCH[current_color_index++%2][i]); 
                    camera_flag.color_switch_falg = 0;               
                }
            }
        }

        if(camera_flag.picture_switch_flag){
            if(can_send(true, 48)){
                for (i = 0;  i != 48 ; i++) {
                    _port->write(PIC_PIC[current_pic_index++%2][i]); 
                    camera_flag.picture_switch_flag = 0;               
                }
            }
        }

        camera_last_send = AP_HAL::millis();
    }    
}

bool AP_Mount_PinLing::read_rc(void){
#define rc_ch(i) RC_Channels::rc_channel(i-1)
    uint8_t zoom_rc_in       = _state._zoom_rc_in;        // camera zoom
    uint8_t color_rc_in      = _state._color_rc_in;       // camera color switch
    uint8_t take_photo_rc_in = _state._take_photo_rc_in;  // camera take photo
    uint8_t record_rc_in     = _state._record_rc_in;      // camera record   
    uint8_t auto_reset_rc_in = _state._auto_reset_rc_in;  // auto reset
    uint8_t auto_pic_rc_in   = _state._auto_pic_rc_in;    // auto reset

    int16_t dz_min;
    int16_t dz_max;
    int16_t rc_in;

    //zoom
    if (zoom_rc_in && (rc_ch(zoom_rc_in) != nullptr) && (rc_ch(zoom_rc_in)->get_radio_in() > 0)){
        rc_in = rc_ch(zoom_rc_in)->get_radio_in();
        dz_min = rc_ch(zoom_rc_in)->get_radio_trim() - rc_ch(zoom_rc_in)->get_dead_zone();
        dz_max = rc_ch(zoom_rc_in)->get_radio_trim() + rc_ch(zoom_rc_in)->get_dead_zone();
        if(rc_in>=dz_min && rc_in<=dz_max){
            camera_flag.zoom_down_flag = 0;
            camera_flag.zoom_up_falg = 0;
            camera_flag.zoom_stop_flag = 1;
        }else if(rc_in<dz_min){
            camera_flag.zoom_down_flag = 1; 
            camera_flag.zoom_stop_flag = 0;
        }else if(rc_in>dz_max){
            camera_flag.zoom_up_falg = 1;
            camera_flag.zoom_stop_flag = 0;
        }
    }

    //color
    if (color_rc_in && (rc_ch(color_rc_in) != nullptr) && (rc_ch(color_rc_in)->get_radio_in()>0)){
        rc_in = rc_ch(color_rc_in)->get_radio_in();
        if(old_color_rc_in>=850 && fabs(rc_in-old_color_rc_in)>=350){
            camera_flag.color_switch_falg = 1;
            old_color_rc_in = rc_in;
        }else old_color_rc_in = rc_in;
    }

    //take photo
    if (take_photo_rc_in && (rc_ch(take_photo_rc_in) != nullptr) && (rc_ch(take_photo_rc_in)->get_radio_in()>0)){
        rc_in = rc_ch(take_photo_rc_in)->get_radio_in();
        if(old_take_photo_rc_in>=850 && fabs(rc_in-old_take_photo_rc_in)>=350){
            camera_flag.take_photo_flag = 1;
            old_take_photo_rc_in = rc_in;
        }else old_take_photo_rc_in = rc_in;
    }

    //record
    if (record_rc_in && (rc_ch(record_rc_in) != nullptr) && (rc_ch(record_rc_in)->get_radio_in() > 0)){
        rc_in = rc_ch(record_rc_in)->get_radio_in();
        dz_min = rc_ch(record_rc_in)->get_radio_trim() - rc_ch(record_rc_in)->get_dead_zone();
        dz_max = rc_ch(record_rc_in)->get_radio_trim() + rc_ch(record_rc_in)->get_dead_zone();
        if(rc_in>dz_max){
            camera_flag.record_flag = 1;
        }
    }

    //auto reset
    if (auto_reset_rc_in && (rc_ch(auto_reset_rc_in) != nullptr) && (rc_ch(auto_reset_rc_in)->get_radio_in() > 0)){
        rc_in = rc_ch(auto_reset_rc_in)->get_radio_in();
        dz_min = rc_ch(auto_reset_rc_in)->get_radio_trim() - rc_ch(auto_reset_rc_in)->get_dead_zone();
        dz_max = rc_ch(auto_reset_rc_in)->get_radio_trim() + rc_ch(auto_reset_rc_in)->get_dead_zone();
        if(rc_in>dz_max){
            camera_flag.auto_reset_flag = 1;
        }
    }

    //pic switch
    if (auto_pic_rc_in && (rc_ch(auto_pic_rc_in) != nullptr) && (rc_ch(auto_pic_rc_in)->get_radio_in()>0)){
        rc_in = rc_ch(auto_pic_rc_in)->get_radio_in();
        if(auto_pic_rc_in>=850 && fabs(rc_in-old_pic_rc_in)>=350){
            camera_flag.picture_switch_flag = 1;
            old_pic_rc_in = rc_in;
        }else old_pic_rc_in = rc_in;
    }

    return true;
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_PinLing::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_PinLing::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_PinLing::status_msg(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle.y, _current_angle.x, _current_angle.z);
}

bool AP_Mount_PinLing::can_send(bool with_control, uint8_t len) {
    uint16_t required_tx = 1;
    if (with_control) {
        required_tx += len;
    }
    return (_port->txspace() >= required_tx);
}


// send_target_angles
void AP_Mount_PinLing::send_target_angles(float pitch_deg, float roll_deg, float yaw_deg)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if ((size_t)_port->txspace() < sizeof(AP_Mount_PinLing::PinLingLongCmd)) {
        return;
    }

    //header
    pin_ling_long_cmd.longCmd.headers[0] = 0xFF;
    pin_ling_long_cmd.longCmd.headers[1] = 0x01;
    pin_ling_long_cmd.longCmd.headers[2] = 0x0F;
    pin_ling_long_cmd.longCmd.headers[3] = 0x10; 

    //mode
    pin_ling_long_cmd.longCmd.roll_mode = AP_Mount_PinLing::CONTROL_MODE::MODE_NONE;
    pin_ling_long_cmd.longCmd.pitch_mode = AP_Mount_PinLing::CONTROL_MODE::MODE_REL_MOTOR_ANGLE;
    pin_ling_long_cmd.longCmd.yaw_mode = AP_Mount_PinLing::CONTROL_MODE::MODE_REL_MOTOR_ANGLE;

    // reverse pitch and yaw control
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    pin_ling_long_cmd.longCmd.pitchControl.pitch_angle = static_cast<int16_t>(pitch_deg / ANGLE_CONTROL_UNIT);
    pin_ling_long_cmd.longCmd.yawControl.yaw_angle = static_cast<int16_t>(yaw_deg / ANGLE_CONTROL_UNIT);

    pin_ling_long_cmd.longCmd.crc = 0;

    for(uint8_t j=4; j<sizeof(pin_ling_long_cmd)-1; ++j){
        pin_ling_long_cmd.longCmd.crc ^= pin_ling_long_cmd.data[j];
    }

    for (uint8_t i = 0;  i != sizeof(pin_ling_long_cmd) ; i++) {
        _port->write(pin_ling_long_cmd.data[i]);
    }

    // store time of send
    _last_send = AP_HAL::millis();
}

void AP_Mount_PinLing::get_angles() {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if (_port->txspace() < 1) {
        return;
    }

    _port->write('d');
}

void AP_Mount_PinLing::read_incoming() {
    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0 ){
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();
    }
}
