
#include "AP_Mount_PinLing.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define rc_ch(i) RC_Channels::rc_channel(i-1)

AP_Mount_PinLing::AP_Mount_PinLing(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _port(nullptr),
    _initialised(false),
    _last_send(0)
{

 }

// init - performs any required initialisation for this instance
void AP_Mount_PinLing::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_PinLing, 0);
    if (_port) {
        uint8_t* pp;
        int i = 0;
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
        for(i=0, pp=(uint8_t*)(&camera_flag); i<sizeof(AP_Mount_PinLing::CameraFlag); ++i) pp[i] = 0;  
        for(i=0, pp=(uint8_t*)(&pin_ling_long_cmd); i<sizeof(AP_Mount_PinLing::PinLingLongCmd); ++i) pp[i] = 0;     
        parse_flag = PARSE_ANGLE_NONE;
        parse_data_index = 0; 
        parse_data_crc = 0; 
        parse_zoom_flag = PARSE_ZOOM_NONE;
        parse_zoom_index = 0;
        old_record_rc_in = 0;       // camera color switch
        old_take_photo_rc_in = 0;  // camera take photo
        old_tra_rc_in = 0;
        old_auto_reset_rc_in = 0;
        old_look_down_rc_in = 0;
        zoom = 1;
        camera_flag.color = 1;
        camera_flag.color_switch_falg = 1;
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
        /*
            {
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }*/
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            /*{
            const Vector3f &target = _state._neutral_angles.get();
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
            //update_targets_from_rc();
            //resend_now = true;
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

    update_targets_from_rc();
    resend_now = true;

    // resend target angles at least once per second
    resend_now = resend_now || ((AP_HAL::millis() - _last_send) > AP_Mount_PinLing_RESEND_MS);

    if ((AP_HAL::millis() - _last_send) > AP_Mount_PinLing_RESEND_MS*2) {

    }
    if (can_send(resend_now, 20)) {
        if (resend_now) {
            uint8_t tilt_rc_in = _state._tilt_rc_in;
            uint8_t pan_rc_in = _state._pan_rc_in;
            
            int16_t pin = rc_ch(tilt_rc_in)->get_radio_in();
            int16_t yin = rc_ch(pan_rc_in)->get_radio_in();

            if(0==_angle_update && AP_HAL::millis()-send_angle_control_time>120 && _state._rc_mode==0){
                send_target_rc();  
                get_angles(); 

                //_angle_ef_target_rad.y = ToRad(_current_angle.y/100.0);
                //_angle_ef_target_rad.x = 0;
                //_angle_ef_target_rad.z = ToRad(_current_angle.z/100.0);
                send_angle_control_time = AP_HAL::millis();
                 _last_send = AP_HAL::millis();
            }

            if(AP_HAL::millis()-send_angle_control_time>180){
                if(_angle_update==0 && (pin<(rc_ch(tilt_rc_in)->get_radio_trim()-rc_ch(tilt_rc_in)->get_dead_zone()) || \
                    pin>(rc_ch(tilt_rc_in)->get_radio_trim()+rc_ch(tilt_rc_in)->get_dead_zone()) || \
                    yin<(rc_ch(pan_rc_in)->get_radio_trim()-rc_ch(pan_rc_in)->get_dead_zone()) || \
                    yin>(rc_ch(pan_rc_in)->get_radio_trim()+rc_ch(pan_rc_in)->get_dead_zone()))){
                        if(_state._rc_mode==1 && need_send_angle()){
                            send_target_angles(ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z));                             
                        }
                    }else if( 0!=_angle_update ){
                        send_target_angles(ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z)); 
                        --_angle_update;                        
                    }
                    _last_send = AP_HAL::millis();
                    send_angle_control_time = AP_HAL::millis();   
                    get_angles(); 
            }
        }
    }
}

//set camera by serial
void AP_Mount_PinLing::set_mount_camera(void){    

    uint8_t i = 0;

    if(AP_HAL::millis()-read_zoom_time>=2000 && (camera_flag.zoom_up_falg==1 || camera_flag.zoom_down_flag==1)){
        //send read zoom
        for (i = 0;  i != sizeof(ZOOM_GET) ; i++) {
            _port->write(ZOOM_GET[i]);                
        }
        read_zoom_time = AP_HAL::millis();
    }

    if(AP_HAL::millis()-request_read_time>=4000){
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

        //gcs().send_text(MAV_SEVERITY_INFO, "deg:%.1f %.1f %.1f",  ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z));

        request_read_time = AP_HAL::millis();
    }

    //auto reset
    if(camera_flag.auto_reset_flag){
        if(can_send(true, sizeof(AUTO_RESET))) {
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
                }
                camera_flag.zoom_stop_flag = 0;                    
            }
        }

        //take photo and record
        if(camera_flag.take_photo_flag){
            if(can_send(true, sizeof(TAKE_PHOTO))){
                for (i = 0;  i != sizeof(TAKE_PHOTO) ; i++) {
                    _port->write(TAKE_PHOTO[i]);                                  
                }
                camera_flag.take_photo_flag = 0;                  
            }
        }

        if(camera_flag.record_flag){
            if(can_send(true, sizeof(RECORD_START))){ 
                for (i = 0;  i != sizeof(RECORD_START) ; i++) {
                    _port->write(RECORD_START[i]);                             
                }
                camera_flag.record_flag = 0;
                camera_flag.recording = 1;                   
            }
        }

        if(camera_flag.record_end_flag){
            if(can_send(true, sizeof(RECORD_STOP))){
                for (i = 0;  i != sizeof(RECORD_STOP) ; i++) {
                    _port->write(RECORD_STOP[i]);                              
                }
                camera_flag.record_end_flag = 0;  
                camera_flag.recording = 0;             
            }
        }

        //switch picture and color
        if(camera_flag.color_switch_falg){
            set_color_pic(0);
        }

        if(camera_flag.picture_switch_flag){
            set_color_pic(1);
        }

        if(camera_flag.auto_trace){
            camera_flag.auto_trace = 0;
            if(camera_flag.auto_tracing){
                if(can_send(true, sizeof(TRACK_OPEN))) {
                    for (i = 0;  i != sizeof(TRACK_OPEN) ; i++) {
                        _port->write(TRACK_OPEN[i]);                
                    }         
                }
            }else{
                if(can_send(true, sizeof(TRACK_CLOSE))) {
                    for (i = 0;  i != sizeof(TRACK_CLOSE) ; i++) {
                        _port->write(TRACK_CLOSE[i]);                
                    }        
                }
            }
        }

        if(camera_flag.look_down_flag){
            if(can_send(true, sizeof(LOOK_DOWN))) {
                for (i = 0;  i != sizeof(LOOK_DOWN) ; i++) {
                    _port->write(LOOK_DOWN[i]);  
                    camera_flag.look_down_flag = 0;              
                }        
            }
        }

        camera_last_send = AP_HAL::millis();
    }    
}


bool AP_Mount_PinLing::read_rc(void){
    
    uint8_t zoom_rc_in       = _state._zoom_rc_in;        // camera zoom
    uint8_t color_rc_in      = _state._color_rc_in;       // camera color switch
    uint8_t take_photo_rc_in = _state._take_photo_rc_in;  // camera take photo
    uint8_t record_rc_in     = _state._record_rc_in;      // camera record   
    uint8_t auto_reset_rc_in = _state._auto_reset_rc_in;  // auto reset
    uint8_t auto_tra_rc_in   = _state._auto_tra_rc_in;    // auto reset
    uint8_t look_down_rc     = _state._auto_look_down_in; // look down 90deg

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

    //take photo
    if (take_photo_rc_in && (rc_ch(take_photo_rc_in) != nullptr) && (rc_ch(take_photo_rc_in)->get_radio_in()>0)){
        rc_in = rc_ch(take_photo_rc_in)->get_radio_in();        
        dz_min = rc_ch(take_photo_rc_in)->get_radio_min() + rc_ch(take_photo_rc_in)->get_dead_zone();
        if(rc_in>dz_min && rc_in-old_take_photo_rc_in>=350 && old_take_photo_rc_in>=800){
            camera_flag.take_photo_flag = 1;
        }
        old_take_photo_rc_in = rc_in;
    }

    //record
    if (record_rc_in && (rc_ch(record_rc_in) != nullptr) && (rc_ch(record_rc_in)->get_radio_in() > 0)){
        rc_in = rc_ch(record_rc_in)->get_radio_in();        
        dz_min = rc_ch(record_rc_in)->get_radio_min() + rc_ch(record_rc_in)->get_dead_zone();
        if(rc_in>dz_min && rc_in-old_record_rc_in>=350 && old_record_rc_in>=800){
            if(camera_flag.recording) camera_flag.record_end_flag = 1; 
            else camera_flag.record_flag = 1;
        }
        old_record_rc_in = rc_in;  
    }

    //auto reset
    if (auto_reset_rc_in && (rc_ch(auto_reset_rc_in) != nullptr) && (rc_ch(auto_reset_rc_in)->get_radio_in() > 0)){
        rc_in = rc_ch(auto_reset_rc_in)->get_radio_in();        
        dz_min = rc_ch(auto_reset_rc_in)->get_radio_min() + rc_ch(auto_reset_rc_in)->get_dead_zone();
        if(rc_in>dz_min && rc_in-old_auto_reset_rc_in>=350 && old_auto_reset_rc_in>=800){
            camera_flag.auto_reset_flag = 1;
        }
        old_auto_reset_rc_in = rc_in;
    }

    //color and picture switch
    if (color_rc_in && (rc_ch(color_rc_in) != nullptr) && (rc_ch(color_rc_in)->get_radio_in()>0)){

        rc_in = rc_ch(color_rc_in)->get_radio_in();
        dz_min = rc_ch(color_rc_in)->get_radio_trim() - rc_ch(color_rc_in)->get_dead_zone();
        dz_max = rc_ch(color_rc_in)->get_radio_trim() + rc_ch(color_rc_in)->get_dead_zone();
        if(rc_in>=dz_min && rc_in<=dz_max){
            camera_flag.color_rc_in_trim_flag = 1;
        }else if(rc_in<dz_min && camera_flag.color_rc_in_trim_flag){
            camera_flag.color_switch_falg = 1;
            camera_flag.color_rc_in_trim_flag = 0;
        }else if(rc_in>dz_max && camera_flag.color_rc_in_trim_flag){
            camera_flag.picture_switch_flag = 1;
             camera_flag.color_rc_in_trim_flag = 0;
        }    
    }

    //auto trance
    if (auto_tra_rc_in && (rc_ch(auto_tra_rc_in) != nullptr) && (rc_ch(auto_tra_rc_in)->get_radio_in()>0)){
        rc_in = rc_ch(auto_tra_rc_in)->get_radio_in();        
        dz_min = rc_ch(auto_tra_rc_in)->get_radio_min() + rc_ch(auto_tra_rc_in)->get_dead_zone();
        if(rc_in-dz_min>=0 && rc_in-old_tra_rc_in-350>=0 && old_tra_rc_in>=800){
            camera_flag.auto_trace = 1;
            if(camera_flag.auto_tracing) camera_flag.auto_tracing = 0;
            else camera_flag.auto_tracing = 1;
        }
        old_tra_rc_in = rc_in;
    }

    //auto look down 90deg
    if (look_down_rc && (rc_ch(look_down_rc) != nullptr) && (rc_ch(look_down_rc)->get_radio_in() > 0)){
        rc_in = rc_ch(look_down_rc)->get_radio_in();        
        dz_min = rc_ch(look_down_rc)->get_radio_min() + rc_ch(look_down_rc)->get_dead_zone();
        if(rc_in>dz_min && rc_in-old_look_down_rc_in>=350 && old_look_down_rc_in>=800){
            camera_flag.look_down_flag = 1;
        }
        old_look_down_rc_in = rc_in;
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

void AP_Mount_PinLing::send_target_rc(void){
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if ((size_t)_port->txspace() < 20) {
        return;
    }

    //header
    pin_ling_long_cmd.longCmd.headers[0] = 0xFF;
    pin_ling_long_cmd.longCmd.headers[1] = 0x01;
    pin_ling_long_cmd.longCmd.headers[2] = 0x0F;
    pin_ling_long_cmd.longCmd.headers[3] = 0x10; 

    //mode
    pin_ling_long_cmd.longCmd.roll_mode = AP_Mount_PinLing::CONTROL_MODE::MODE_NONE;
    pin_ling_long_cmd.longCmd.pitch_mode = AP_Mount_PinLing::CONTROL_MODE::MODE_RC;
    pin_ling_long_cmd.longCmd.yaw_mode = AP_Mount_PinLing::CONTROL_MODE::MODE_RC;

    uint8_t tilt_rc_in = _state._tilt_rc_in;
    uint8_t pan_rc_in = _state._pan_rc_in;
    int16_t rcin = 0;
    int16_t dz_min;
    int16_t dz_max;
  
    if (tilt_rc_in && (rc_ch(tilt_rc_in) != nullptr) && (rc_ch(tilt_rc_in)->get_radio_in() > 0)){

        rcin = rc_ch(tilt_rc_in)->get_radio_in();
        dz_min = rc_ch(tilt_rc_in)->get_radio_trim() - rc_ch(tilt_rc_in)->get_dead_zone();
        dz_max = rc_ch(tilt_rc_in)->get_radio_trim() + rc_ch(tilt_rc_in)->get_dead_zone();

        if(rcin<dz_min || rcin>dz_max){
            rcin = rcin>=1800 ? 1800 : rcin;
            pin_ling_long_cmd.longCmd.pitchControl.pitch_angle = -static_cast<int16_t>(rcin-rc_ch(tilt_rc_in)->get_radio_trim());
        }else {
            pin_ling_long_cmd.longCmd.pitchControl.pitch_angle = 0;
        }
    }
    if (pan_rc_in && (rc_ch(pan_rc_in) != nullptr) && (rc_ch(pan_rc_in)->get_radio_in() > 0)){
        rcin = rc_ch(pan_rc_in)->get_radio_in();
        dz_min = rc_ch(pan_rc_in)->get_radio_trim() - rc_ch(pan_rc_in)->get_dead_zone();
        dz_max = rc_ch(pan_rc_in)->get_radio_trim() + rc_ch(pan_rc_in)->get_dead_zone();

        if(rcin<dz_min || rcin>dz_max){
            rcin = rcin>=1800 ? 1800 : rcin;
            pin_ling_long_cmd.longCmd.yawControl.yaw_angle = static_cast<int16_t>(rcin-rc_ch(pan_rc_in)->get_radio_trim());
        }else {
            pin_ling_long_cmd.longCmd.yawControl.yaw_angle = 0;
        }
    }

    pin_ling_long_cmd.longCmd.crc = 0;

    for(uint8_t j=4; j<20-1; ++j){
        pin_ling_long_cmd.longCmd.crc += pin_ling_long_cmd.data[j];
    }

    for (uint8_t i = 0;  i < 20 ; i++) {
        _port->write(pin_ling_long_cmd.data[i]);
    }
    // store time of send
    _last_send = AP_HAL::millis();
}

// send_target_angles
void AP_Mount_PinLing::send_target_angles(float pitch_deg, float roll_deg, float yaw_deg)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if ((size_t)_port->txspace() < 20) {
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
    yaw_deg = yaw_deg;

    pin_ling_long_cmd.longCmd.pitchControl.pitch_angle = static_cast<int16_t>(pitch_deg / ANGLE_CONTROL_UNIT);
    pin_ling_long_cmd.longCmd.yawControl.yaw_angle = static_cast<int16_t>(yaw_deg / ANGLE_CONTROL_UNIT);

    pin_ling_long_cmd.longCmd.crc = 0;

    for(uint8_t j=4; j<20-1; ++j){
        pin_ling_long_cmd.longCmd.crc += pin_ling_long_cmd.data[j];
    }

    for (uint8_t i = 0;  i < 20 ; i++) {
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

    if(can_send(true, sizeof(READ_DATA))){
        //send read angle
        for (int i = 0;  i < sizeof(READ_DATA) ; i++) {
            _port->write(READ_DATA[i]);                
        }
    }    

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

        parse_angle(data);
        parse_zoom(data);
    }
}

bool AP_Mount_PinLing::parse_angle(uint8_t data){
    //3E 3D 36 73 98 FF 98 FF E6 FF FF FF 00 00 00 00 00 00 00 00 00 00 1C F7 1C F7 28 F7 FF FF 00 00 00 00 00 00 00 00 00 00 88 FF 88 FF
    //F8 F6 FF FF 00 00 00 00 00 00 00 00 00 00 4E
    if(parse_flag==PARSE_ANGLE_NONE && data==0x3E){
        parse_data_index = 0;
        parse_data_crc = 0;
        parse_flag = PARSE_ANGLE_HEADER;
    }else if(parse_flag==PARSE_ANGLE_HEADER && data==0x3D){
        parse_flag = PARSE_ANGLE_ID;
    }else if(parse_flag==PARSE_ANGLE_ID && data==0x36){
        parse_flag = PARSE_ANGLE_DATA_LEN;
    }else if(parse_flag==PARSE_ANGLE_DATA_LEN && data==0x73){
        parse_flag = PARSE_ANGLE_CRC;
    }else if(parse_flag==PARSE_ANGLE_CRC){
        ++parse_data_index;
        parse_data_crc += data;
        if(parse_data_index>=5 && parse_data_index<=8){
            mount_data.roll_angle.data[parse_data_index-5] = data;
        }else if(parse_data_index>=23 && parse_data_index<=26){
            mount_data.pitch_angle.data[parse_data_index-23] = data;
        }else if(parse_data_index>=41 && parse_data_index<=44){
            mount_data.yaw_angle.data[parse_data_index-41] = data;
        }
        if(parse_data_index==0x36){
            parse_flag = PARSE_DATA_CRC;
        }
    }else if(parse_flag==PARSE_DATA_CRC){
        if((parse_data_crc&0x0ff)==data){
                parse_flag = PARSE_DATA_OK;             
        }else{
                parse_flag = PARSE_ANGLE_NONE;
        }
    }else {
        parse_flag = PARSE_ANGLE_NONE;
    }

    if(parse_flag==PARSE_DATA_OK){
        //set angle
        _current_angle.y = static_cast<int32_t>((-mount_data.pitch_angle.pitch_angle * ANGLE_CONTROL_UNIT)*100);
        _current_angle.x = static_cast<int32_t>((mount_data.roll_angle.roll_angle * ANGLE_CONTROL_UNIT)*100);
        _current_angle.z = static_cast<int32_t>((mount_data.yaw_angle.yaw_angle * ANGLE_CONTROL_UNIT)*100);
        parse_flag = PARSE_ANGLE_NONE;
        //gcs().send_text(MAV_SEVERITY_INFO, "angle:%d %d %d",  _current_angle.y, _current_angle.z);
        return true;
    }else return false;
}

bool AP_Mount_PinLing::parse_zoom(uint8_t data){
    //90 50 0p 0q 0r 0s FF 
    if(parse_zoom_flag==PARSE_ZOOM_NONE && data==0x90){
        parse_zoom_index = 0;
        parse_zoom_src_data = 0;
        parse_zoom_flag = PARSE_ZOOM_HEADER;
    }else if(parse_zoom_flag==PARSE_ZOOM_HEADER && data==0x50){
        parse_zoom_flag = PARSE_ZOOM_ID;
    }else if(parse_zoom_flag==PARSE_ZOOM_ID && ((data&0xf0)==0)){
        ++parse_zoom_index;
        parse_zoom_src_data = (parse_zoom_src_data<<4)|data;
        if(parse_zoom_index==4){
            parse_zoom_flag = PARSE_ZOOM_CRC;
        }
    }else if(parse_zoom_flag==PARSE_ZOOM_CRC && data==0xff){
        //get zoom
        zoom = parse_zoom_src_data;
        parse_zoom_flag=PARSE_ZOOM_NONE;
        //gcs().send_text(MAV_SEVERITY_INFO, "have zoom %d", zoom);
        return true;
    }else {
        parse_zoom_flag=PARSE_ZOOM_NONE;
        return false;
    }
    return false;
}

bool AP_Mount_PinLing::need_send_angle(){

    if( fabsf(ToDeg(_angle_ef_target_rad.y)-_current_angle.y/100.0)<=0.5 && 
        fabsf(ToDeg(_angle_ef_target_rad.z)-_current_angle.z/100.0)<=0.5)
        return false;
    return true;
}

void AP_Mount_PinLing::set_color_pic(uint8_t type){  //type 0-color  1-picture
    uint8_t i = 0;
    uint32_t crc = 0;
    uint8_t tm = 0;

    if(0==type){
        ++camera_flag.color;
        camera_flag.color %= 6;
    }else if(1==type){
        ++camera_flag.pic_pic;
        camera_flag.pic_pic %= 4;
    }        

    if(can_send(true, 48)){
        for (i = 0;  i<6 ; i++) {
            _port->write(PIC_COLOR_HEADER[i]);      
            crc += PIC_COLOR_HEADER[i];                        
        } 
        if(camera_flag.color==1 || camera_flag.color==0){
            _port->write(static_cast<uint8_t>(0x00)); 
        }else {
            _port->write(camera_flag.color-1); 
            crc += (camera_flag.color-1);
        }
        for( i=0; i<40; ++i ){
            tm = 0;
            switch(i){
                case 3:
                    tm = 0x48;
                    break;
                case 4:
                    tm = 0x43;
                    break;
                case 5:
                    if(2==camera_flag.color) tm = 1;
                    else tm = 0;
                    break;
                case 7:
                    tm = camera_flag.pic_pic;
                    break;
                case 8:
                    if(1==camera_flag.color) tm = camera_flag.color;
                    else tm = 0;
                    break;
                case 10:
                    tm = 1;
                    break;
                case 11:
                    tm = 1;
                    break;
                default:
                    tm = 0;
                    break;
            }
            crc += tm;
            _port->write(static_cast<uint8_t>(tm));
        } 
        _port->write(static_cast<uint8_t>(crc&0x0ff));  

        if(0==type){
            camera_flag.color_switch_falg = 0;  
        }else if(1==type){
            camera_flag.picture_switch_flag = 0; 
        }         
    }
}

void AP_Mount_PinLing::set_param(float p1, float p2, float p3){
    _param[0] = static_cast<int32_t>(p1);
    _param[1] = static_cast<int32_t>(p2);
    _param[2] = static_cast<int32_t>(p3);

    if(_param[0]==1){
        camera_flag.look_down_flag = 1;
        _angle_ef_target_rad.y = -ToRad(90.0);
        _angle_ef_target_rad.x = 0;
        _angle_ef_target_rad.z = 0;
    }
}
