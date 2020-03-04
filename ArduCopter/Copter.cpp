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
#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    : DataFlash(g.log_bitmask),
    flight_modes(&g.flight_mode1),
    control_mode(STABILIZE),
    scaleLongDown(1),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    inertial_nav(ahrs),
    param_loader(var_info),
    flightmode(&mode_stabilize)
{
    // init sensor error logging flags
    //���ܵ�ط�������
	_mv_count = 0;
	_mv_success = false;
	_fly_status = Copter::DISARMING;
	_system_init = false;
	_log_record_time = 0;
    _hor_count = 0;
	_open_rtl = false;
	_set_error = false;
	_armed = false;
    for(uint8_t i=0; i<10; ++i) _hor_mah_speed[i]=0.0;
	init();
}

// 100ms����һ��
void Copter::batterySmartRTLUpdate(void){

	if(ap.initialised && !_system_init){   //ȷ��ϵͳ�Ƿ��ʼ��
		_system_init = true;
		if(g.bat_auto_rtl) gcs().send_text(MAV_SEVERITY_WARNING, "Batt SmartRTL Type: %d", _battery_type);
        else return;
		_battery_type = g.bat_type;
	}

    if(g.bat_auto_rtl==0) return;

	if(flightmode==&mode_rtl || _open_rtl || mode_auto.mode()==Auto_RTL || mode_auto.mode()==Auto_Land) return;

	if(!position_ok()) return;   //��ǰģʽΪRTL ���� λ�ö�λ������

	if(!flightmode->requires_GPS()) return;     //��ǰ����ģʽ����ҪGPS

	if(_battery_type==0){  //0-Lipo��� 
		LiPoBatteryAutoRTL();
	}else if(_battery_type==1){  //1-���ܵ��
		smartBatteryAutoRTL();
	}
	
	if(_mv_success){
		//�������
		float climb_rate1 = inertial_nav.get_velocity_z(); //��ȡ�����ٶ� cm/s
		float groundSpeed = ahrs.groundspeed()*100;    //��ȡ��ǰ����   cm/s

		_up_flag = climb_rate1>=0 ? true:false;				     //��������  trueΪ����


		if(fabs(climb_rate1)>=50 && groundSpeed<=50){    //�ɻ���ֱ�ƶ�����ˮƽ�ƶ�
			climbUseMahCal();
		}else if(fabs(climb_rate1)<50 && groundSpeed>=50){   //�ɻ�ֻˮƽ�ƶ�
			horUserMahCal();
		}else if(fabs(climb_rate1)>=50 && groundSpeed>=50) { //�ɻ���ˮƽ�ƶ��ִ�ֱ�ƶ�
			horClimbUseMahCal();
		}else if(fabs(climb_rate1)<50 && groundSpeed<50){    //��ͣ״̬
			stopUseMahCal();
		}

		//�����Զ���������
		lowPowerRTL();
	}

	++_log_record_time;
	if(_log_record_time>=10){  //1000ms����һ��log
		_log_record_time = 0;
		writeLog();
	}

	if(_set_error && AP_HAL::millis()-_error_time>5000){   //��ز������ô���5������һ��
		_error_time = AP_HAL::millis();
		gcs().send_text(MAV_SEVERITY_WARNING, "Batt SmartRTL Set Error!Type:%d, Cell:%d", _battery_type, g.bat_cell);
	}
}


void Copter::init(void){
	_hormove_start = false;
	_horMoving = false;
	_stopping = false;
	_stop_start = false;
	_horclimbe_start = false;
	_horclimbing = false;
	_climb_start = false;
	_climbing = false;

	_old_alt = 0;
	_up_flag = 0;
}

void Copter::climbUseMahCal(void){   //�������

	if(!_climb_start){   //ֻ���������߶Ⱥĵ���
		if(!_climbing){
			_climbing = true;
			_old_climb_time = AP_HAL::millis();
		}
		if(_climbing && (AP_HAL::millis()-_old_climb_time)>=500){  //����500ms����
			_climb_start = true;  //������ʼ
			_old_use_mah = battery.consumed_mah();  //��ȡ�Ѿ����ĵ�����
			_fly_status = Copter::CLIMB;
			_old_pos_ms = AP_HAL::millis();

			//���������־
			_hormove_start = false;
			_horMoving = false;
			_stopping = false;
			_stop_start = false;
			_horclimbe_start = false;
			_horclimbing = false;
		}
	}

	if(_climb_start && AP_HAL::millis()-_old_pos_ms>=500){
		float cu =  battery.consumed_mah();
    	if(!_up_flag){ //��������
    		_verUseMah -= cu-_old_use_mah;
    	}else if(_up_flag){ //�����У��������ĵ��ܵ���
    		_verUseMah += cu-_old_use_mah;    //��������ʹ�õĵ���
    	}
		_old_use_mah = cu;
		_old_pos_ms = AP_HAL::millis();
	}
}

void Copter::horUserMahCal(void){    //ˮƽ���м��
	if(!_hormove_start){
		if(!_horMoving){
			_horMoving = true;
			_old_hormove_time = AP_HAL::millis();
		}
		if(_horMoving && (AP_HAL::millis()-_old_hormove_time)>=500){  //����500ms����
			_hormove_start = true;  //������ʼ
			_old_use_mah = battery.consumed_mah();  //��ȡ��ǰʹ������
			_old_pos = inertial_nav.get_position();   //��ȡ��ʼλ��
			_old_pos_ms = AP_HAL::millis();
			_fly_status = Copter::HOR_MOVING;

			_climb_start = false;
			_climbing = false;
			_stopping = false;
			_stop_start = false;
			_horclimbe_start = false;
		    _horclimbing = false;
		}
	}

	if(_hormove_start){
		if(AP_HAL::millis()-_old_pos_ms>=500 && position_ok() && control_mode!=RTL){   //1S����һ��ƽ��ֵ
    		Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = inertial_nav.get_position();
			_old_pos_ms = AP_HAL::millis();
			distance = get_horizontal_distance_cm(_old_pos, curr_pos); //����λ��
			_old_pos = curr_pos;

			if(distance<=0.0001) return; 

    		float curr_mah = battery.consumed_mah(); // ��ǰ���ĵ���
    		float mah_speed = (curr_mah-_old_use_mah)/distance;     // mah/cm
    		_old_use_mah = curr_mah;

    		// ���û���ƽ���㷨��ĵ��ٶ� mah/cm
			if(_hor_count==10) _hor_count = 0;
			_hor_mah_speed[_hor_count] = mah_speed;
			++_hor_count;
			uint8_t i = 0;
			float sum = 0.0;
			for( ; i<10; ++i)
			{
				if(_hor_mah_speed[i]>0.00001){
					sum += _hor_mah_speed[i];
				}else break;
			}
			if(i!=0) _hor_mah_speed_avr = sum / i;
			else _hor_mah_speed_avr = sum/1;
		}
	}
}

void Copter::horClimbUseMahCal(void){   //��������ˮƽ���м��
	if(!_horclimbe_start){   //ֻ���������߶Ⱥĵ���
		if(!_horclimbing){
			_horclimbing = true;
			_old_horclimb_time = AP_HAL::millis();
		}
		if(_horclimbing && (AP_HAL::millis()-_old_horclimb_time)>=500){  //����200ms����
			_horclimbe_start = true;  //������ʼ
			_old_use_mah = battery.consumed_mah();  //��ȡ��ǰ����
			_old_pos = inertial_nav.get_position();   //��ȡ��ʼλ��
			_old_alt = barometer.get_altitude()*100;
			_old_pos_ms = AP_HAL::millis();
			_fly_status = Copter::HOR_CLIMB;

			//���������־
			_hormove_start = false;
			_horMoving = false;
			_stopping = false;
			_stop_start = false;
			_climb_start = false;
			_climbing = false;
		}
	}

	if(_horclimbe_start){
    	if(AP_HAL::millis()-_old_pos_ms>=500 && position_ok() && control_mode!=RTL){   //1S����һ��ƽ��ֵ
			Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = inertial_nav.get_position();
			float curr_alt = barometer.get_altitude()*100;
			_old_pos_ms = AP_HAL::millis();
			distance = get_horizontal_distance_cm(_old_pos, curr_pos); // ����λ��
			_old_pos = curr_pos;

			float curr_mah = battery.consumed_mah(); // ��ǰ���ĵ���
			float diffalt = curr_alt-_old_alt;
			float ccc = distance+fabs(diffalt);
			if(ccc<=0.0001) return;
			float mah_speed = (curr_mah-_old_use_mah)/ccc;     //mah/cm
			_old_use_mah = curr_mah;
			_old_alt = curr_alt;

			// ���û���ƽ���㷨��ĵ��ٶ� mah/cm
			if(_hor_count==10) _hor_count = 0;
			_hor_mah_speed[_hor_count] = mah_speed;
			++_hor_count;
			uint8_t i = 0;
			float sum = 0.0;
			for( ; i<10; ++i)
			{
				if(_hor_mah_speed[i]>0.00001){
					sum += _hor_mah_speed[i];
				}else break;
			}
			if(i!=0) _hor_mah_speed_avr = sum / i;
			else _hor_mah_speed_avr = sum/1;
			_old_pos_ms = AP_HAL::millis();

			if(diffalt>0.0001)  //��Ը߶ȴ���0����ζ������
				_verUseMah += fabs(diffalt)*_hor_mah_speed_avr;   //�߶����ĵ���
			else
				_verUseMah -= fabs(diffalt)*_hor_mah_speed_avr;
		}
	}
}

void Copter::stopUseMahCal(void){   //��ͣ���
	if(!_stop_start){
		if(!_stopping){
			_stopping = true;
			_old_stop_time = AP_HAL::millis();
		}
		if(_stopping && (AP_HAL::millis()-_old_stop_time)>=500){
			_stop_start = true;  //��ͣ״̬
			_fly_status = Copter::STOP;

			_hormove_start = false;
			_horMoving = false;
			_climb_start = false;
			_climbing = false;
			_horclimbe_start = false;
			_horclimbing = false;

		}
	}
}

void Copter::lowPowerRTL(void){
	_to_home_distance = home_distance(); // ��ǰλ����ؼҵ�λ�þ���

	if(_verUseMah<0) _verUseMah = fabs(_hor_mah_speed_avr*barometer.get_altitude()*100);

	_to_home_mah = _to_home_distance*_hor_mah_speed_avr + _verUseMah + _verUseMah*0.10;      // �����Ե�ǰ״̬�ĺĵ������ص��ؼҵ���Ҫ�ĵ���

	if(mode_auto.mode()==Auto_RTL || mode_auto.mode()==Auto_Land || control_mode==RTL ) return;

	// �ж������Ƿ��ܷ��أ���������10%�� �������½������� �½�������΢����������������ȡ10%
	if( !_open_rtl && (_to_home_mah>=(_pre_arm_mah-battery.pack_capacity_mah()*g.bat_auto_rtl_keep_cap-battery.consumed_mah())) )	{
		_open_rtl = true;
		set_mode_RTL_or_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
		AP_Notify::flags.failsafe_battery = true;
		gcs().send_text(MAV_SEVERITY_INFO, "RTL! Used %.1fmah", battery.consumed_mah());
		writeLog();

		init();
		return;
	}
}


/*    �������ܵ�صķ���
 * �����������:  1��ֻ���߻��䣬��ˮƽ�ƶ�    2��ֻˮƽ�ƶ���������     3����ˮƽ�ƶ�������     4����ͣ
 */
void Copter::smartBatteryAutoRTL(void){

	//����ؼ�����Ƿ���ڣ��Զ����������Ƿ���
	if(!battery.has_current()) return;

	//����Ƿ�������ܵ�ؼ��
	if(battery.get_type()!=AP_BattMonitor_Params::BattMonitor_TYPE_MAXELL) return;

	//�鿴����״̬�������������е�ؼ�س����Զ�����������⴦��
	if(motors->armed()){
		if(!_armed){
			init();
			_pre_arm_mah = battery.remaining_mah();   //��ȡ����ʱ���ʣ�N����
			_armed = true;
			_mv_success = true;
			gcs().send_text(MAV_SEVERITY_WARNING, "Armed! Smart Battery Remaining %.1fmah, type:%d", _pre_arm_mah, _battery_type);
		}
	}else {
		if(_armed){
			_armed = false;
			_mv_count = 0;
			_mv_success = false;
			_fly_status = Copter::DISARMING;
		}
	}

}

/*   18650�����Զ�����
 * �����������:  1��ֻ���߻��䣬��ˮƽ�ƶ�    2��ֻˮƽ�ƶ���������     3����ˮƽ�ƶ�������     4����ͣ
 */
void Copter::LiPoBatteryAutoRTL(void){

	//����ؼ�����Ƿ���ڣ��Զ����������Ƿ���
	if(!battery.has_current()) return;

	//����Ƿ�������ܵ�ؼ��
	if(_battery_type!=0) return;

	//�鿴����״̬�������������е�ؼ�س����Զ�����������⴦��
	if(motors->armed()){

		if(!_mv_success) // ����Ƿ����
		{
			_mv[_mv_count] = (unsigned short)((battery.voltage()*1000.0)/(g.bat_cell));
			if(_mv[_mv_count]>4600) {   //��ص�ѹ������Χ,���ô���
				_set_error = true;
				return;
			}
			++_mv_count;
			if(5==_mv_count) //����������
			{
				_mv_success = true;
				_armed = true;
				float sum=0.0;
				float avr = 0.0;
				unsigned char i=0;
				for(i=0; i<5; ++i)
				{
					sum += _mv[i];
				}
				avr = (sum/5.0);
				gcs().send_text(MAV_SEVERITY_INFO, "avr vol = %.1f", avr);

				// ���㵱ǰ��ѹ����������Χ�� �����Խ��������
				unsigned char index1=0, index2=0;
				for( i=0; i<sizeof(vol)/sizeof(unsigned short); ++i)
				{
					if(avr>=vol[i])
					{
						index1 = i;
					}else{
						index2 = i;
						break;
					}
				}
				gcs().send_text(MAV_SEVERITY_INFO, "2 = %d, 1 = %d", index2, index1);
				if(0==index2 && 0==index1)
				{
						//û�е�
					_pre_arm_mah = 0;
					gcs().send_text(MAV_SEVERITY_INFO, "No Power");
				}else if((sizeof(vol)/sizeof(unsigned short)-1)==index1)
				{
					_pre_arm_mah = battery.pack_capacity_mah();
					gcs().send_text(MAV_SEVERITY_INFO, "Power Full");
				}else
				{
					_pre_arm_mah = ( ((vol_mah_pre[index2]-vol_mah_pre[index1])/(vol[index2]-vol[index1]))*(avr-vol[index1]) + vol_mah_pre[index1] )*battery.pack_capacity_mah(); // �����ϵ�ʱ�ĵ���
					gcs().send_text(MAV_SEVERITY_WARNING, "Armed! Smart Battery Remaining %.1fmah, type:%d", _pre_arm_mah, _battery_type);
				}
				_old_mah = battery.consumed_mah();  //����֮ǰ���ĵĵ���
				init();
			}
		}
	}else {
		if(_armed){
			_mv_count = 0;
			_armed = false;
			_mv_success = false;
			_fly_status = Copter::DISARMING;
		}
	}

}


//��¼log
void Copter::writeLog(void){

	if(_set_error) return;

	struct log_Bat_smart_rtl bat_smart{
		LOG_PACKET_HEADER_INIT(LOG_BAT_SMART_RTL),
		time_us	:	AP_HAL::micros64(),
		vol : battery.voltage(),
		currentmah : battery.consumed_mah(),
		vertmah : _verUseMah,
		hormahAvr : _hor_mah_speed_avr*100,  /* m/mah */
		returnToHomeMah : _to_home_mah,
		homeDistance : _to_home_distance,
		currentAlt : barometer.get_altitude(),
		flyStatus : _fly_status,
		type : _battery_type,
	};

	DataFlash.WriteCriticalBlock(&bat_smart, sizeof(bat_smart));
}

void Copter::switchModeMessage(control_mode_t mode, mode_reason_t reason){
	uint8_t md = 0;
	if(mode==LAND) md = 8;
	else if(mode==DRIFT) md = 9;
	else if(mode<=CIRCLE) md = (uint8_t)mode;
	else if(mode>=SPORT) md = (uint8_t)(mode-3);

	gcs().send_text(MAV_SEVERITY_WARNING, "Mode:%s, Reason:%s", MODE_STRING[md], MODE_REASON[(uint8_t)reason]);

}


Copter copter;
