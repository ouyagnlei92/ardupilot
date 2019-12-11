/*
 * smart_auto_rtl.cpp
 *
 *  Created on: 2019��12��8��
 *      Author: JoytonAwesome
 */

#include "smart_auto_rtl.h"


SmartAutoRTL::SmartAutoRTL( Copter* copter,  AP_BattMonitor* battery){
	this->_pCopter = copter;
	this->_pBattery = battery;
	_mv_count = 0;
	_fly_status = SmartAutoRTL::DISARMING;
	_system_init = false;
	_old_mode = STABILIZE;
}

/* 100ms����һ�� */
void SmartAutoRTL::update(void){

	if(_pCopter->ap.initialised&&!_system_init){
		_system_init = true;
		_battery_type = _pCopter->g2.bat_type;
	}

	if(_pCopter->control_mode==RTL || _old_mode==RTL || !_pCopter->position_ok()) return;

	if(!_pCopter->flightmode->requires_GPS()) return;     //��ǰ����ģʽ����ҪGPS

	if(_pCopter->mode_auto.mode()==Auto_RTL || _pCopter->mode_auto.mode()==Auto_Land) return;


	if(_pCopter->control_mode!=_old_mode) _old_mode=_pCopter->control_mode;

	if(_pCopter->g2.bat_auto_rtl){    //�����Զ�����ص���
		if(_battery_type==0){  //0-Lipo���
			LiPoBatteryAutoRTL();

			++_log_record_time;
			if(_log_record_time==5){
				_log_record_time = 0;
				writeLog();
			}
		}else if(_battery_type==1){  //1-���ܵ��
			smartBatteryAutoRTL();

			++_log_record_time;
			if(_log_record_time==5){ //500ms��¼һ��log
				_log_record_time = 0;
				writeLog();
			}
		}
	}
}


void SmartAutoRTL::init(void){
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

void SmartAutoRTL::climbUseMahCal(void){   //�������
	if(!_climb_start){   //ֻ���������߶Ⱥĵ���
		if(!_climbing){
			_climbing = true;
			_old_climb_time = AP_HAL::millis();
		}
		if(_climbing && (AP_HAL::millis()-_old_climb_time)>=500){  //����500ms����
			_climb_start = true;  //������ʼ
			_old_use_mah = _pBattery->consumed_mah();  //��ȡ�Ѿ����ĵ�����
			_fly_status = SmartAutoRTL::CLIMB;
			_old_pos_ms = AP_HAL::millis();

			//���������־
			_hormove_start = false;
			_horMoving = false;
			_stopping = false;
			_stop_start = false;
			_horclimbe_start = false;
			_horclimbing = false;
			gcs().send_text(MAV_SEVERITY_WARNING, "Start Climb! Used %.1fmah", _old_use_mah);
		}
	}

	if(_climb_start && AP_HAL::millis()-_old_pos_ms>=1000){
		float cu =  _pBattery->consumed_mah();
    	if(!_up_flag){ //��������
    		_verUseMah -= cu-_old_use_mah;
    	}else if(_up_flag){ //�����У��������ĵ��ܵ���
    		_verUseMah += cu-_old_use_mah;    //��������ʹ�õĵ���
    	}
		_old_use_mah = cu;
		_old_pos_ms = AP_HAL::millis();
	}
}

void SmartAutoRTL::horUserMahCal(void){    //ˮƽ���м��
	if(!_hormove_start){
		if(!_horMoving){
			_horMoving = true;
			_old_hormove_time = AP_HAL::millis();
		}
		if(_horMoving && (AP_HAL::millis()-_old_hormove_time)>=500){  //����500ms����
			_hormove_start = true;  //������ʼ
			_old_use_mah = _pBattery->consumed_mah();  //��ȡ��ǰʹ������
			_old_pos = _pCopter->inertial_nav.get_position();   //��ȡ��ʼλ��
			_old_pos_ms = AP_HAL::millis();
			_fly_status = SmartAutoRTL::HOR_MOVING;

			_climb_start = false;
			_climbing = false;
			_stopping = false;
			_stop_start = false;
			_horclimbe_start = false;
		    _horclimbing = false;
		    _hor_count = 0;
		    for(uint8_t i=0; i<10; ++i) _hor_mah_speed[i]=0.0;
		    gcs().send_text(MAV_SEVERITY_WARNING, "Hor Moveing! Used %.1fmah", _old_use_mah);
		}
	}

	if(_hormove_start){
		if(AP_HAL::millis()-_old_pos_ms>=1000 && _pCopter->position_ok() && _pCopter->control_mode!=RTL){   //1S����һ��ƽ��ֵ
    		Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = _pCopter->inertial_nav.get_position();
			distance = _pCopter->get_horizontal_distance_cm(curr_pos, _old_pos); /* ����λ�� */
			_old_pos = curr_pos;

    		float curr_mah = _pBattery->consumed_mah(); /* ��ǰ���ĵ��� */
    		float mah_speed = (curr_mah-_old_use_mah)/distance;     /*mah/cm*/
    		_old_use_mah = curr_mah;

    		/* ���û���ƽ���㷨��ĵ��ٶ� mah/cm */
			if(_hor_count==10) _hor_count = 0;
			_hor_mah_speed[_hor_count] = mah_speed;
			++_hor_count;
			uint8_t i = 0;
			float sum = 0.0;
			for( ; i<10; ++i)
			{
				if(_hor_mah_speed[i]>0.001){
					sum += _hor_mah_speed[i];
				}else break;
			}
			if(i!=0) _hor_mah_speed_avr = sum / i;
			else _hor_mah_speed_avr = sum/1;

			_old_pos_ms = AP_HAL::millis();
		}
	}
}

void SmartAutoRTL::horClimbUseMahCal(void){   //��������ˮƽ���м��
	if(!_horclimbe_start){   //ֻ���������߶Ⱥĵ���
		if(!_horclimbing){
			_horclimbing = true;
			_old_horclimb_time = AP_HAL::millis();
		}
		if(_horclimbing && (AP_HAL::millis()-_old_horclimb_time)>=500){  //����200ms����
			_horclimbe_start = true;  //������ʼ
			_old_use_mah = _pBattery->consumed_mah();  //��ȡ��ǰ����
			_old_pos = _pCopter->inertial_nav.get_position();   //��ȡ��ʼλ��
			_old_pos_ms = AP_HAL::millis();
			_old_alt = _pCopter->barometer.get_altitude()*100;
			_fly_status = SmartAutoRTL::HOR_CLIMB;

			//���������־
			_hormove_start = false;
			_horMoving = false;
			_stopping = false;
			_stop_start = false;
			_climb_start = false;
			_climbing = false;
			_hor_count = 0;
			for(uint8_t i=0; i<10; ++i) _hor_mah_speed[i]=0.0;
			gcs().send_text(MAV_SEVERITY_WARNING, "Hor/Ver moving! Used %.1fmah", _old_use_mah);
		}
	}

	if(_horclimbe_start){
    	if(AP_HAL::millis()-_old_pos_ms>=1000 && _pCopter->position_ok() && _pCopter->control_mode!=RTL){   //1S����һ��ƽ��ֵ
			Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = _pCopter->inertial_nav.get_position();
			distance = _pCopter->pv_get_horizontal_distance_cm(curr_pos, _old_pos); /* ����λ�� */
			_old_pos = curr_pos;

			float curr_alt = _pCopter->barometer.get_altitude()*100;
			float curr_mah = _pBattery->consumed_mah(); /* ��ǰ���ĵ��� */
			float diffalt = curr_alt-_old_alt;
			float mah_speed = (curr_mah-_old_use_mah)/(distance+fabs(diffalt));     /*mah/cm*/
			_old_use_mah = curr_mah;
			_old_alt = curr_alt;

			/* ���û���ƽ���㷨��ĵ��ٶ� mah/cm */
			if(_hor_count==10) _hor_count = 0;
			_hor_mah_speed[_hor_count] = mah_speed;
			++_hor_count;
			uint8_t i = 0;
			float sum = 0.0;
			for( ; i<10; ++i)
			{
				if(_hor_mah_speed[i]>0.001){
					sum += _hor_mah_speed[i];
				}else break;
			}
			if(i!=0) _hor_mah_speed_avr = sum / i;
			else _hor_mah_speed_avr = sum/1;
			_old_pos_ms = AP_HAL::millis();

			if(diffalt>0.0001)  //��Ը߶ȴ���0����ζ������
				_verUseMah += fabs(diffalt)*_hor_mah_speed_avr*0.6;   //�߶����ĵ���
			else
				_verUseMah -= fabs(diffalt)*_hor_mah_speed_avr*0.6;

			_hor_mah_speed_avr *= 0.4;
			_old_pos_ms = AP_HAL::millis();
		}
	}
}

void SmartAutoRTL::stopUseMahCal(void){   //��ͣ���
	if(!_stop_start){
		if(!_stopping){
			_stopping = true;
			_old_stop_time = AP_HAL::millis();
		}
		if(_stopping && (AP_HAL::millis()-_old_stop_time)>=500){
			_stop_start = true;  //��ͣ״̬
			_fly_status = SmartAutoRTL::STOP;

			_hormove_start = false;
			_horMoving = false;
			_climb_start = false;
			_climbing = false;
			_horclimbe_start = false;
			_horclimbing = false;

			gcs().send_text(MAV_SEVERITY_WARNING, "Stop Move! Used %.1fmah", _pBattery->consumed_mah());

		}
	}
}

void SmartAutoRTL::lowPowerRTL(void){
	_to_home_distance = _pCopter->home_distance(); /* ��ǰλ����ؼҵ�λ�þ��� */

	_to_home_mah = _to_home_distance*_hor_mah_speed_avr + _verUseMah + _verUseMah*0.10;      /* �����Ե�ǰ״̬�ĺĵ������ص��ؼҵ���Ҫ�ĵ��� */

	/* �ж������Ƿ��ܷ��أ���������10%�� �������½������� �½�������΢����������������ȡ10% */
	if(_to_home_mah>=(_pre_arm_mah-_pBattery->pack_capacity_mah()*_pCopter->g2.bat_auto_rtl_keep_cap-_pBattery->consumed_mah()))	{
		gcs().send_text(MAV_SEVERITY_INFO, "BATTERY FAILSAFE RTL! Used %.1fmah", _pBattery->consumed_mah());
		_pCopter->set_mode(RTL, MODE_REASON_BATTERY_FAILSAFE);
		return;
	}
}


/*    �������ܵ�صķ���
 * �����������:  1��ֻ���߻��䣬��ˮƽ�ƶ�    2��ֻˮƽ�ƶ���������     3����ˮƽ�ƶ�������     4����ͣ
 */
void SmartAutoRTL::smartBatteryAutoRTL(void){

	//����ؼ�����Ƿ���ڣ��Զ����������Ƿ���
	if(!_pBattery->has_current()) return;

	//����Ƿ�������ܵ�ؼ��
	if(_pBattery->get_type()!=AP_BattMonitor_Params::BattMonitor_TYPE_MAXELL) return;

	//�鿴����״̬�������������е�ؼ�س����Զ�����������⴦��
	if(_pCopter->arming.is_armed()){
		if(!_armed){
			init();
			_pre_arm_mah = _pBattery->remaining_mah();   //��ȡ����ʱ���ʣ�N����
			_armed = true;
			gcs().send_text(MAV_SEVERITY_WARNING, "Armed! Smart Battery Remaining %.1fmah, type:%d", _pre_arm_mah, _battery_type);
		}
		//�������
		float climb_rate = _pCopter->inertial_nav.get_velocity_z(); //��ȡ�����ٶ� cm/s
		float groundSpeed = _pCopter->ahrs.groundspeed()*100;    //��ȡ��ǰ����   cm/s

	    _up_flag = climb_rate>=0 ? true:false;				     //��������  trueΪ����

	    if(fabs(climb_rate)>=30 && groundSpeed<=30){    //�ɻ�ֻ��ֱ�ƶ�����ˮƽ�ƶ�
	    	climbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed>=30){   //�ɻ�ֻˮƽ�ƶ�
	    	horUserMahCal();
	    }else if(fabs(climb_rate)>=30 && groundSpeed>=30) { //�ɻ���ˮƽ�ƶ��ִ�ֱ�ƶ�
	    	horClimbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed<30){    //��ͣ״̬
	    	stopUseMahCal();
	    }

		//�����Զ���������
	    lowPowerRTL();
	}else {
		_fly_status = SmartAutoRTL::DISARMING;
		_armed = false;
	}

}

/*   18650�����Զ�����
 * �����������:  1��ֻ���߻��䣬��ˮƽ�ƶ�    2��ֻˮƽ�ƶ���������     3����ˮƽ�ƶ�������     4����ͣ
 */
void SmartAutoRTL::LiPoBatteryAutoRTL(void){

	//����ؼ�����Ƿ���ڣ��Զ����������Ƿ���
	if(!_pBattery->has_current()) return;

	//����Ƿ�������ܵ�ؼ��
	if(_pCopter->_battery_type!=0) return;

	//�鿴����״̬�������������е�ؼ�س����Զ�����������⴦��
	if(_pCopter->arming.is_armed()){
		if(!_mv_success) /* ����Ƿ����  */
		{
			_mv[_mv_count] = (unsigned short)(_pBattery->voltage()*1000.0/(_pCopter->g2.bat_cell));
			++_mv_count;
			if(5==_mv_count) /* ���������� */
			{
				_mv_success = true;
				float sum=0.0;
				float avr = 0.0;
				unsigned char i=0;
				for(i=0; i<5; ++i)
				{
					sum += _mv[i];
				}
				avr = (sum/5.0);

				/* ���㵱ǰ��ѹ����������Χ�� �����Խ��������  */
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
				if(0==index2 && 0==index1)
				{
						/*û�е�*/
					_pre_arm_mah = 0;
					gcs().send_text(MAV_SEVERITY_INFO, "No Power");
				}else if((sizeof(vol)/sizeof(unsigned short)-1)==index1)
				{
					_pre_arm_mah = _pBattery->pack_capacity_mah();
					gcs().send_text(MAV_SEVERITY_INFO, "Power Full");
				}else
				{
					_pre_arm_mah = (((vol_mah_pre[index2]-vol_mah_pre[index1])/(vol[index2]-vol[index1]))*(avr-vol[index1]))*_pBattery->pack_capacity_mah(); /* �����ϵ�ʱ�ĵ��� */
					gcs().send_text(MAV_SEVERITY_INFO, "Armed! Lipo Remaining %.1fmah", _pre_arm_mah);
				}
				_old_mah = _pBattery->consumed_mah();
				init();
			}
		}

		//�������
		float climb_rate = _pCopter->inertial_nav.get_velocity_z(); //��ȡ�����ٶ� cm/s
		float groundSpeed = _pCopter->ahrs.groundspeed()*100;    //��ȡ��ǰ����   cm/s

	    _up_flag = climb_rate>=0 ? true:false;				     //��������  trueΪ����


	    if(fabs(climb_rate)>=30 && groundSpeed<=30){    //�ɻ�ֻ��ֱ�ƶ�����ˮƽ�ƶ�
	    	climbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed>=30){   //�ɻ�ֻˮƽ�ƶ�
	    	horUserMahCal();
	    }else if(fabs(climb_rate)>=30 && groundSpeed>=30) { //�ɻ���ˮƽ�ƶ��ִ�ֱ�ƶ�
	    	horClimbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed<30){    //��ͣ״̬
	    	stopUseMahCal();
	    }

		//�����Զ���������
	    lowPowerRTL();
	}else {
		_mv_count = 0;
		_armed = false;
		_mv_success = false;
		_fly_status = SmartAutoRTL::DISARMING;
	}

}


//��¼log
void SmartAutoRTL::writeLog(void){
	struct log_Bat_smart_rtl bat_smart{
		LOG_PACKET_HEADER_INIT(LOG_BAT_SMART_RTL),
		time_us	:	AP_HAL::micros64(),
		vol : _pBattery->voltage(),
		currentmah : _pBattery->remaining_mah(),
		flyStatus : _fly_status,
		type : _battery_type,
		vertmah : _verUseMah,
		hormahAvr : _hor_mah_speed_avr,
		returnToHomeMah : _to_home_mah,
		homeDistance : _to_home_distance,
		currentAlt : _pCopter->barometer.get_altitude(),
	};

	DataFlash_Class::instance()->WriteBlock(&bat_smart, sizeof(bat_smart));
}

