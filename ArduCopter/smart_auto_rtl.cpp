/*
 * smart_auto_rtl.cpp
 *
 *  Created on: 2019年12月8日
 *      Author: JoytonAwesome
 */

#include "Copter.h"

#include "smart_auto_rtl.h"

SmartAutoRTL::SmartAutoRTL(void){
	_mv_count = 0;
	_fly_status = SmartAutoRTL::DISARMING;
	_system_init = false;
	_old_mode = STABILIZE;
}

/* 100ms调用一次 */
void SmartAutoRTL::update(void){

	if(copter.ap.initialised&&!_system_init){
		_system_init = true;
		_battery_type = copter.g2.bat_type;
	}

	if(copter.control_mode==RTL || _old_mode==RTL || !copter.position_ok()) return;

	if(!copter.flightmode->requires_GPS()) return;     //当前飞行模式不需要GPS

	if(copter.mode_auto.mode()==Auto_RTL || copter.mode_auto.mode()==Auto_Land) return;


	if(copter.control_mode!=_old_mode) _old_mode=copter.control_mode;

	if(copter.g2.bat_auto_rtl){    //开启自动检测电池电量
		if(_battery_type==0){  //0-Lipo电池
			LiPoBatteryAutoRTL();

			++_log_record_time;
			if(_log_record_time==5){
				_log_record_time = 0;
				writeLog();
			}
		}else if(_battery_type==1){  //1-智能电池
			smartBatteryAutoRTL();

			++_log_record_time;
			if(_log_record_time==5){ //500ms记录一次log
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

void SmartAutoRTL::climbUseMahCal(void){   //爬升检测
	if(!_climb_start){   //只计算爬升高度耗电量
		if(!_climbing){
			_climbing = true;
			_old_climb_time = AP_HAL::millis();
		}
		if(_climbing && (AP_HAL::millis()-_old_climb_time)>=500){  //持续500ms爬升
			_climb_start = true;  //爬升开始
			_old_use_mah = copter.battery.consumed_mah();  //获取已经消耗的能量
			_fly_status = SmartAutoRTL::CLIMB;
			_old_pos_ms = AP_HAL::millis();

			//清楚其他标志
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
		float cu =  copter.battery.consumed_mah();
    	if(!_up_flag){ //不是爬升
    		_verUseMah -= cu-_old_use_mah;
    	}else if(_up_flag){ //爬升中，爬升消耗的总电量
    		_verUseMah += cu-_old_use_mah;    //更新爬升使用的电量
    	}
		_old_use_mah = cu;
		_old_pos_ms = AP_HAL::millis();
	}
}

void SmartAutoRTL::horUserMahCal(void){    //水平飞行检测
	if(!_hormove_start){
		if(!_horMoving){
			_horMoving = true;
			_old_hormove_time = AP_HAL::millis();
		}
		if(_horMoving && (AP_HAL::millis()-_old_hormove_time)>=500){  //持续500ms爬升
			_hormove_start = true;  //爬升开始
			_old_use_mah = copter.battery.consumed_mah();  //获取当前使用容量
			_old_pos = copter.inertial_nav.get_position();   //获取开始位置
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
		if(AP_HAL::millis()-_old_pos_ms>=1000 && copter.position_ok() && copter.control_mode!=RTL){   //1S计算一次平均值
    		Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = copter.inertial_nav.get_position();
			distance = get_horizontal_distance_cm(curr_pos, _old_pos); /* 飞行位移 */
			_old_pos = curr_pos;

    		float curr_mah = copter.battery.consumed_mah(); /* 当前消耗电量 */
    		float mah_speed = (curr_mah-_old_use_mah)/distance;     /*mah/cm*/
    		_old_use_mah = curr_mah;

    		/* 采用滑动平均算法求耗电速度 mah/cm */
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

void SmartAutoRTL::horClimbUseMahCal(void){   //爬升或者水平飞行检测
	if(!_horclimbe_start){   //只计算爬升高度耗电量
		if(!_horclimbing){
			_horclimbing = true;
			_old_horclimb_time = AP_HAL::millis();
		}
		if(_horclimbing && (AP_HAL::millis()-_old_horclimb_time)>=500){  //持续200ms爬升
			_horclimbe_start = true;  //爬升开始
			_old_use_mah = copter.battery.consumed_mah();  //获取当前容量
			_old_pos = copter.inertial_nav.get_position();   //获取开始位置
			_old_pos_ms = AP_HAL::millis();
			_old_alt = copter.barometer.get_altitude()*100;
			_fly_status = SmartAutoRTL::HOR_CLIMB;

			//清楚其他标志
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
    	if(AP_HAL::millis()-_old_pos_ms>=1000 && copter.position_ok() && copter.control_mode!=RTL){   //1S计算一次平均值
			Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = copter.inertial_nav.get_position();
			distance = get_horizontal_distance_cm(curr_pos, _old_pos); /* 飞行位移 */
			_old_pos = curr_pos;

			float curr_alt = copter.barometer.get_altitude()*100;
			float curr_mah = copter.battery.consumed_mah(); /* 当前消耗电量 */
			float diffalt = curr_alt-_old_alt;
			float mah_speed = (curr_mah-_old_use_mah)/(distance+fabs(diffalt));     /*mah/cm*/
			_old_use_mah = curr_mah;
			_old_alt = curr_alt;

			/* 采用滑动平均算法求耗电速度 mah/cm */
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

			if(diffalt>0.0001)  //相对高度大于0，意味着爬升
				_verUseMah += fabs(diffalt)*_hor_mah_speed_avr*0.6;   //高度消耗叠加
			else
				_verUseMah -= fabs(diffalt)*_hor_mah_speed_avr*0.6;

			_hor_mah_speed_avr *= 0.4;
			_old_pos_ms = AP_HAL::millis();
		}
	}
}

void SmartAutoRTL::stopUseMahCal(void){   //悬停检测
	if(!_stop_start){
		if(!_stopping){
			_stopping = true;
			_old_stop_time = AP_HAL::millis();
		}
		if(_stopping && (AP_HAL::millis()-_old_stop_time)>=500){
			_stop_start = true;  //悬停状态
			_fly_status = SmartAutoRTL::STOP;

			_hormove_start = false;
			_horMoving = false;
			_climb_start = false;
			_climbing = false;
			_horclimbe_start = false;
			_horclimbing = false;

			gcs().send_text(MAV_SEVERITY_WARNING, "Stop Move! Used %.1fmah", copter.battery.consumed_mah());

		}
	}
}

void SmartAutoRTL::lowPowerRTL(void){
	_to_home_distance = copter.home_distance(); /* 当前位置离回家点位置距离 */

	_to_home_mah = _to_home_distance*_hor_mah_speed_avr + _verUseMah + _verUseMah*0.10;      /* 计算以当前状态的耗电量返回到回家点需要的电量 */

	/* 判断容量是否能返回，保留电量10%， 爬升和下降电量， 下降电量稍微大于爬升电量，多取10% */
	if(_to_home_mah>=(_pre_arm_mah-copter.battery.pack_capacity_mah()*copter.g2.bat_auto_rtl_keep_cap-copter.battery.consumed_mah()))	{
		gcs().send_text(MAV_SEVERITY_INFO, "BATTERY FAILSAFE RTL! Used %.1fmah", copter.battery.consumed_mah());
		copter.set_mode(RTL, MODE_REASON_BATTERY_FAILSAFE);
		return;
	}
}


/*    带有智能电池的返航
 * 分四种种情况:  1、只爬高或降落，不水平移动    2、只水平移动，不爬高     3、既水平移动又爬高     4、悬停
 */
void SmartAutoRTL::smartBatteryAutoRTL(void){

	//检测电池检测器是否存在，自动返航开关是否开启
	if(!copter.battery.has_current()) return;

	//检测是否存在智能电池监控
	if(copter.battery.get_type()!=AP_BattMonitor_Params::BattMonitor_TYPE_MAXELL) return;

	//查看解锁状态，解锁，则运行电池监控程序，自动返航条件检测处理
	if(copter.arming.is_armed()){
		if(!_armed){
			init();
			_pre_arm_mah = copter.battery.remaining_mah();   //获取解锁时候的剩餘容量
			_armed = true;
			gcs().send_text(MAV_SEVERITY_WARNING, "Armed! Smart Battery Remaining %.1fmah, type:%d", _pre_arm_mah, _battery_type);
		}
		//检测爬升
		float climb_rate = copter.inertial_nav.get_velocity_z(); //获取爬升速度 cm/s
		float groundSpeed = copter.ahrs.groundspeed()*100;    //获取当前地速   cm/s

	    _up_flag = climb_rate>=0 ? true:false;				     //爬升方向  true为向上

	    if(fabs(climb_rate)>=30 && groundSpeed<=30){    //飞机只垂直移动，无水平移动
	    	climbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed>=30){   //飞机只水平移动
	    	horUserMahCal();
	    }else if(fabs(climb_rate)>=30 && groundSpeed>=30) { //飞机既水平移动又垂直移动
	    	horClimbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed<30){    //悬停状态
	    	stopUseMahCal();
	    }

		//计算自动返航条件
	    lowPowerRTL();
	}else {
		_fly_status = SmartAutoRTL::DISARMING;
		_armed = false;
	}

}

/*   18650计算自动返航
 * 分四种种情况:  1、只爬高或降落，不水平移动    2、只水平移动，不爬高     3、既水平移动又爬高     4、悬停
 */
void SmartAutoRTL::LiPoBatteryAutoRTL(void){

	//检测电池检测器是否存在，自动返航开关是否开启
	if(!copter.battery.has_current()) return;

	//检测是否存在智能电池监控
	if(_battery_type!=0) return;

	//查看解锁状态，解锁，则运行电池监控程序，自动返航条件检测处理
	if(copter.arming.is_armed()){
		if(!_mv_success) /* 检测是否完成  */
		{
			_mv[_mv_count] = (unsigned short)(copter.battery.voltage()*1000.0/(copter.g2.bat_cell));
			++_mv_count;
			if(5==_mv_count) /* 采样五次完成 */
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

				/* 计算当前电压处于容量范围， 并线性近似求电量  */
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
						/*没有电*/
					_pre_arm_mah = 0;
					gcs().send_text(MAV_SEVERITY_INFO, "No Power");
				}else if((sizeof(vol)/sizeof(unsigned short)-1)==index1)
				{
					_pre_arm_mah = copter.battery.pack_capacity_mah();
					gcs().send_text(MAV_SEVERITY_INFO, "Power Full");
				}else
				{
					_pre_arm_mah = (((vol_mah_pre[index2]-vol_mah_pre[index1])/(vol[index2]-vol[index1]))*(avr-vol[index1]))*copter.battery.pack_capacity_mah(); /* 计算上电时的电量 */
					gcs().send_text(MAV_SEVERITY_INFO, "Armed! Lipo Remaining %.1fmah", _pre_arm_mah);
				}
				_old_mah = copter.battery.consumed_mah();
				init();
			}
		}

		//检测爬升
		float climb_rate = copter.inertial_nav.get_velocity_z(); //获取爬升速度 cm/s
		float groundSpeed = copter.ahrs.groundspeed()*100;    //获取当前地速   cm/s

	    _up_flag = climb_rate>=0 ? true:false;				     //爬升方向  true为向上


	    if(fabs(climb_rate)>=30 && groundSpeed<=30){    //飞机只垂直移动，无水平移动
	    	climbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed>=30){   //飞机只水平移动
	    	horUserMahCal();
	    }else if(fabs(climb_rate)>=30 && groundSpeed>=30) { //飞机既水平移动又垂直移动
	    	horClimbUseMahCal();
	    }else if(fabs(climb_rate)<30 && groundSpeed<30){    //悬停状态
	    	stopUseMahCal();
	    }

		//计算自动返航条件
	    lowPowerRTL();
	}else {
		_mv_count = 0;
		_armed = false;
		_mv_success = false;
		_fly_status = SmartAutoRTL::DISARMING;
	}

}


//记录log
void SmartAutoRTL::writeLog(void){
	struct log_Bat_smart_rtl bat_smart{
		LOG_PACKET_HEADER_INIT(LOG_BAT_SMART_RTL),
		time_us	:	AP_HAL::micros64(),
		vol : copter.battery.voltage(),
		currentmah : copter.battery.remaining_mah(),
		flyStatus : _fly_status,
		type : _battery_type,
		vertmah : _verUseMah,
		hormahAvr : _hor_mah_speed_avr,
		returnToHomeMah : _to_home_mah,
		homeDistance : _to_home_distance,
		currentAlt : copter.barometer.get_altitude(),
	};

	DataFlash_Class::instance()->WriteBlock(&bat_smart, sizeof(bat_smart));
}

