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
    //智能电池返航定义
	_mv_count = 0;
	_mv_success = false;
	_fly_status = Copter::DISARMING;
	_system_init = false;
	_old_mode = STABILIZE;
	_log_record_time = 0;
    _hor_count = 0;
	_open_rtl = false;
	_set_error = false;
	_armed = false;
    for(uint8_t i=0; i<10; ++i) _hor_mah_speed[i]=0.0;
	init();
}

// 100ms调用一次
void Copter::batterySmartRTLUpdate(void){

	if(ap.initialised && !_system_init){   //确定系统是否初始化
		_system_init = true;
		_battery_type = g.bat_type;
		gcs().send_text(MAV_SEVERITY_WARNING, "Batt SmartRTL Type: %d", _battery_type);
	}

	if(!position_ok()) return;   //当前模式为RTL 或者 位置定位不满足

	if(!flightmode->requires_GPS()) return;     //当前飞行模式不需要GPS

	if(g.bat_auto_rtl){    //开启自动检测电池电量
		if(_battery_type==0){  //0-Lipo电池
			LiPoBatteryAutoRTL();
		}else if(_battery_type==1){  //1-智能电池
			smartBatteryAutoRTL();
		}

		++_log_record_time;
		if(_log_record_time>=5){  //500ms保存一次log
			_log_record_time = 0;
			writeLog();
		}

		if(_set_error && AP_HAL::millis()-_error_time>5000){   //电池参数设置错误，5秒提醒一次
			_error_time = AP_HAL::millis();
			gcs().send_text(MAV_SEVERITY_WARNING, "Batt SmartRTL Set Error!Type:%d, Cell:%d", _battery_type, g.bat_cell);
		}
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

void Copter::climbUseMahCal(void){   //爬升检测

	if(!_climb_start){   //只计算爬升高度耗电量
		if(!_climbing){
			_climbing = true;
			_old_climb_time = AP_HAL::millis();
		}
		if(_climbing && (AP_HAL::millis()-_old_climb_time)>=500){  //持续500ms爬升
			_climb_start = true;  //爬升开始
			_old_use_mah = battery.consumed_mah();  //获取已经消耗的能量
			_fly_status = Copter::CLIMB;
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
		float cu =  battery.consumed_mah();
    	if(!_up_flag){ //不是爬升
    		_verUseMah -= cu-_old_use_mah;
    	}else if(_up_flag){ //爬升中，爬升消耗的总电量
    		_verUseMah += cu-_old_use_mah;    //更新爬升使用的电量
    	}
		_old_use_mah = cu;
		_old_pos_ms = AP_HAL::millis();
	}
}

void Copter::horUserMahCal(void){    //水平飞行检测
	if(!_hormove_start){
		if(!_horMoving){
			_horMoving = true;
			_old_hormove_time = AP_HAL::millis();
		}
		if(_horMoving && (AP_HAL::millis()-_old_hormove_time)>=500){  //持续500ms爬升
			_hormove_start = true;  //爬升开始
			_old_use_mah = battery.consumed_mah();  //获取当前使用容量
			_old_pos = inertial_nav.get_position();   //获取开始位置
			_old_pos_ms = AP_HAL::millis();
			_fly_status = Copter::HOR_MOVING;

			_climb_start = false;
			_climbing = false;
			_stopping = false;
			_stop_start = false;
			_horclimbe_start = false;
		    _horclimbing = false;
		    gcs().send_text(MAV_SEVERITY_WARNING, "Hor Moveing! Used %.1fmah", _old_use_mah);
		}
	}

	if(_hormove_start){
		if(AP_HAL::millis()-_old_pos_ms>=1000 && position_ok() && control_mode!=RTL){   //1S计算一次平均值
    		Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = inertial_nav.get_position();
			_old_pos_ms = AP_HAL::millis();
			distance = get_horizontal_distance_cm(_old_pos, curr_pos); //飞行位移
			_old_pos = curr_pos;

    		float curr_mah = battery.consumed_mah(); // 当前消耗电量
    		float mah_speed = (curr_mah-_old_use_mah)/distance;     // mah/cm
    		_old_use_mah = curr_mah;

    		// 采用滑动平均算法求耗电速度 mah/cm
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
		}
	}
}

void Copter::horClimbUseMahCal(void){   //爬升或者水平飞行检测
	if(!_horclimbe_start){   //只计算爬升高度耗电量
		if(!_horclimbing){
			_horclimbing = true;
			_old_horclimb_time = AP_HAL::millis();
		}
		if(_horclimbing && (AP_HAL::millis()-_old_horclimb_time)>=500){  //持续200ms爬升
			_horclimbe_start = true;  //爬升开始
			_old_use_mah = battery.consumed_mah();  //获取当前容量
			_old_pos = inertial_nav.get_position();   //获取开始位置
			_old_alt = barometer.get_altitude()*100;
			_old_pos_ms = AP_HAL::millis();
			_fly_status = Copter::HOR_CLIMB;

			//清楚其他标志
			_hormove_start = false;
			_horMoving = false;
			_stopping = false;
			_stop_start = false;
			_climb_start = false;
			_climbing = false;
			gcs().send_text(MAV_SEVERITY_WARNING, "Hor/Ver moving! Used %.1fmah", _old_use_mah);
		}
	}

	if(_horclimbe_start){
    	if(AP_HAL::millis()-_old_pos_ms>=1000 && position_ok() && control_mode!=RTL){   //1S计算一次平均值
			Vector3f  curr_pos;
			float distance = 0.0;

			curr_pos = inertial_nav.get_position();
			float curr_alt = barometer.get_altitude()*100;
			_old_pos_ms = AP_HAL::millis();
			distance = get_horizontal_distance_cm(_old_pos, curr_pos); // 飞行位移
			_old_pos = curr_pos;

			float curr_mah = battery.consumed_mah(); // 当前消耗电量
			float diffalt = curr_alt-_old_alt;
			float mah_speed = (curr_mah-_old_use_mah)/(distance+fabs(diffalt));     //mah/cm
			_old_use_mah = curr_mah;
			_old_alt = curr_alt;

			// 采用滑动平均算法求耗电速度 mah/cm
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
		}
	}
}

void Copter::stopUseMahCal(void){   //悬停检测
	if(!_stop_start){
		if(!_stopping){
			_stopping = true;
			_old_stop_time = AP_HAL::millis();
		}
		if(_stopping && (AP_HAL::millis()-_old_stop_time)>=500){
			_stop_start = true;  //悬停状态
			_fly_status = Copter::STOP;

			_hormove_start = false;
			_horMoving = false;
			_climb_start = false;
			_climbing = false;
			_horclimbe_start = false;
			_horclimbing = false;

			gcs().send_text(MAV_SEVERITY_WARNING, "Stop Move! Used %.1fmah", battery.consumed_mah());

		}
	}
}

void Copter::lowPowerRTL(void){
	_to_home_distance = home_distance(); // 当前位置离回家点位置距离

	_to_home_mah = _to_home_distance*_hor_mah_speed_avr + _verUseMah + _verUseMah*0.10;      // 计算以当前状态的耗电量返回到回家点需要的电量

	if(mode_auto.mode()==Auto_RTL || mode_auto.mode()==Auto_Land || control_mode==RTL ) return;

	// 判断容量是否能返回，保留电量10%， 爬升和下降电量， 下降电量稍微大于爬升电量，多取10%
	if( !_open_rtl && (_to_home_mah>=(_pre_arm_mah-battery.pack_capacity_mah()*g.bat_auto_rtl_keep_cap-battery.consumed_mah())) )	{
		_open_rtl = true;
		set_mode_RTL_or_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
		gcs().send_text(MAV_SEVERITY_INFO, "BATTERY FAILSAFE RTL! Used %.1fmah", battery.consumed_mah());
		return;
	}
}


/*    带有智能电池的返航
 * 分四种种情况:  1、只爬高或降落，不水平移动    2、只水平移动，不爬高     3、既水平移动又爬高     4、悬停
 */
void Copter::smartBatteryAutoRTL(void){

	//检测电池检测器是否存在，自动返航开关是否开启
	if(!battery.has_current()) return;

	//检测是否存在智能电池监控
	if(battery.get_type()!=AP_BattMonitor_Params::BattMonitor_TYPE_MAXELL) return;

	//查看解锁状态，解锁，则运行电池监控程序，自动返航条件检测处理
	if(arming.is_armed()){
		if(!_armed){
			init();
			_pre_arm_mah = battery.remaining_mah();   //获取解锁时候的剩餘容量
			_armed = true;
			gcs().send_text(MAV_SEVERITY_WARNING, "Armed! Smart Battery Remaining %.1fmah, type:%d", _pre_arm_mah, _battery_type);
		}
		//检测爬升
		float climb_rate = inertial_nav.get_velocity_z(); //获取爬升速度 cm/s
		float groundSpeed = ahrs.groundspeed()*100;    //获取当前地速   cm/s

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
		if(_armed){
			_armed = false;
			_mv_count = 0;
			_mv_success = false;
			_fly_status = Copter::DISARMING;
		}
	}

}

/*   18650计算自动返航
 * 分四种种情况:  1、只爬高或降落，不水平移动    2、只水平移动，不爬高     3、既水平移动又爬高     4、悬停
 */
void Copter::LiPoBatteryAutoRTL(void){

	//检测电池检测器是否存在，自动返航开关是否开启
	if(!battery.has_current()) return;

	//检测是否存在智能电池监控
	if(_battery_type!=0) return;

	//查看解锁状态，解锁，则运行电池监控程序，自动返航条件检测处理
	if(arming.is_armed()){
		if(!_mv_success) // 检测是否完成
		{
			_mv[_mv_count] = (unsigned short)((battery.voltage()*1000.0)/(g.bat_cell));
			if(_mv[_mv_count]>4600) {   //电池电压超出范围,设置错误
				_set_error = true;
				return;
			}
			++_mv_count;
			if(5==_mv_count) //采样五次完成
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

				// 计算当前电压处于容量范围， 并线性近似求电量
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
						//没有电
					_pre_arm_mah = 0;
					gcs().send_text(MAV_SEVERITY_INFO, "No Power");
				}else if((sizeof(vol)/sizeof(unsigned short)-1)==index1)
				{
					_pre_arm_mah = battery.pack_capacity_mah();
					gcs().send_text(MAV_SEVERITY_INFO, "Power Full");
				}else
				{
					_pre_arm_mah = (((vol_mah_pre[index2]-vol_mah_pre[index1])/(vol[index2]-vol[index1]))*(avr-vol[index1]))*battery.pack_capacity_mah(); // 计算上电时的电量
					gcs().send_text(MAV_SEVERITY_WARNING, "Armed! Smart Battery Remaining %.1fmah, type:%d", _pre_arm_mah, _battery_type);
				}
				_old_mah = battery.consumed_mah();  //解锁之前消耗的电量
				init();
			}
		}

		//检测爬升
		float climb_rate = inertial_nav.get_velocity_z(); //获取爬升速度 cm/s
		float groundSpeed = ahrs.groundspeed()*100;    //获取当前地速   cm/s

	    _up_flag = climb_rate>=0 ? true:false;				     //爬升方向  true为向上


	    if(fabs(climb_rate)>=25 && groundSpeed<=25){    //飞机垂直移动，无水平移动
	    	climbUseMahCal();
	    }else if(fabs(climb_rate)<25 && groundSpeed>=25){   //飞机只水平移动
	    	horUserMahCal();
	    }else if(fabs(climb_rate)>=25 && groundSpeed>=25) { //飞机既水平移动又垂直移动
	    	horClimbUseMahCal();
	    }else if(fabs(climb_rate)<25 && groundSpeed<25){    //悬停状态
	    	stopUseMahCal();
	    }

		//计算自动返航条件
	    lowPowerRTL();
	}else {
		if(_armed){
			_mv_count = 0;
			_armed = false;
			_mv_success = false;
			_fly_status = Copter::DISARMING;
		}
	}

}


//记录log
void Copter::writeLog(void){

	if(_set_error) return;

	struct log_Bat_smart_rtl bat_smart{
		LOG_PACKET_HEADER_INIT(LOG_BAT_SMART_RTL),
		time_us	:	AP_HAL::micros64(),
		vol : battery.voltage(),
		currentmah : battery.remaining_mah(),
		vertmah : _verUseMah,
		hormahAvr : _hor_mah_speed_avr,
		returnToHomeMah : _to_home_mah,
		homeDistance : _to_home_distance,
		currentAlt : barometer.get_altitude(),
		flyStatus : _fly_status,
		type : _battery_type,
	};

	DataFlash.WriteCriticalBlock(&bat_smart, sizeof(bat_smart));
}

void Copter::switchModeMessage(control_mode_t mode, mode_reason_t reason){
	uint8_t md;
	if(mode==LAND) md = 8;
	else if(mode==DRIFT) md = 9;
	else if(mode<=CIRCLE) md = (uint8_t)mode;
	else if(mode>=SPORT) md = (uint8_t)(mode-3);

	gcs().send_text(MAV_SEVERITY_WARNING, "Mode:%s, Reason:%s", MODE_STRING[mode], MODE_REASON[(uint8_t)reason]);

}


Copter copter;
