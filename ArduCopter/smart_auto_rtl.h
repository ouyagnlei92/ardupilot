/*
 * smart_auto_rtl.h
 *
 *  Created on: 2019年12月8日
 *      Author: JoytonAwesome
 */
#pragma once

class SmartAutoRTL{

public:
	SmartAutoRTL(void);

    	void update(void);

	void smartBatteryAutoRTL(void);

	void LiPoBatteryAutoRTL(void);

	typedef enum FLY_STATUS{
		DISARMING,
		STOP,
		HOR_MOVING,
		CLIMB,
		HOR_CLIMB,
	}FLY_STATUS;

private:

	uint8_t _battery_type;

	float _old_mah;        		 //记录上一次记录的耗电量
	float _pre_arm_mah;    		 //解锁之前消耗的电量
	float _mah_speed_avr;  		 //平均消耗速度  mah/cm

	const unsigned short vol[13] = {2500, 3500, 3680, 3700, 3730, 3770, 3790, 3820, 3870, 3930, 4000, 4060, 4200};
	const float vol_mah_pre[13] =  {0,    0.05,  0.1,  0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
	                                      /* 0%   5%   10%  15%  20%  30%  40%  50%  60%  70%  80%  90%  100%*/

	unsigned short _mv[5];      //上电采样五次电压
	uint8_t _mv_count; //上电采样五次电压次数
	bool _mv_success;  //采样完成

	bool _climbing;   //爬升中
	bool _climb_start;   //爬升开始
	bool _up_flag;    //爬升方向      true: up;  false: down
	float _old_use_mah;   //爬升开始消耗的电量

	bool _stopping;    //悬停中
	bool _stop_start; //悬停开始
	uint32_t _old_stop_time;  //悬停开始时间

	bool _horMoving;    //水平移动中
	bool _hormove_start; //水平移动开始
	uint32_t _old_hormove_time;  //水平移动开始时间

	bool _horclimbing;    //水平移动中
	bool _horclimbe_start; //水平移动开始
	uint32_t _old_horclimb_time;  //水平移动开始时间
	float _old_alt;

    float _verUseMah; //垂直方向使用的耗电量
    float _to_home_mah;
    float _to_home_distance;
    uint32_t _old_climb_time;   //开始爬升时间

    uint8_t _hor_count;
    float _hor_mah_speed[10];
    float _hor_mah_speed_avr;  //水平方向平均耗电速度   mah/cm

    Vector3f _old_pos;
    uint32_t _old_pos_ms;

    uint8_t _log_record_time;

    uint8_t _fly_status;

    bool _armed;
    bool _system_init;

    control_mode_t _old_mode;

    void init(void);
    void climbUseMahCal(void);
    void horUserMahCal(void);
    void horClimbUseMahCal(void);
    void stopUseMahCal(void);
    void lowPowerRTL(void);

    void writeLog(void);

};



