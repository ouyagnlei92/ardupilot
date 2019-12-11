/*
 * smart_auto_rtl.h
 *
 *  Created on: 2019��12��8��
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

	float _old_mah;        		 //��¼��һ�μ�¼�ĺĵ���
	float _pre_arm_mah;    		 //����֮ǰ���ĵĵ���
	float _mah_speed_avr;  		 //ƽ�������ٶ�  mah/cm

	const unsigned short vol[13] = {2500, 3500, 3680, 3700, 3730, 3770, 3790, 3820, 3870, 3930, 4000, 4060, 4200};
	const float vol_mah_pre[13] =  {0,    0.05,  0.1,  0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
	                                      /* 0%   5%   10%  15%  20%  30%  40%  50%  60%  70%  80%  90%  100%*/

	unsigned short _mv[5];      //�ϵ������ε�ѹ
	uint8_t _mv_count; //�ϵ������ε�ѹ����
	bool _mv_success;  //�������

	bool _climbing;   //������
	bool _climb_start;   //������ʼ
	bool _up_flag;    //��������      true: up;  false: down
	float _old_use_mah;   //������ʼ���ĵĵ���

	bool _stopping;    //��ͣ��
	bool _stop_start; //��ͣ��ʼ
	uint32_t _old_stop_time;  //��ͣ��ʼʱ��

	bool _horMoving;    //ˮƽ�ƶ���
	bool _hormove_start; //ˮƽ�ƶ���ʼ
	uint32_t _old_hormove_time;  //ˮƽ�ƶ���ʼʱ��

	bool _horclimbing;    //ˮƽ�ƶ���
	bool _horclimbe_start; //ˮƽ�ƶ���ʼ
	uint32_t _old_horclimb_time;  //ˮƽ�ƶ���ʼʱ��
	float _old_alt;

    float _verUseMah; //��ֱ����ʹ�õĺĵ���
    float _to_home_mah;
    float _to_home_distance;
    uint32_t _old_climb_time;   //��ʼ����ʱ��

    uint8_t _hor_count;
    float _hor_mah_speed[10];
    float _hor_mah_speed_avr;  //ˮƽ����ƽ���ĵ��ٶ�   mah/cm

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



