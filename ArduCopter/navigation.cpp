#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);

    flightmode->update_navigation();

    //�������к����¼���ϵ���������д�ڸò���
    wp_continue_fly();
}

// distance between vehicle and home in cm
uint32_t Copter::home_distance()
{
    if (position_ok()) {
        const Vector3f home = pv_location_to_vector(ahrs.get_home());
        const Vector3f curr = inertial_nav.get_position();
        _home_distance = get_horizontal_distance_cm(curr, home);
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    if (position_ok()) {
        const Vector3f home = pv_location_to_vector(ahrs.get_home());
        const Vector3f curr = inertial_nav.get_position();
        _home_bearing = get_bearing_cd(curr,home);
    }
    return _home_bearing;
}

void Copter::wp_continue_fly()
{
	static bool wp_continue_start = false;
	static bool wp_continue_complete = false;
	static Location break_wp_pos;   //���������ж�λ��
	static float old_ground_speed = 0.0;
	static uint32_t pos_last_time_ms = 0;
	static uint8_t current_cmd_index = 0;
	static AP_Mission::Mission_Command  mission_cmd[2] = { {}, {} };
	static AP_Mission::Mission_Command mission_add_cmd[3];
	static AP_Mission::Mission_Command old_nav_cmd;
	static bool auto_continue_success = false;

	uint32_t pos_time_ms = 0;

	if(!mission.wp_continue_is_open()) return;   //û�д򿪶ϵ���������  �û���ѡ��򿪶ϵ����ɹ���

	//��������ģʽ������ִ��RTL����LANDָ���ȡ���ϵ��������м�¼����
	if(flightmode==&copter.mode_auto){
		if(mode_auto.mode()==Auto_RTL || mode_auto.mode()==Auto_Land){
			wp_continue_start = false;
			wp_continue_complete = false;
			auto_continue_success = false;
		}
	}

	//���������жϣ���¼�ж�λ��
	if(!wp_continue_complete && wp_continue_start && mission.state()==MISSION_STOPPED){
		break_wp_pos = copter.current_loc;
		wp_continue_complete = true;
		gcs().send_text(MAV_SEVERITY_INFO, "Break Auto! Record Pos!");
	}

	if(mission.state()==MISSION_RUNNING){  //�����������ڽ���

		//�ж��Ƿ�ʼ����ϵ��¼
		if(!wp_continue_start){
			AP_Mission::Mission_Command nav_cmd1 = mission.wp_continue_nav_cmd_complete();
			if(nav_cmd1.id==MAV_CMD_NAV_WAYPOINT){
				wp_continue_start = true;
				old_ground_speed = ahrs.groundspeed()*100;     //��ȡ��ǰ����   cm/s
				pos_last_time_ms = AP_HAL::millis();
				gcs().send_text(MAV_SEVERITY_INFO, "Start Record Waypoint");
			}
		}

		//ִ�к���ָ���ʼ������ʱ�����Ҽ�¼λ��
		if(wp_continue_start){
			float groundSpeed = ahrs.groundspeed()*100;    //��ȡ��ǰ����   cm/s
			float dist = wp_continue_pos_distance();              //�û��趨�Ķϵ�������ǰѡ�����
			dist = dist<=10.0 ? 10.0 : dist;
			dist = dist*100.0;
			if(groundSpeed<=100) pos_time_ms = dist/100;
			else pos_time_ms = dist / (groundSpeed*0.6+old_ground_speed*0.4);
			old_ground_speed = groundSpeed;

			if(AP_HAL::millis()-pos_last_time_ms>=pos_time_ms){ //��¼һ������λ��
				current_cmd_index %= 2;
				mission_cmd[current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
				mission_cmd[current_cmd_index].content.location = copter.current_loc;  //��¼��ǰλ��
				++current_cmd_index;
				pos_last_time_ms = AP_HAL::millis();
			}

			AP_Mission::Mission_Command nav_cmd1 = copter.mission.wp_continue_nav_cmd_complete();
			AP_Mission::Mission_Command do_cmd1 = copter.mission.wp_continue_do_cmd_complete();
			if(old_nav_cmd.index!= nav_cmd1.index && nav_cmd1.id==MAV_CMD_NAV_WAYPOINT){
				old_nav_cmd = nav_cmd1;
			}

			if(do_cmd1.id==MAV_CMD_DO_CHANGE_SPEED){
				if(do_cmd1.index!=mission_add_cmd[0].index) mission_add_cmd[0] = do_cmd1;
			}else if(do_cmd1.id==MAV_CMD_DO_SET_CAM_TRIGG_DIST){
				if(do_cmd1.index!=mission_add_cmd[1].index)  mission_add_cmd[1] = do_cmd1;
			}else if(do_cmd1.id==MAV_CMD_NAV_TAKEOFF){   //TAKE OFFִ����ϣ�ѡ���Զ�����
				if(copter.mission.wp_continue_cmd_total()==copter.mission.num_commands() && copter.mission.wp_continue_nav_cmd_index()>0 && copter.mission.wp_continue_cmd_total()>1){  //����û�б�����޸Ĺ�����ѡ��ϵ�����
					if(!auto_continue_success && set_current_cmd(copter.mission.wp_continue_nav_cmd_index())){
						auto_continue_success = true;
						gcs().send_text(MAV_SEVERITY_INFO, "Auto Switch to WP %d", copter.mission.wp_continue_nav_cmd_index());
					}
				}
			}
		}
	  }

	//������ɲ����п������ϵ�����,�������к���,�ڼ�����ʱ�����
   if(!motors->armed() && wp_continue_complete && ap.land_complete && copter.mission.state()!=MISSION_COMPLETE){

			//ѡ���һ����¼�ĵ�
			if(mission_cmd[current_cmd_index].id!=MAV_CMD_NAV_WAYPOINT){
				current_cmd_index = (current_cmd_index==0?1:0);
			}

			if(mission_cmd[current_cmd_index].id==MAV_CMD_NAV_WAYPOINT){

				//��������ĺ����
				const Vector3f wpo = pv_location_to_vector(old_nav_cmd.content.location); //����ɹ���һ������
				const Vector3f stopwp = pv_location_to_vector(break_wp_pos); //ֹͣ��������ʱ��λ��
				Vector3f insertwp;

				insertwp = pv_location_to_vector(mission_cmd[current_cmd_index].content.location);  //�ȴ�����ĺ���

				float wpo_curr = get_horizontal_distance_cm(stopwp, wpo);
				float insertwp_curr = get_horizontal_distance_cm(stopwp, insertwp);

				if(wpo_curr-insertwp_curr>=0.001){ //�˳�����ģʽλ����ǰһ����ľ�����ڼ�¼����ľ��룬�򽫲��뺽�㵽֮ǰ����֮��
					copter.mission.wp_continue_set_nav_cmd_index(old_nav_cmd.index+1);
				}else if(insertwp_curr-wpo_curr>=0.001){ //�˳�����ģʽλ����ǰһ����ľ���С�ڼ�¼����ľ��룬�򽫲��뺽�㵽֮ǰ����֮ǰ
					copter.mission.wp_continue_set_nav_cmd_index(old_nav_cmd.index);
				}

				mission_cmd[current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
				mission_cmd[current_cmd_index].index = copter.mission.wp_continue_nav_cmd_index();
				mission_cmd[current_cmd_index].p1 = old_nav_cmd.p1;

			    //��ʼ���򺽵�,��ʱ������Ч
				if(copter.mission.wp_continue_reset_wp(copter.mission.wp_continue_nav_cmd_index(), mission_cmd[current_cmd_index])){
					gcs().send_text(MAV_SEVERITY_INFO, "Reset WP Success!");
					copter.mission.wp_continue_stop();
					wp_continue_start = false;
					wp_continue_complete = false;
				}
			}
	}
}
