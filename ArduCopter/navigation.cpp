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
	static bool pos_record_falg = false;

	if(!copter.mission.wp_continue_is_open()) return;   //û�д򿪶ϵ���������

	//��������ģʽ������ִ��RTL����LANDָ���ȡ���ϵ��������м�¼����
	if(flightmode==&copter.mode_auto){
		if(copter.mode_auto.mode()==Auto_RTL || copter.mode_auto.mode()==Auto_Land){
			copter.mission.wp_continue_stop();
			pos_record_falg = false;
		}
	}

	//����������û��ִ��������֮��ǿ���жϣ���¼��ǰ�ж�λ��
	if(!pos_record_falg && copter.mission.wp_continue_is_end()){
		copter.mission.wp_continue_abort_pos(inertial_nav.get_position());
		gcs().send_text(MAV_SEVERITY_INFO, "Aboart Auto! Record Pos!");
		pos_record_falg = true;
	}

	//������ɲ����п������ϵ�����,�������к���,�ڼ�����ʱ�����
   if(!motors->armed() && copter.mission.wp_continue_is_end() && ap.land_complete && copter.mission.state()!=MISSION_COMPLETE){

			//ѡ���һ����¼�ĵ�
			if(copter.mission.mission_cmd[copter.mission.current_cmd_index].id!=MAV_CMD_NAV_WAYPOINT){
				copter.mission.current_cmd_index = (copter.mission.current_cmd_index==0?1:0);
			}

			if(copter.mission.mission_cmd[copter.mission.current_cmd_index].id==MAV_CMD_NAV_WAYPOINT){

				//��������ĺ����
				const Vector3f wpo = pv_location_to_vector(copter.mission.old_cmd.content.location); //����ɹ���һ������
				const Vector3f stopwp = pv_location_to_vector(copter.mission.stop_mission_location); //ֹͣ��������ʱ��λ��
				Vector3f insertwp;

				insertwp = pv_location_to_vector(copter.mission.mission_cmd[copter.mission.current_cmd_index].content.location);  //�ȴ�����ĺ���

				float wpo_curr = get_horizontal_distance_cm(stopwp, wpo);
				float insertwp_curr = get_horizontal_distance_cm(stopwp, insertwp);

				if(wpo_curr-insertwp_curr>=0.001){ //�˳�����ģʽλ����ǰһ����ľ�����ڼ�¼����ľ��룬�򽫲��뺽�㵽֮ǰ����֮��
					copter.mission.continue_wp_index = copter.mission.old_cmd.index+1;
				}else if(insertwp_curr-wpo_curr>=0.001){ //�˳�����ģʽλ����ǰһ����ľ���С�ڼ�¼����ľ��룬�򽫲��뺽�㵽֮ǰ����֮ǰ
					copter.mission.continue_wp_index = copter.mission.old_cmd.index;
				}

				copter.mission.mission_cmd[copter.mission.current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
				copter.mission.mission_cmd[copter.mission.current_cmd_index].index = insert_index;
				copter.mission.mission_cmd[copter.mission.current_cmd_index].p1 = copter.mission.old_cmd.p1;

			    //��ʼ���򺽵�,��ʱ������Ч
				if(copter.mission.wp_continue_reset_wp(copter.mission.continue_wp_index, copter.mission.mission_cmd[copter.mission.current_cmd_index])){
					gcs().send_text(MAV_SEVERITY_INFO, "Reset WP Success!");
					copter.mission.wp_continue_stop();
					pos_record_falg = false;
				}
			}
		}
}
