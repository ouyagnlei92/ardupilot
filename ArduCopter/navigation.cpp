#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);

    flightmode->update_navigation();
    
    wp_continue_fly();
}

// distance between vehicle and home in cm
uint32_t Copter::home_distance()
{
    if (position_ok()) {
        _home_distance = current_loc.get_distance(ahrs.get_home()) * 100;
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    if (position_ok()) {
        _home_bearing = current_loc.get_bearing_to(ahrs.get_home());
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
	static uint32_t pos_count = 0;
	static bool have_take_off = false;
	AP_Mission::Mission_Command nav_cmd1 = {};
	AP_Mission::Mission_Command do_cmd1 = {};
	static uint32_t led_time_ms = 0;

	uint32_t pos_time_ms = 0;
	AP_Mission& mission = copter.mode_auto.mission;

	if(!mission.wp_continue_is_open()) return;   //û�д򿪶ϵ���������  �û���ѡ��򿪶ϵ����ɹ���

	//��������ģʽ������ִ��RTL����LANDָ���ȡ���ϵ��������м�¼����
	if(flightmode==&copter.mode_auto){
		if(mode_auto.mode()==Auto_RTL || mode_auto.mode()==Auto_Land){
			wp_continue_start = false;
			wp_continue_complete = false;
			auto_continue_success = false;
			nav_cmd1.id = 0;
			do_cmd1.id = 0;
			pos_count = 0;
			mission.wp_continue_set_cmd_index(0);
			return;
		}
	}

	nav_cmd1 = mission.wp_continue_nav_cmd_complete();
	do_cmd1 = mission.wp_continue_do_cmd_complete();

	//���������жϣ���¼�ж�λ��
	if(!wp_continue_complete && wp_continue_start && mission.state()==AP_Mission::MISSION_STOPPED){
		break_wp_pos = copter.current_loc;
		wp_continue_complete = true;
		gcs().send_text(MAV_SEVERITY_INFO, "Break Auto! Record Pos!");
	}

	if(mission.state()==AP_Mission::MISSION_RUNNING){  //�����������ڽ���

		//�ж��Ƿ�ʼ����ϵ��¼
		if(motors->armed() && !wp_continue_start){
			if(have_take_off && nav_cmd1.id==MAV_CMD_NAV_WAYPOINT){
				wp_continue_start = true;
				nav_cmd1.id = 0;
				do_cmd1.id = 0;
				pos_count = 0;
				mission_cmd[0].id = 0;
				mission_cmd[1].id = 0;
				old_ground_speed = ahrs.groundspeed()*100;     //��ȡ��ǰ����   cm/s
				pos_last_time_ms = AP_HAL::millis();
				gcs().send_text(MAV_SEVERITY_INFO, "Start Record Waypoint");
			}else if(nav_cmd1.id==MAV_CMD_NAV_TAKEOFF /*|| copter.mission.get_current_nav_cmd().id==MAV_CMD_NAV_TAKEOFF*/){   //TAKE OFFִ����ϣ�ѡ���Զ�����
			    have_take_off = true;
				if(mission.wp_continue_cmd_total()==mission.num_commands() && mission.wp_continue_nav_cmd_index()>0 &&mission.wp_continue_cmd_total()>1){  //����û�б�����޸Ĺ�����ѡ��ϵ�����
					if(!auto_continue_success && mission.set_current_cmd(mission.wp_continue_nav_cmd_index())){
						auto_continue_success = true;
						gcs().send_text(MAV_SEVERITY_INFO, "Auto Switch to WP #%d", mission.wp_continue_nav_cmd_index());
					}
				}
			} 

			if(do_cmd1.id==MAV_CMD_DO_CHANGE_SPEED && mission_add_cmd[0].index!=do_cmd1.index){
				mission_add_cmd[0] = do_cmd1;
				//gcs().send_text(MAV_SEVERITY_INFO, "have speed");
			}else if(do_cmd1.content.cam_trigg_dist.meters>0.5 && do_cmd1.id==MAV_CMD_DO_SET_CAM_TRIGG_DIST && mission_add_cmd[1].index!=do_cmd1.index){
				mission_add_cmd[1] = do_cmd1;
				//gcs().send_text(MAV_SEVERITY_INFO, "have camera trigg dist %3.1fm", do_cmd1.content.cam_trigg_dist.meters);
			}
		}

		//ִ�к���ָ���ʼ������ʱ�����Ҽ�¼λ��
		if(wp_continue_start){
			float groundSpeed = ahrs.groundspeed()*100;    //��ȡ��ǰ����   cm/s
			float dist = mission.wp_continue_pos_distance();              //�û��趨�Ķϵ�������ǰѡ�����
			dist = dist<=10.0 ? 10.0 : dist;
			dist = dist*100.0;
			float gds = (groundSpeed*0.6+old_ground_speed*0.4);
			if(gds<0.001) return;
			if(groundSpeed<=100) pos_time_ms = dist/100;
			else pos_time_ms = dist / gds;
			old_ground_speed = groundSpeed;

			if(AP_HAL::millis()-pos_last_time_ms>=pos_time_ms){ //��¼һ������λ��
			    if(nav_cmd1.id==MAV_CMD_NAV_WAYPOINT){
					//current wp distance with current pos
					Vector3f currentpos; 
					Vector3f currentwp;
					bool terra;
					copter.wp_nav->get_vector_NEU(copter.current_loc, currentpos, terra);
					copter.wp_nav->get_vector_NEU(nav_cmd1.content.location, currentwp, terra);
					float curr_posdis = get_horizontal_distance_cm(currentwp, currentpos);
					if(curr_posdis<=mission.wp_continue_pos_distance()*100){
						pos_count = 0;
						current_cmd_index = 0;
					}else{
						current_cmd_index %= 2;
						mission_cmd[current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
						mission_cmd[current_cmd_index].content.location = copter.current_loc;  //��¼��ǰλ��
						++current_cmd_index;
						++pos_count;
					}						
					pos_last_time_ms = AP_HAL::millis();
				}				
			}

			if(old_nav_cmd.index!= nav_cmd1.index && nav_cmd1.id==MAV_CMD_NAV_WAYPOINT){
				old_nav_cmd = nav_cmd1;
			}

			if(do_cmd1.id==MAV_CMD_DO_CHANGE_SPEED && mission_add_cmd[0].index!=do_cmd1.index){
				mission_add_cmd[0] = do_cmd1;
				//gcs().send_text(MAV_SEVERITY_INFO, "have speed");
			}else if(do_cmd1.content.cam_trigg_dist.meters>0.5 && do_cmd1.id==MAV_CMD_DO_SET_CAM_TRIGG_DIST && mission_add_cmd[1].index!=do_cmd1.index){
				mission_add_cmd[1] = do_cmd1;
				//gcs().send_text(MAV_SEVERITY_INFO, "have camera trigg dist %3.1fm", do_cmd1.content.cam_trigg_dist.meters);
			}
		}
	  }

	//������ɲ����п������ϵ�����,�������к���,�ڼ�����ʱ�����
   if(!motors->armed() && wp_continue_complete && ap.land_complete && mission.state()!=AP_Mission::MISSION_COMPLETE){

		if(pos_count==0 && mission_cmd[0].id==0 && mission_cmd[1].id==0){
			//copter.mission.wp_continue_set_cmd_index(0);	
			//copter.mission.wp_continue_set_continue_total(0);
			gcs().send_text(MAV_SEVERITY_INFO, "No Continue WP #%d", mission.wp_continue_nav_cmd_index());		
		}else if(pos_count<=2 && mission_cmd[0].id==MAV_CMD_NAV_WAYPOINT && mission_cmd[1].id==MAV_CMD_NAV_WAYPOINT){
			mission.wp_continue_set_cmd_index(old_nav_cmd.index);
			mission.wp_continue_set_continue_total(mission.num_commands());
			gcs().send_text(MAV_SEVERITY_INFO, "Save Continue WP #%d", old_nav_cmd.index);
		}else if(pos_count>2&& mission_cmd[0].id==MAV_CMD_NAV_WAYPOINT && mission_cmd[1].id==MAV_CMD_NAV_WAYPOINT){
			mission.wp_continue_set_cmd_index(old_nav_cmd.index+1);
			current_cmd_index = (current_cmd_index%2==0?0:1);
			mission_cmd[current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
			mission_cmd[current_cmd_index].index = mission.wp_continue_nav_cmd_index();
			mission_cmd[current_cmd_index].p1 = old_nav_cmd.p1;
			mission_add_cmd[2] = mission_cmd[current_cmd_index];
			mission_add_cmd[2].content.location.alt = old_nav_cmd.content.location.alt;
			if(mission.wp_continue_reset_wp(mission.wp_continue_nav_cmd_index(), &mission_add_cmd[0])){
				AP_Notify::flags.wp_continue = true;
				led_time_ms = AP_HAL::millis();
				gcs().send_text(MAV_SEVERITY_INFO, "Reset WP Success, WP #%d", old_nav_cmd.index+1);
			}
		}

		wp_continue_start = false;
		wp_continue_complete = false;
		auto_continue_success = false;
		have_take_off = false;
		nav_cmd1.id = 0;
		do_cmd1.id = 0;
	}

	if(mission.state()==AP_Mission::MISSION_COMPLETE){
		wp_continue_start = false;
		wp_continue_complete = false;
		auto_continue_success = false;
		auto_continue_success = false;
		nav_cmd1.id = 0;
		do_cmd1.id = 0;
		pos_count = 0;
		have_take_off = false;
		mission.wp_continue_set_cmd_index(0);
	}

	if(AP_Notify::flags.wp_continue && AP_HAL::millis()-led_time_ms>=8000){
		AP_Notify::flags.wp_continue = false;
	}
}
