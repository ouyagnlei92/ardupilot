#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);

    flightmode->update_navigation();

    //自主飞行航点记录，断点续航功能写在该部分
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
	static Location break_wp_pos;   //自主任务中断位置
	static float old_ground_speed = 0.0;
	static uint32_t pos_last_time_ms = 0;
	static uint8_t current_cmd_index = 0;
	static AP_Mission::Mission_Command  mission_cmd[2] = { {}, {} };
	static AP_Mission::Mission_Command mission_add_cmd[3];
	static AP_Mission::Mission_Command old_nav_cmd;
	static bool auto_continue_success = false;

	uint32_t pos_time_ms = 0;

	if(!mission.wp_continue_is_open()) return;   //没有打开断点续航功能  用户可选择打开断点续飞功能

	//自主飞行模式下正在执行RTL或者LAND指令，则取消断点续航飞行记录程序
	if(flightmode==&copter.mode_auto){
		if(mode_auto.mode()==Auto_RTL || mode_auto.mode()==Auto_Land){
			wp_continue_start = false;
			wp_continue_complete = false;
			auto_continue_success = false;
		}
	}

	//自主任务被中断，记录中断位置
	if(!wp_continue_complete && wp_continue_start && mission.state()==MISSION_STOPPED){
		break_wp_pos = copter.current_loc;
		wp_continue_complete = true;
		gcs().send_text(MAV_SEVERITY_INFO, "Break Auto! Record Pos!");
	}

	if(mission.state()==MISSION_RUNNING){  //自主任务正在进行

		//判断是否开始进入断点记录
		if(!wp_continue_start){
			AP_Mission::Mission_Command nav_cmd1 = mission.wp_continue_nav_cmd_complete();
			if(nav_cmd1.id==MAV_CMD_NAV_WAYPOINT){
				wp_continue_start = true;
				old_ground_speed = ahrs.groundspeed()*100;     //获取当前地速   cm/s
				pos_last_time_ms = AP_HAL::millis();
				gcs().send_text(MAV_SEVERITY_INFO, "Start Record Waypoint");
			}
		}

		//执行航点指令，开始计数记时，并且记录位置
		if(wp_continue_start){
			float groundSpeed = ahrs.groundspeed()*100;    //获取当前地速   cm/s
			float dist = wp_continue_pos_distance();              //用户设定的断点续飞往前选点距离
			dist = dist<=10.0 ? 10.0 : dist;
			dist = dist*100.0;
			if(groundSpeed<=100) pos_time_ms = dist/100;
			else pos_time_ms = dist / (groundSpeed*0.6+old_ground_speed*0.4);
			old_ground_speed = groundSpeed;

			if(AP_HAL::millis()-pos_last_time_ms>=pos_time_ms){ //记录一个航点位置
				current_cmd_index %= 2;
				mission_cmd[current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
				mission_cmd[current_cmd_index].content.location = copter.current_loc;  //记录当前位置
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
			}else if(do_cmd1.id==MAV_CMD_NAV_TAKEOFF){   //TAKE OFF执行完毕，选择自动续飞
				if(copter.mission.wp_continue_cmd_total()==copter.mission.num_commands() && copter.mission.wp_continue_nav_cmd_index()>0 && copter.mission.wp_continue_cmd_total()>1){  //航点没有被外界修改过，则选择断点续飞
					if(!auto_continue_success && set_current_cmd(copter.mission.wp_continue_nav_cmd_index())){
						auto_continue_success = true;
						gcs().send_text(MAV_SEVERITY_INFO, "Auto Switch to WP %d", copter.mission.wp_continue_nav_cmd_index());
					}
				}
			}
		}
	  }

	//着落完成并且有开启过断点续航,重新排列航点,在加锁的时候进行
   if(!motors->armed() && wp_continue_complete && ap.land_complete && copter.mission.state()!=MISSION_COMPLETE){

			//选择第一个记录的点
			if(mission_cmd[current_cmd_index].id!=MAV_CMD_NAV_WAYPOINT){
				current_cmd_index = (current_cmd_index==0?1:0);
			}

			if(mission_cmd[current_cmd_index].id==MAV_CMD_NAV_WAYPOINT){

				//求得需插入的航点号
				const Vector3f wpo = pv_location_to_vector(old_nav_cmd.content.location); //最近飞过的一个航点
				const Vector3f stopwp = pv_location_to_vector(break_wp_pos); //停止自主飞行时的位置
				Vector3f insertwp;

				insertwp = pv_location_to_vector(mission_cmd[current_cmd_index].content.location);  //等待插入的航点

				float wpo_curr = get_horizontal_distance_cm(stopwp, wpo);
				float insertwp_curr = get_horizontal_distance_cm(stopwp, insertwp);

				if(wpo_curr-insertwp_curr>=0.001){ //退出自主模式位置与前一航点的距离大于记录航点的距离，则将插入航点到之前航点之后
					copter.mission.wp_continue_set_nav_cmd_index(old_nav_cmd.index+1);
				}else if(insertwp_curr-wpo_curr>=0.001){ //退出自主模式位置与前一航点的距离小于记录航点的距离，则将插入航点到之前航点之前
					copter.mission.wp_continue_set_nav_cmd_index(old_nav_cmd.index);
				}

				mission_cmd[current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
				mission_cmd[current_cmd_index].index = copter.mission.wp_continue_nav_cmd_index();
				mission_cmd[current_cmd_index].p1 = old_nav_cmd.p1;

			    //开始排序航点,此时解锁无效
				if(copter.mission.wp_continue_reset_wp(copter.mission.wp_continue_nav_cmd_index(), mission_cmd[current_cmd_index])){
					gcs().send_text(MAV_SEVERITY_INFO, "Reset WP Success!");
					copter.mission.wp_continue_stop();
					wp_continue_start = false;
					wp_continue_complete = false;
				}
			}
	}
}
