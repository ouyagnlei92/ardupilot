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
	static bool pos_record_falg = false;

	if(!copter.mission.wp_continue_is_open()) return;   //没有打开断点续航功能

	//自主飞行模式下正在执行RTL或者LAND指令，则取消断点续航飞行记录程序
	if(flightmode==&copter.mode_auto){
		if(copter.mode_auto.mode()==Auto_RTL || copter.mode_auto.mode()==Auto_Land){
			copter.mission.wp_continue_stop();
			pos_record_falg = false;
		}
	}

	//自主任务在没有执行完任务之后被强行中断，记录当前中断位置
	if(!pos_record_falg && copter.mission.wp_continue_is_end()){
		copter.mission.wp_continue_abort_pos(inertial_nav.get_position());
		gcs().send_text(MAV_SEVERITY_INFO, "Aboart Auto! Record Pos!");
		pos_record_falg = true;
	}

	//着落完成并且有开启过断点续航,重新排列航点,在加锁的时候进行
   if(!motors->armed() && copter.mission.wp_continue_is_end() && ap.land_complete && copter.mission.state()!=MISSION_COMPLETE){

			//选择第一个记录的点
			if(copter.mission.mission_cmd[copter.mission.current_cmd_index].id!=MAV_CMD_NAV_WAYPOINT){
				copter.mission.current_cmd_index = (copter.mission.current_cmd_index==0?1:0);
			}

			if(copter.mission.mission_cmd[copter.mission.current_cmd_index].id==MAV_CMD_NAV_WAYPOINT){

				//求得需插入的航点号
				const Vector3f wpo = pv_location_to_vector(copter.mission.old_cmd.content.location); //最近飞过的一个航点
				const Vector3f stopwp = pv_location_to_vector(copter.mission.stop_mission_location); //停止自主飞行时的位置
				Vector3f insertwp;

				insertwp = pv_location_to_vector(copter.mission.mission_cmd[copter.mission.current_cmd_index].content.location);  //等待插入的航点

				float wpo_curr = get_horizontal_distance_cm(stopwp, wpo);
				float insertwp_curr = get_horizontal_distance_cm(stopwp, insertwp);

				if(wpo_curr-insertwp_curr>=0.001){ //退出自主模式位置与前一航点的距离大于记录航点的距离，则将插入航点到之前航点之后
					copter.mission.continue_wp_index = copter.mission.old_cmd.index+1;
				}else if(insertwp_curr-wpo_curr>=0.001){ //退出自主模式位置与前一航点的距离小于记录航点的距离，则将插入航点到之前航点之前
					copter.mission.continue_wp_index = copter.mission.old_cmd.index;
				}

				copter.mission.mission_cmd[copter.mission.current_cmd_index].id = MAV_CMD_NAV_WAYPOINT;
				copter.mission.mission_cmd[copter.mission.current_cmd_index].index = insert_index;
				copter.mission.mission_cmd[copter.mission.current_cmd_index].p1 = copter.mission.old_cmd.p1;

			    //开始排序航点,此时解锁无效
				if(copter.mission.wp_continue_reset_wp(copter.mission.continue_wp_index, copter.mission.mission_cmd[copter.mission.current_cmd_index])){
					gcs().send_text(MAV_SEVERITY_INFO, "Reset WP Success!");
					copter.mission.wp_continue_stop();
					pos_record_falg = false;
				}
			}
		}
}
