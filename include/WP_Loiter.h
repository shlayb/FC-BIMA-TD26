#pragma once

#include "Attitude.h"
#include "AutoMode.h"
#include "AHRS.h"
#include "Navigation.h"
#include <AP_Math.h>
#include "Radio.h"

/*https://www.movable-type.co.uk/scripts/latlong.html*/
int calc_wp_loit() {
    // hitung jumlah waypoint dibutuhkan untuk loiter
	int circumference =2.0f*M_PI*loiter.loiter_radius;
	return (circumference/(2*loiter.wp_radius))+1;
}

void gene_center_loiter()
{
	Locations pivot_p; // the location of 1's waypoint 
	if(ahrs.groundspeed!= 0){
		float pivot_range = loiter.loiter_radius;
	}else {
		float pivot_range = loiter.loiter_radius;
	}
	current_loc.point_use_bearing(pivot_p, imu.heading, loiter.loiter_radius);
	
	float i;
	if(loiter.direction==-1){
		i = imu.heading-90.0f;
		if(i<0){
			i+=360;
		}
		pivot_p.point_use_bearing(loiter.center,i,loiter.loiter_radius);
	}else{
		i = imu.heading+90.0f;
		if(i>360){
			i-=360;
		}
		pivot_p.point_use_bearing(loiter.center,i,loiter.loiter_radius);
	}	
}

void update_loiter()
{
	loiter.sum_cd +=(1/loiter.num_wp_loiter);
	loiter.flag_wp++;
	if(loiter.flag_wp >= loiter.num_wp_loiter) {
		loiter.flag_wp = 0;
		loiter.counter++;
	}
	if(loiter.counter >= 2){
		mode_now == 1;
	}
	set_next_WP(loiter.wp_loiter[loiter.flag_wp]);
}

void init_loiter(){
	loiter.flag_wp = -1;
	//set_param_loiter(Location center,radius);
	loiter.loiter_radius = MAX(LOITER_RADIUS_DEFAULT,loiter.in_loit_rad);
	gene_center_loiter();
	loiter.counter = 0;
	loiter.wp_needed = calc_wp_loit();
	loiter.num_wp_loiter = MIN(loiter.wp_needed, 100);
	float div = loiter.num_wp_loiter*1.0f;
	// float angle_interval = 360 / loiter.num_wp_loiter;
	if(loiter.direction == 1)
	{
		for(int x=0;x<loiter.num_wp_loiter;x++)
			{
				float i;
				i=(x/div)*360.0f + imu.heading-90.0f;
				if(i>360)
				{
					i-=360;
				}
				if(i<0)
				{
					i+=360;
				}
				loiter.center.point_use_bearing(loiter.wp_loiter[x],i,loiter.loiter_radius);
			}
	}
	else if(loiter.direction==-1)
	{
		for(int x=0; x<loiter.num_wp_loiter; x++)
			{	

				float i;
				i=((div-x)/div)*360 + imu.heading+ 90.0f;
				
				if(i>360)
				{
					i-=360;
				}
				if(i<0)
				{
					i+=360;
				}
				loiter.center.point_use_bearing(loiter.wp_loiter[x],i,loiter.loiter_radius);
			}
	}
	// dbg("%d,%d,red,, \r\n",fixwing.current_loc.lat,fixwing.current_loc.lng);
	// dbg("%d,%d,black,,\r\n",loiter.center.lat,loiter.center.lng);
	// for(int a=0;a<fixwing.loiter.num_wp_loiter;a++)
	// {
	// 	telemprint("%d,%d,red,square, \r\n",loiter.wp_loiter[a].lat,loiter.wp_loiter[a].lng) ;
	// 	dbg("%d,%d,,,  \r\n",loiter.wp_loiter[a].lat,loiter.wp_loiter[a].lng) ;
	// }
	update_loiter();
 }


void set_param_loiter(Locations center,float radius)
{
	loiter.center = center;
	loiter.in_loit_rad = radius;
};
