/*  
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
 *
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *  Based on revision : cs.c,v 1.8 2009/05/27 20:04:07 zer0 Exp
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "actuator.h"

void dump_cs(const char *name, struct cs *cs);

/* called every 5 ms */
void do_cs(void *dummy) 
{
	/* read encoders */
	if (slavedspic.flags & DO_ENCODERS) {
		encoders_dspic_manage(NULL);
	}

	/* control system */
	if (slavedspic.flags & DO_CS) {
		if (slavedspic.y.on)
			cs_manage(&slavedspic.y.cs);
		if (slavedspic.z.on)
			cs_manage(&slavedspic.z.cs);
	}

	/* blocking detection */
	if (slavedspic.flags & DO_BD) {
		bd_manage_from_cs(&slavedspic.y.bd, &slavedspic.y.cs);
		bd_manage_from_cs(&slavedspic.z.bd, &slavedspic.z.cs);
	}

	/* brakes */

	if (slavedspic.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
	
}

void dump_cs_debug(const char *name, struct cs *cs)
{
	DEBUG(E_USER_CS, "%s cons=% .5ld fcons=% .5ld err=% .5ld "
	      "in=% .5ld out=% .5ld", 
	      name, cs_get_consign(cs), cs_get_filtered_consign(cs),
	      cs_get_error(cs), cs_get_filtered_feedback(cs),
	      cs_get_out(cs));
}

void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons=% .5ld fcons=% .5ld err=% .5ld "
		      "in=% .5ld out=% .5ld\r\n"), 
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

void dspic_cs_init(void)
{

	/* ---- CS axis Y */

	/* PID */
	pid_init(&slavedspic.y.pid);
	pid_set_gains(&slavedspic.y.pid, 1800, 0, 1500);
	pid_set_maximums(&slavedspic.y.pid, 0, 2100, 2100);
	pid_set_out_shift(&slavedspic.y.pid, 5);	
	pid_set_derivate_filter(&slavedspic.y.pid, 1);

	/* QUADRAMP */
	quadramp_init(&slavedspic.y.qr);
	quadramp_set_1st_order_vars(&slavedspic.y.qr, 70, 70); 	/* set speed */
	quadramp_set_2nd_order_vars(&slavedspic.y.qr, 5, 5); 		/* set accel */

	/* CS */
	cs_init(&slavedspic.y.cs);
	cs_set_consign_filter(&slavedspic.y.cs, quadramp_do_filter, &slavedspic.y.qr);
	cs_set_correct_filter(&slavedspic.y.cs, pid_do_filter, &slavedspic.y.pid);
	cs_set_process_in(&slavedspic.y.cs, pwm_mc_set_and_save, PWM_MC_Y);
	cs_set_process_out(&slavedspic.y.cs, encoders_dspic_get_value, ENCODER_Y);
	cs_set_consign(&slavedspic.y.cs, 0);

	/* Blocking detection */
	//bd_init(&slavedspic.y.bd);
	//bd_set_speed_threshold(&slavedspic.y.bd, 2);
	//bd_set_current_thresholds(&slavedspic.y.bd, 1500, 8000, 1000000, 50);

	/* ---- CS axis Z */

	/* PID */
	pid_init(&slavedspic.z.pid);
	pid_set_gains(&slavedspic.z.pid, 3000, 0, 1000);
	pid_set_maximums(&slavedspic.z.pid, 0, 65000, 65000);
	pid_set_out_shift(&slavedspic.z.pid, 1);	
	pid_set_derivate_filter(&slavedspic.z.pid, 1);

	/* QUADRAMP */
	quadramp_init(&slavedspic.z.qr);
	quadramp_set_1st_order_vars(&slavedspic.z.qr, 70, 70); 	/* set speed */
	quadramp_set_2nd_order_vars(&slavedspic.z.qr, 5, 5); 		/* set accel */

	/* CS */
	cs_init(&slavedspic.z.cs);
	cs_set_consign_filter(&slavedspic.z.cs, quadramp_do_filter, &slavedspic.z.qr);
	cs_set_correct_filter(&slavedspic.z.cs, pid_do_filter, &slavedspic.z.pid);
	cs_set_process_in(&slavedspic.z.cs, dac_set_and_save, DAC_MC_Z);
	cs_set_process_out(&slavedspic.z.cs, encoders_dspic_get_value, ENCODER_Z);
	cs_set_consign(&slavedspic.z.cs, 0);

	/* Blocking detection */
	//bd_init(&slavedspic.z.bd);
	//bd_set_speed_threshold(&slavedspic.z.bd, 5);
	//bd_set_current_thresholds(&slavedspic.z.bd, 100, 8000, 1000000, 30);


	/* set them on !! */
	slavedspic.y.on = 1;
	slavedspic.z.on = 1;

}


