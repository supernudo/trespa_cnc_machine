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
#include <dac_mc.h>
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
static void do_cs(void *dummy) 
{
	/* read encoders */
	if (maindspic.flags & DO_ENCODERS) {
		encoders_dspic_manage(NULL);
	}

	/* control system */
	if (maindspic.flags & DO_CS) {
		if (maindspic.axis_x.on)
			cs_manage(&maindspic.axis_x.cs);
	}

	/* blocking detection */
	if (maindspic.flags & DO_BD) {
		bd_manage_from_cs(&maindspic.axis_x.bd, &maindspic.axis_x.cs);
	}
	
	/* brakes */
	if (maindspic.flags & DO_POWER)
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

	/* ---- CS axis x */
	/* PID */
	pid_init(&maindspic.axis_x.pid);
	pid_set_gains(&maindspic.axis_x.pid, P_CONST, I_CONST, D_CONST);
	pid_set_maximums(&maindspic.axis_x.pid, 0, 65000, 65000);
	pid_set_out_shift(&maindspic.axis_x.pid,1);	
	pid_set_derivate_filter(&maindspic.axis_x.pid, 1);

	/* QUADRAMP */
	quadramp_init(&maindspic.axis_x.qr);
	quadramp_set_1st_order_vars(&maindspic.axis_x.qr, NORMAL_SPEED, NORMAL_SPEED); 	/* set speed */
	quadramp_set_2nd_order_vars(&maindspic.axis_x.qr, 1, 1); 	/* set accel */
	/* CS */
	cs_init(&maindspic.axis_x.cs);
	cs_set_consign_filter(&maindspic.axis_x.cs, quadramp_do_filter, &maindspic.axis_x.qr);
	cs_set_correct_filter(&maindspic.axis_x.cs, pid_do_filter, &maindspic.axis_x.pid);
	cs_set_process_in(&maindspic.axis_x.cs, dac_set_and_save, X_DAC);
	cs_set_process_out(&maindspic.axis_x.cs, encoders_dspic_get_value, X_ENCODER);
	cs_set_consign(&maindspic.axis_x.cs, 0);

	/* Blocking detection */
	bd_init(&maindspic.axis_x.bd);
	bd_set_speed_threshold(&maindspic.axis_x.bd, 5);
	bd_set_current_thresholds(&maindspic.axis_x.bd, 100, 8000, 1000000, 30);


	/* set them on !! */
	maindspic.axis_x.on = 1;

	scheduler_add_periodical_event_priority(do_cs, NULL,
						5000L / SCHEDULER_UNIT,
						CS_PRIO);
}

