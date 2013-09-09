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
#include <ax12.h>
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
#include "ax12_user.h"

void dump_cs(const char *name, struct cs *cs);

/* called every 5 ms */
static void do_cs(void *dummy) 
{
	/* read encoders */
	if (slavedspic.flags & DO_ENCODERS) {
		encoders_dspic_manage(NULL);
	}

	/* control system */
	if (slavedspic.flags & DO_CS) {
		if (slavedspic.alpha.on)
			cs_manage(&slavedspic.alpha.cs);
		if (slavedspic.beta.on)
			cs_manage(&slavedspic.beta.cs);
	}

	/* blocking detection */
	if (slavedspic.flags & DO_BD) {
		bd_manage_from_cs(&slavedspic.alpha.bd, &slavedspic.alpha.cs);
		bd_manage_from_cs(&slavedspic.beta.bd, &slavedspic.beta.cs);
	}

	/* brakes */
//	if (slavedspic.flags & DO_POWER)
//		BRAKE_OFF();
//	else
//		BRAKE_ON();
	
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

	/* ---- CS axis alpha */
	/* PID */
	pid_init(&slavedspic.alpha.pid);
	pid_set_gains(&slavedspic.alpha.pid, 10000, 800, 20000);
	pid_set_maximums(&slavedspic.alpha.pid, 0, 250, 1023);
	pid_set_out_shift(&slavedspic.alpha.pid, 10);	
	pid_set_derivate_filter(&slavedspic.alpha.pid, 4);

	/* QUADRAMP */
	quadramp_init(&slavedspic.alpha.qr);
	quadramp_set_1st_order_vars(&slavedspic.alpha.qr, 3, 3); 	/* set speed */
	quadramp_set_2nd_order_vars(&slavedspic.alpha.qr, 1, 1); 	/* set accel */

	/* CS */
	cs_init(&slavedspic.alpha.cs);
	cs_set_consign_filter(&slavedspic.alpha.cs, quadramp_do_filter, &slavedspic.alpha.qr);
	cs_set_correct_filter(&slavedspic.alpha.cs, pid_do_filter, &slavedspic.alpha.pid);
	cs_set_process_in(&slavedspic.alpha.cs, ax12_set_and_save, (void *)ALPHA_AX12);
	cs_set_process_out(&slavedspic.alpha.cs, encoders_dspic_get_value, ALPHA_ENCODER);
	cs_set_consign(&slavedspic.alpha.cs, 0);

	/* Blocking detection */
	bd_init(&slavedspic.alpha.bd);
	bd_set_speed_threshold(&slavedspic.alpha.bd, 2);
	bd_set_current_thresholds(&slavedspic.alpha.bd, 1500, 8000, 1000000, 50);

	/* ---- CS axis beta */
	/* PID */
	pid_init(&slavedspic.beta.pid);
	pid_set_gains(&slavedspic.beta.pid, 10000, 800, 20000);
	pid_set_maximums(&slavedspic.beta.pid, 0, 250, 1023);
	pid_set_out_shift(&slavedspic.beta.pid, 10);	
	pid_set_derivate_filter(&slavedspic.beta.pid, 4);

	/* QUADRAMP */
	quadramp_init(&slavedspic.beta.qr);
	quadramp_set_1st_order_vars(&slavedspic.beta.qr, 3, 3); 	/* set speed */
	quadramp_set_2nd_order_vars(&slavedspic.beta.qr, 1, 1);		/* set accel */

	/* CS */
	cs_init(&slavedspic.beta.cs);
	cs_set_consign_filter(&slavedspic.beta.cs, quadramp_do_filter, &slavedspic.beta.qr);
	cs_set_correct_filter(&slavedspic.beta.cs, pid_do_filter, &slavedspic.beta.pid);
	cs_set_process_in(&slavedspic.beta.cs, ax12_set_and_save, (void *)BETA_AX12);
	cs_set_process_out(&slavedspic.beta.cs, encoders_dspic_get_value, BETA_ENCODER);
	cs_set_consign(&slavedspic.beta.cs, 0);

	/* Blocking detection */
	bd_init(&slavedspic.beta.bd);
	bd_set_speed_threshold(&slavedspic.beta.bd, 2);
	bd_set_current_thresholds(&slavedspic.beta.bd, 1500, 8000, 1000000, 50);


	/* set them on !! */
	slavedspic.alpha.on = 1;
	slavedspic.beta.on = 1;

}

