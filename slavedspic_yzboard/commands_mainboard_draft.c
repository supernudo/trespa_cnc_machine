/*
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
 *
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *	Based on: commands_maindspic.c,v 1.8 2009/05/27 20:04:07 zer0 Exp
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
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <time.h>
#include <encoders_dspic.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "sensor.h"
#include "cmdline.h"
#include "actuator.h"

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = DO_ENCODERS | DO_CS | DO_BD | DO_POWER;
		if (!strcmp_P(res->arg2, PSTR("on")))
			maindspic.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			maindspic.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\n\r"), 
				 (DO_ENCODERS & maindspic.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\n\r"), 
				 (DO_CS & maindspic.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\n\r"), 
				 (DO_BD & maindspic.flags) ? "on":"off");
			printf_P(PSTR("power is %s\n\r"), 
				 (DO_POWER & maindspic.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		bit = DO_CS;
	}
	else if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;

	if (!strcmp_P(res->arg2, PSTR("on")))
		maindspic.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
		 	dac_mc_set(X_DAC, 0);
		}
		maindspic.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\n\r"), res->arg1, 
		      (bit & maindspic.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#bd#power";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
	.f = cmd_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_event_arg0, 
		(prog_void *)&cmd_event_arg1, 
		(prog_void *)&cmd_event_arg2, 
		NULL,
	},
};



/**********************************************************/
/* slavedspic commands */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_slavedspic_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};


static void cmd_slavedspic_parsed(void * parsed_result, void * data)
{
	int16_t c;
	int8_t cmd = 0;
	struct vt100 vt100;

	struct cmd_slavedspic_result *res = parsed_result;

	vt100_init(&vt100);
	
	if(!strcmp_P(res->arg1, "raw_mode")){
		while(cmd != KEY_CTRL_C) {
			
			/* received from slavedspic */
			if((c = uart_recv_nowait(1))!= -1)
				uart_send_nowait(CMDLINE_UART,c);
			
			/* send to slavedspic */
			c = cmdline_getchar();
			if (c == -1) {
				continue;
			}
			cmd = vt100_parser(&vt100, c);
			uart_send_nowait(1,c);	
		}
	}	
}

prog_char str_slavedspic_arg0[] = "slavedspic";
parse_pgm_token_string_t cmd_slavedspic_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_slavedspic_result, arg0, str_slavedspic_arg0);
prog_char str_slavedspic_arg1[] = "raw_mode";
parse_pgm_token_string_t cmd_slavedspic_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_slavedspic_result, arg1, str_slavedspic_arg1);

prog_char help_slavedspic[] = "slavedspic commads";
parse_pgm_inst_t cmd_slavedspic = {
	.f = cmd_slavedspic_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_slavedspic,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_slavedspic_arg0, 
		(prog_void *)&cmd_slavedspic_arg1, 
		NULL,
	},
};


/* Application functions */

void hard_stop(struct cs_block *csb, void * enc_id)
{
	cs_set_consign(&csb->cs, encoders_dspic_get_value(enc_id));

	csb->qr.previous_var = 0;
	csb->qr.previous_out = encoders_dspic_get_value(enc_id);
}

int8_t wait_pos_end(struct cs_block *csb)
{
	uint8_t ret=0;

	while(ret == 0){

		/* test traj end */
		if(cs_get_consign(&csb->cs) == cs_get_filtered_consign(&csb->cs)){
			ret = 1;
			NOTICE(E_USER_APP, "Positioning ends OK");
			return 1;
		}

		/* test blocking */
		ret = bd_get(&csb->bd);
		if(ret){

			hard_stop(csb, X_ENCODER);
			pid_reset(&csb->pid);
			bd_reset(&csb->bd);

			ERROR(E_USER_APP, "Positioning ends BLOCKING!!");
			return -1;
		}
	}
}

#define AUTOPOS_SPEED	1
#define AUTOPOS_ACCEL	100
//#define MEASURE_X_RANGE
void axis_x_autopos(struct cs_block *csb, struct dac_mc *dac_mc, void * enc_id, uint8_t reverse)
{
#ifdef MEASURE_X_RANGE
	int32_t val;
#endif
	int8_t ret;


	/* goto LEFT without CS, low speed */
	/* wait for LEFT FC */
	/* goto reach calib sensor without CS */
	/* wait for calib sensor ON (encoders reset is done by input interrupt) */

#ifdef MEASURE_X_RANGE
	/* goto RIGHT with CS */
	/* wait RIGHT FC or blocking */
#endif

	/* goto zero with cs */
	cs_set_consign(&csb->cs, (int32_t)(0*DIST_IMP_MM));
	maindspic.flags |= DO_CS;
	DEBUG(E_USER_APP, "Goto near zero");	

	/* wait trajectory end */
	ret = wait_pos_end(csb);

	/* set calibrate flag */
	if(ret == 1){
		maindspic.calib_x = 1;
	}

	DEBUG(E_USER_APP, "Calibration ends");	

	/* disable position bd */
	maindspic.position_bd = 0;

	/* set PID constants */
	pid_set_gains(&csb->pid, 800, 0, 5000);

	/* goto out zero FC range */
	if(sensor_get(S_X_MOT_L_FC)){
		maindspic.flags |= DO_CS;
		if(reverse)
			cs_set_consign(&csb->cs, (int32_t)(-200*DIST_IMP_MM));
		else
			cs_set_consign(&csb->cs, (int32_t)((200*DIST_IMP_MM)));

		while(sensor_get(S_X_MOT_L_FC));
	}

	/* goto zero with cs */
	maindspic.flags |= DO_CS;
	if(reverse)
		cs_set_consign(&csb->cs, (int32_t)(5000*DIST_IMP_MM));
	else
		cs_set_consign(&csb->cs, (int32_t)(-(5000*DIST_IMP_MM)));
	
	DEBUG(E_USER_APP, "Goto zero, reverse = %d", reverse);	

	/* wait FC */
	while(!sensor_get(S_X_MOT_L_FC));
	DEBUG(E_USER_APP, "zero FC reached");			

	/* set low speed, and hi acceleration for fast response */
	quadramp_set_1st_order_vars(&csb->qr, AUTOPOS_SPEED, AUTOPOS_SPEED);
	quadramp_set_2nd_order_vars(&csb->qr, AUTOPOS_ACCEL, AUTOPOS_ACCEL);
	DEBUG(E_USER_APP, "Down speed, new speed is %d", (int16_t)AUTOPOS_SPEED);

	/* wait end blocking */
	while(!bd_get(&csb->bd));	
	DEBUG(E_USER_APP, "End blocking");			

	/* reset encoder */
	encoders_dspic_set_value(enc_id, 0);
	hard_stop(csb, X_ENCODER);
	pid_reset(&csb->pid);
	bd_reset(&csb->bd);

	DEBUG(E_USER_APP, "Encoder reset to zero");	

	/* restore normal speed and acceleration */
	quadramp_set_1st_order_vars(&csb->qr, NORMAL_SPEED, NORMAL_SPEED);
	quadramp_set_2nd_order_vars(&csb->qr, 1, 1);
	DEBUG(E_USER_APP, "New speed is %d", (int16_t)NORMAL_SPEED);

#ifdef MEASURE_X_RANGE
	/* goto opposite with cs */
	maindspic.flags |= DO_CS;
	if(reverse)
		cs_set_consign(&csb->cs, (int32_t)(-(5000*DIST_IMP_MM)));
	else
		cs_set_consign(&csb->cs, (int32_t)(5000*DIST_IMP_MM));

	DEBUG(E_USER_APP, "Goto opposite, reverse = %d", reverse);	
	
	/* wait FC */
	while(!sensor_get(S_X_MOT_R_FC));
	DEBUG(E_USER_APP, "opposite FC reached");			

	/* set low speed */
	quadramp_set_1st_order_vars(&csb->qr, AUTOPOS_SPEED, AUTOPOS_SPEED);
	quadramp_set_2nd_order_vars(&csb->qr, AUTOPOS_ACCEL, AUTOPOS_ACCEL);
	DEBUG(E_USER_APP, "Down speed, new speed is %d", (int16_t)AUTOPOS_SPEED);

	/* wait end blocking */
	while(!bd_get(&csb->bd));	
	DEBUG(E_USER_APP, "End blocking");			

	/* get encoder value and stop */
	val = encoders_dspic_get_value(enc_id);
	hard_stop(csb, X_ENCODER);
	pid_reset(&csb->pid);
	bd_reset(&csb->bd);

	/* calculate X range */
	maindspic.offset_x_mm = 480;
	maindspic.pos_x_max_imp = (int32_t)(maindspic.offset_x_mm*DIST_IMP_MM) + val - (int32_t)(5*DIST_IMP_MM);
	maindspic.pos_x_min_imp = (int32_t)(5*DIST_IMP_MM);
	printf("Axis X range is [%ld %ld] mm\n\r",
	 (int32_t)(maindspic.pos_x_min_imp/DIST_IMP_MM),
	 (int32_t)(maindspic.pos_x_max_imp/DIST_IMP_MM));
	DEBUG(E_USER_APP, "Encoder get %ld impulses", (int32_t)val);	
#else
	/* calculate X range */
	maindspic.offset_x_mm = 480;
	maindspic.pos_x_max_imp = (int32_t)(maindspic.offset_x_mm*DIST_IMP_MM) + 54400 - (int32_t)(5*DIST_IMP_MM);
	maindspic.pos_x_min_imp = (int32_t)(maindspic.offset_x_mm*DIST_IMP_MM) + (5*DIST_IMP_MM);
	printf("Axis X range is [%ld %ld] mm\n\r",
	 (int32_t)(maindspic.pos_x_min_imp/DIST_IMP_MM),
	 (int32_t)(maindspic.pos_x_max_imp/DIST_IMP_MM));
#endif


	/* restore normal speed, acceleration and PID */
	quadramp_set_1st_order_vars(&csb->qr, NORMAL_SPEED, NORMAL_SPEED);
	quadramp_set_2nd_order_vars(&csb->qr, 1, 1);
	pid_set_gains(&csb->pid, P_CONST, I_CONST, D_CONST);
	DEBUG(E_USER_APP, "New speed is %d", (int16_t)NORMAL_SPEED);

	/* disable position bd */
	maindspic.position_bd = 1;

	/* goto new zero with cs */
	cs_set_consign(&csb->cs, (int32_t)(10*DIST_IMP_MM));
	maindspic.flags |= DO_CS;
	DEBUG(E_USER_APP, "Goto near zero");	

	/* wait trajectory end */
	ret = wait_pos_end(csb);

	/* set calibrate flag */
	if(ret == 1){
		maindspic.calib_x = 1;
	}

	DEBUG(E_USER_APP, "Calibration ends");	
}

void axis_z_set(int32_t dist_mm)
{
	struct cs_block *csb = &slavedspic.z;
	int32_t val_imp;

	/* check if axis is calibrated */	
	if(!slavedspic.z_calib){
		printf("Axis is not calibrated yet\n\r");
		return;	
	}		

	/* value in pulses */
	val_imp = (int32_t)(dist_mm*DIST_Z_IMP_MM);

	/* check range */
	if((val_imp < slavedspic.z_pos_min_imp) || (val_imp > slavedspic.z_pos_max_imp)){
		printf("Consign out of range\n\r");
		return;	
	}

	/* set consign */
	cs_set_consign(&csb->cs, (int32_t)((dist_mm-slavedspic.z_offset_mm)*DIST_Z_IMP_MM));
	DEBUG(E_USER_APP, "Set axis Z consign to %ld deg (%ld imp)",
		dist_mm, (int32_t)(dist_mm*DIST_Z_IMP_MM));

	/* wait trajectory end */
	wait_pos_end(csb);
}

void axis_y_set(int32_t dist_mm)
{
	struct cs_block *csb = &slavedspic.y;
	int32_t val_imp;

	/* check if axis is calibrated */	
	if(!slavedspic.y_calib){
		printf("Axis is not calibrated yet\n\r");
		return;	
	}		

	/* value in pulses */
	val_imp = (int32_t)(dist_mm*DIST_Y_IMP_MM);

	/* check range */
	if((val_imp < slavedspic.y_pos_min_imp) || (val_imp > slavedspic.y_pos_max_imp)){
		printf("Consign out of range\n\r");
		return;	
	}

	/* set consign */
	cs_set_consign(&csb->cs, (int32_t)((dist_mm-slavedspic.y_offset_mm)*DIST_Y_IMP_MM));
	DEBUG(E_USER_APP, "Set axis Y consign to %ld deg (%ld imp)",
		dist_mm, (int32_t)(dist_mm*DIST_Y_IMP_MM));

	/* wait trajectory end */
	wait_pos_end(csb);
}

double axis_z_get(void)
{
	/* check if axis is calibrated */
	if(!slavedspic.z_calib){
		printf("Axis is not calibrated yet\n\r");
		return 0.0;	
	}		

	return ((double)(slavedspic.z_offset_mm + (encoders_dspic_get_value(ENCODER_Z)/DIST_Z_IMP_MM)));
}

double axis_y_get(void)
{
	/* check if axis is calibrated */
	if(!slavedspic.y_calib){
		printf("Axis is not calibrated yet\n\r");
		return 0.0;	
	}		

	return ((double)(slavedspic.y_offset_mm + (encoders_dspic_get_value(ENCODER_Y)/DIST_Y_IMP_MM)));
}

/**********************************************************/
/* Axis mode1 */

/* this structure is filled when cmd_axis_mode1 is parsed successfully */
struct cmd_axis_mode1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_axis_mode1 is parsed successfully */
static void cmd_axis_mode1_parsed(void *parsed_result, void *data)
{
	struct cmd_axis_mode1_result *res = parsed_result;
	double ret;

	if (!strcmp_P(res->arg1, "autopos")) {
		axis_x_autopos(&maindspic.axis_x, X_DAC, X_ENCODER, 0);
	}
	else if (!strcmp_P(res->arg1, "get")) {
		ret = axis_x_get(X_ENCODER);
		printf("axis_x pos = %.4f mm\n\r", ret);
	}

	printf("Done\n\r");
}

prog_char str_axis_mode1_arg0[] = "axis_z#axis_y";
parse_pgm_token_string_t cmd_axis_mode1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_axis_mode1_result, arg0, str_axis_mode1_arg0);
prog_char str_axis_mode1_arg1[] = "autopos#get";
parse_pgm_token_string_t cmd_axis_mode1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_axis_mode1_result, arg1, str_axis_mode1_arg1);

prog_char help_axis_mode1[] = "Axis X positioning commands (mode1)";
parse_pgm_inst_t cmd_axis_mode1 = {
	.f = cmd_axis_mode1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_axis_mode1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_axis_mode1_arg0, 
		(prog_void *)&cmd_axis_mode1_arg1, 
		NULL,
	},
};

/**********************************************************/
/* axis_x mode2 */

/* this structure is filled when cmd_axis_mode2 is parsed successfully */
struct cmd_axis_mode2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg3;
};

/* function called when cmd_axis_mode2 is parsed successfully */
static void cmd_axis_mode2_parsed(void *parsed_result, void *show)
{
	struct cmd_axis_mode2_result *res = parsed_result;

	if(!show){
		printf("Axis X offset = %ld mm\n\r", maindspic.offset_x_mm);
		printf("Axis X range is [%ld %ld] mm\n\r",
		 (int32_t)(maindspic.pos_x_min_imp/DIST_IMP_MM),
		 (int32_t)(maindspic.pos_x_max_imp/DIST_IMP_MM));
		goto end;
	}
	
	if (!strcmp_P(res->arg1, "set")) {
		axis_x_set(&maindspic.axis_x, res->arg3);
	}
	else if (!strcmp_P(res->arg1, "offset")) {
		maindspic.offset_x_mm = res->arg3;
		maindspic.pos_x_min_imp = (int32_t)(maindspic.offset_x_mm*DIST_IMP_MM) + (5*DIST_IMP_MM);
		maindspic.pos_x_max_imp = (int32_t)(maindspic.offset_x_mm*DIST_IMP_MM) + 54400 - (int32_t)(5*DIST_IMP_MM);

		printf("Axis X range is [%ld %ld] mm\n\r",
		 (int32_t)(maindspic.pos_x_min_imp/DIST_IMP_MM),
		 (int32_t)(maindspic.pos_x_max_imp/DIST_IMP_MM));
	}

 end:
	printf("Done\n\r");
}

prog_char str_axis_mode2_arg0[] = "axis_x";
parse_pgm_token_string_t cmd_axis_mode2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_axis_mode2_result, arg0, str_axis_mode2_arg0);
prog_char str_axis_mode2_arg1[] = "set#offset";
parse_pgm_token_string_t cmd_axis_mode2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_axis_mode2_result, arg1, str_axis_mode2_arg1);
parse_pgm_token_num_t cmd_axis_mode2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_axis_mode2_result, arg3, INT16);

prog_char help_axis_mode2[] = "Axis X positioning commands (mode2)";
parse_pgm_inst_t cmd_axis_mode2 = {
	.f = cmd_axis_mode2_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_axis_mode2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_axis_mode2_arg0, 
		(prog_void *)&cmd_axis_mode2_arg1, 
		(prog_void *)&cmd_axis_mode2_arg3,
		NULL,
	},
};

/* show */

/* this structure is filled when cmd_axis_mode2_show is parsed successfully */
struct cmd_axis_mode2_show_result {
	fixed_string_t show;
};

prog_char str_axis_mode2_show_arg0[] = "show";
parse_pgm_token_string_t cmd_axis_mode2_show_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_axis_mode2_show_result, show, str_axis_mode2_show_arg0);

prog_char help_axis_mode2_show[] = "Show Axis X offset and range";
parse_pgm_inst_t cmd_axis_mode2_show = {
	.f = cmd_axis_mode2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_axis_mode2_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_axis_mode2_arg0, 
		(prog_void *)&cmd_axis_mode2_arg1, 
		(prog_void *)&cmd_axis_mode2_show_arg3,
		NULL,
	},
};

