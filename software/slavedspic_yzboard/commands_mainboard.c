/*
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
 *
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *	Based on: commands_mainboard.c,v 1.8 2009/05/27 20:04:07 zer0 Exp
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
#include <encoders_dspic.h>
#include <time.h>

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
			slavedspic.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			slavedspic.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\n\r"), 
				 (DO_ENCODERS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\n\r"), 
				 (DO_CS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\n\r"), 
				 (DO_BD & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("power is %s\n\r"), 
				 (DO_POWER & slavedspic.flags) ? "on":"off");
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
		slavedspic.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
		 // TODO	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x00);
		}
		slavedspic.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\n\r"), res->arg1, 
		      (bit & slavedspic.flags) ? "on":"off");
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

/* Application functions */

/* TODO */

#ifdef TODO

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
			//NOTICE(E_USER_APP, "Trajectory end at %f deg",
			//	 (double)(encoders_dspic_get_value(enc_id)/ANGLE_IMP_DEG));
			NOTICE(E_USER_APP, "Positioning ends OK");
			return 1;
		}

		/* test blocking */
		ret = bd_get(&csb->bd);
		if(ret){
			if(csb == &slavedspic.alpha)
				hard_stop(csb, ALPHA_ENCODER);
			else
				hard_stop(csb, BETA_ENCODER);
				
			pid_reset(&csb->pid);
			bd_reset(&csb->bd);

			ERROR(E_USER_APP, "Positioning end BLOCKING!!");
			return -1;
		}
	}
}



#define AUTOPOS_SPEED	1
void angle_autopos(struct cs_block *csb, uint16_t ax12_id, void * enc_id, uint8_t reverse)
{
	int32_t val;
	int16_t angle_consign;
	int8_t ret;

	/* disable position bd */
	slavedspic.position_bd = 0;

	/* set calibrate angle consign */
	if(ax12_id == ALPHA_AX12)
		angle_consign = 102;
	else
		angle_consign = 62;


	/* set new speed and gains */
	quadramp_set_1st_order_vars(&csb->qr, AUTOPOS_SPEED, AUTOPOS_SPEED);
	pid_set_gains(&csb->pid, 10000, 0, 20000);
	DEBUG(E_USER_APP, "New speed is %d", (int16_t)AUTOPOS_SPEED);

	/* goto zero with cs */
	slavedspic.flags |= DO_CS;
	if(reverse)
		cs_set_consign(&csb->cs, (int32_t)(angle_consign*ANGLE_IMP_DEG));
	else
		cs_set_consign(&csb->cs, (int32_t)(-(angle_consign*ANGLE_IMP_DEG)));
	
	DEBUG(E_USER_APP, "Goto zero, reverse = %d", reverse);	

	/* wait end blocking */
	while(!bd_get(&csb->bd));	
	DEBUG(E_USER_APP, "End blocking");			

	/* brake on and disable cs */
	slavedspic.flags &= (~DO_CS);
	ax12_set_and_save((void *)ax12_id, 0);
	BRAKE_ON();
	quadramp_reset(&csb->qr);
	pid_reset(&csb->pid);
	bd_reset(&csb->bd);

	/* push a little and reset encoder */
	ax12_set_and_save((void *)ax12_id, -350);
	wait_ms(100);
	encoders_dspic_set_value(enc_id, 0);
	ax12_set_and_save((void *)ax12_id, 0);
	csb->qr.previous_var = 0;
	csb->qr.previous_out = encoders_dspic_get_value(enc_id);
	cs_set_consign(&csb->cs, encoders_dspic_get_value(enc_id));

	DEBUG(E_USER_APP, "Encoder reset to zero");	


	/* goto opposite with cs */
	slavedspic.flags |= DO_CS;
	if(reverse)
		cs_set_consign(&csb->cs, (int32_t)(-(angle_consign*ANGLE_IMP_DEG)));
	else
		cs_set_consign(&csb->cs, (int32_t)(angle_consign*ANGLE_IMP_DEG));

	DEBUG(E_USER_APP, "Goto opposite, reverse = %d", reverse);	
	
	/* wait end blocking */
	while(!bd_get(&csb->bd));	
	DEBUG(E_USER_APP, "End blocking");			

	/* brake on and disable cs */
	slavedspic.flags &= (~DO_CS);
	ax12_set_and_save((void *)ax12_id, 0);
	BRAKE_ON();
	quadramp_reset(&csb->qr);
	pid_reset(&csb->pid);
	bd_reset(&csb->bd);

	/* push a little and get encoder */
	ax12_set_and_save((void *)ax12_id, 40);
	wait_ms(100);
	val = encoders_dspic_get_value(enc_id);
	ax12_set_and_save((void *)ax12_id, 0);

	/* set encoder offset */
	encoders_dspic_set_value(enc_id, (int32_t)(val/2));
	csb->qr.previous_var = 0;
	csb->qr.previous_out = encoders_dspic_get_value(enc_id);
	cs_set_consign(&csb->cs, encoders_dspic_get_value(enc_id));

	DEBUG(E_USER_APP, "Encoder set to %ld of %ld impulses", (int32_t)(val/2), (int32_t)val);	

	/* calcule limit ranges */
	if(ax12_id == ALPHA_AX12){
		slavedspic.alpha_pos_max_imp = (int32_t)((val/2)-(ANGLE_IMP_DEG));
		slavedspic.alpha_pos_min_imp = -(int32_t)((val/2)-(ANGLE_IMP_DEG));
		printf("Alpha range is [%ld %ld] deg\n\r", 
			(int32_t)(slavedspic.alpha_pos_max_imp/ANGLE_IMP_DEG),
			(int32_t)(slavedspic.alpha_pos_min_imp/ANGLE_IMP_DEG));
	}
	else{
		slavedspic.beta_pos_max_imp = (int32_t)((val/2)-(ANGLE_IMP_DEG));
		slavedspic.beta_pos_min_imp = -(int32_t)((val/2)-(ANGLE_IMP_DEG));
		printf("Beta range is [%ld %ld] deg\n\r",			
			(int32_t)(slavedspic.beta_pos_max_imp/ANGLE_IMP_DEG),
			(int32_t)(slavedspic.beta_pos_min_imp/ANGLE_IMP_DEG));
	}	
	
	/* restore normal speed */
	quadramp_set_1st_order_vars(&csb->qr, NORMAL_SPEED, NORMAL_SPEED);
	pid_set_gains(&csb->pid, 10000, 800, 20000);
	DEBUG(E_USER_APP, "New speed is %d", (int16_t)NORMAL_SPEED);

	/* disable position bd */
	slavedspic.position_bd = 1;

	/* goto new zero with cs */
	cs_set_consign(&csb->cs, 0);
	slavedspic.flags |= DO_CS;
	DEBUG(E_USER_APP, "Goto new zero");

	/* wait trajectory end */
	ret = wait_pos_end(csb);

	/* set calibrate flag */
	if(ret == 1){
		if(csb == &slavedspic.alpha)
			slavedspic.alpha_calib = 1;
		else
			slavedspic.beta_calib = 1;
	}

	DEBUG(E_USER_APP, "Calibration ends");	
}

void angle_set(struct cs_block *csb, int32_t angle_deg)
{
	int32_t val_imp;

	/* check if angle is calibrated */
	if(csb == &slavedspic.alpha){
		if(!slavedspic.alpha_calib){
			printf("Angle is not calibrated yet\n\r");
			return;	
		}
	}	
	else{
		if(!slavedspic.beta_calib){
			printf("Angle is not calibrated yet\n\r");
			return;	
		}
	}		


	/* value in deg */
	val_imp = (int32_t)(angle_deg*ANGLE_IMP_DEG);

	/* check valid range */
	if(csb == &slavedspic.alpha){
		if((val_imp < slavedspic.alpha_pos_min_imp) || (val_imp > slavedspic.alpha_pos_max_imp)){
			printf("Consign out of range\n\r");
			return;	
		}
	}
	else{
		if((val_imp < slavedspic.beta_pos_min_imp) || (val_imp > slavedspic.beta_pos_max_imp)){
			printf("Consign out of range\n\r");
			return;	
		}
	}	

	/* set consign */
	cs_set_consign(&csb->cs, (int32_t)(angle_deg*ANGLE_IMP_DEG));
	DEBUG(E_USER_APP, "Set angle consign to %ld deg (%ld imp)",
		angle_deg, (int32_t)(angle_deg*ANGLE_IMP_DEG));

	/* wait trajectory end */
	wait_pos_end(csb);
}

double angle_get(void * enc_id)
{
	/* check if angle is calibrated */
	if(enc_id == ALPHA_ENCODER){
		if(!slavedspic.alpha_calib){
			printf("Angle is not calibrated yet\n\r");
			return 0;	
		}
	}	
	else{
		if(!slavedspic.beta_calib){
			printf("Angle is not calibrated yet\n\r");
			return 0;	
		}
	}

	return ((double)(encoders_dspic_get_value(enc_id)/ANGLE_IMP_DEG));
}



/**********************************************************/
/* Alpha mode1 */

/* this structure is filled when cmd_alpha_mode1 is parsed successfully */
struct cmd_alpha_mode1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_alpha_mode1 is parsed successfully */
static void cmd_alpha_mode1_parsed(void *parsed_result, void *data)
{
	struct cmd_alpha_mode1_result *res = parsed_result;
	double ret;

	if (!strcmp_P(res->arg1, "autopos")) {
		angle_autopos(&slavedspic.alpha, ALPHA_AX12, ALPHA_ENCODER, 0);
	}
	else if (!strcmp_P(res->arg1, "get")) {
		ret = angle_get(ALPHA_ENCODER);
		printf("alpha angle = %.3f deg\n\r", ret);
	}

	printf("Done\n\r");
}

prog_char str_alpha_mode1_arg0[] = "alpha";
parse_pgm_token_string_t cmd_alpha_mode1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_alpha_mode1_result, arg0, str_alpha_mode1_arg0);
prog_char str_alpha_mode1_arg1[] = "autopos#get";
parse_pgm_token_string_t cmd_alpha_mode1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_alpha_mode1_result, arg1, str_alpha_mode1_arg1);

prog_char help_alpha_mode1[] = "Alpha positioning commands (mode1)";
parse_pgm_inst_t cmd_alpha_mode1 = {
	.f = cmd_alpha_mode1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_alpha_mode1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_alpha_mode1_arg0, 
		(prog_void *)&cmd_alpha_mode1_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Alpha mode2 */

/* this structure is filled when cmd_alpha_mode2 is parsed successfully */
struct cmd_alpha_mode2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg3;
};

/* function called when cmd_alpha_mode2 is parsed successfully */
static void cmd_alpha_mode2_parsed(void *parsed_result, void *data)
{
	struct cmd_alpha_mode2_result *res = parsed_result;

	if (!strcmp_P(res->arg1, "set")) {
		angle_set(&slavedspic.alpha, res->arg3);
		printf("Done\n\r");
	}
}

prog_char str_alpha_mode2_arg0[] = "alpha";
parse_pgm_token_string_t cmd_alpha_mode2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_alpha_mode2_result, arg0, str_alpha_mode2_arg0);
prog_char str_alpha_mode2_arg1[] = "set";
parse_pgm_token_string_t cmd_alpha_mode2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_alpha_mode2_result, arg1, str_alpha_mode2_arg1);
parse_pgm_token_num_t cmd_alpha_mode2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_alpha_mode2_result, arg3, INT16);

prog_char help_alpha_mode2[] = "Alpha positioning commands (mode2)";
parse_pgm_inst_t cmd_alpha_mode2 = {
	.f = cmd_alpha_mode2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_alpha_mode2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_alpha_mode2_arg0, 
		(prog_void *)&cmd_alpha_mode2_arg1, 
		(prog_void *)&cmd_alpha_mode2_arg3,
		NULL,
	},
};


/**********************************************************/
/* Beta mode1 */

/* this structure is filled when cmd_beta_mode1 is parsed successfully */
struct cmd_beta_mode1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_beta_mode1 is parsed successfully */
static void cmd_beta_mode1_parsed(void *parsed_result, void *data)
{
	struct cmd_beta_mode1_result *res = parsed_result;
	double ret;

	if (!strcmp_P(res->arg1, "autopos")) {
		angle_autopos(&slavedspic.beta, BETA_AX12, BETA_ENCODER, 0);
	}
	else if (!strcmp_P(res->arg1, "get")) {
		ret = angle_get(BETA_ENCODER);
		printf("beta angle = %.3f deg\n\r", ret);
	}

	printf("Done\n\r");
}

prog_char str_beta_mode1_arg0[] = "beta";
parse_pgm_token_string_t cmd_beta_mode1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beta_mode1_result, arg0, str_beta_mode1_arg0);
prog_char str_beta_mode1_arg1[] = "autopos#get";
parse_pgm_token_string_t cmd_beta_mode1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beta_mode1_result, arg1, str_beta_mode1_arg1);

prog_char help_beta_mode1[] = "Beta positioning commands (mode1)";
parse_pgm_inst_t cmd_beta_mode1 = {
	.f = cmd_beta_mode1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beta_mode1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beta_mode1_arg0, 
		(prog_void *)&cmd_beta_mode1_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Beta mode2 */

/* this structure is filled when cmd_beta_mode2 is parsed successfully */
struct cmd_beta_mode2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg3;
};

/* function called when cmd_beta_mode2 is parsed successfully */
static void cmd_beta_mode2_parsed(void *parsed_result, void *data)
{
	struct cmd_beta_mode2_result *res = parsed_result;

	if (!strcmp_P(res->arg1, "set")) {
		angle_set(&slavedspic.beta, (double)res->arg3);
		printf("Done\n\r");
	}
}

prog_char str_beta_mode2_arg0[] = "beta";
parse_pgm_token_string_t cmd_beta_mode2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beta_mode2_result, arg0, str_beta_mode2_arg0);
prog_char str_beta_mode2_arg1[] = "set";
parse_pgm_token_string_t cmd_beta_mode2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beta_mode2_result, arg1, str_beta_mode2_arg1);
parse_pgm_token_num_t cmd_beta_mode2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_beta_mode2_result, arg3, INT16);

prog_char help_beta_mode2[] = "Beta positioning commands (mode2)";
parse_pgm_inst_t cmd_beta_mode2 = {
	.f = cmd_beta_mode2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beta_mode2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beta_mode2_arg0, 
		(prog_void *)&cmd_beta_mode2_arg1, 
		(prog_void *)&cmd_beta_mode2_arg3,
		NULL,
	},
};



#endif


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

void axis_x_set(struct cs_block *csb, int32_t dist_mm)
{
	int32_t val_imp;

	/* check if axis is calibrated */	
	if(!maindspic.calib_x){
		printf("Axis is not calibrated yet\n\r");
		return;	
	}		

	/* value in pulses */
	val_imp = (int32_t)(dist_mm*DIST_IMP_MM);

	/* check range */
	if((val_imp < maindspic.pos_x_min_imp) || (val_imp > maindspic.pos_x_max_imp)){
		printf("Consign out of range\n\r");
		return;	
	}

	/* set consign */
	cs_set_consign(&csb->cs, (int32_t)((dist_mm-maindspic.offset_x_mm)*DIST_IMP_MM));
	DEBUG(E_USER_APP, "Set angle consign to %ld deg (%ld imp)",
		dist_mm, (int32_t)(dist_mm*DIST_IMP_MM));

	/* wait trajectory end */
	wait_pos_end(csb);
}

double axis_x_get(void * enc_id)
{
	/* check if axis is calibrated */
	if(!maindspic.){
		printf("Axis is not calibrated yet\n\r");
		return 0.0;	
	}		

	return ((double)(maindspic.offset_x_mm + (encoders_dspic_get_value(enc_id)/DIST_IMP_MM)));
}


/**********************************************************/
/* Axis X mode1 */

/* this structure is filled when cmd_axis_x_mode1 is parsed successfully */
struct cmd_axis_x_mode1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_axis_x_mode1 is parsed successfully */
static void cmd_axis_x_mode1_parsed(void *parsed_result, void *data)
{
	struct cmd_axis_x_mode1_result *res = parsed_result;
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

prog_char str_axis_x_mode1_arg0[] = "axis_x";
parse_pgm_token_string_t cmd_axis_x_mode1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_axis_x_mode1_result, arg0, str_axis_x_mode1_arg0);
prog_char str_axis_x_mode1_arg1[] = "autopos#get";
parse_pgm_token_string_t cmd_axis_x_mode1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_axis_x_mode1_result, arg1, str_axis_x_mode1_arg1);

prog_char help_axis_x_mode1[] = "Axis X positioning commands (mode1)";
parse_pgm_inst_t cmd_axis_x_mode1 = {
	.f = cmd_axis_x_mode1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_axis_x_mode1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_axis_x_mode1_arg0, 
		(prog_void *)&cmd_axis_x_mode1_arg1, 
		NULL,
	},
};

/**********************************************************/
/* axis_x mode2 */

/* this structure is filled when cmd_axis_x_mode2 is parsed successfully */
struct cmd_axis_x_mode2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg3;
};

/* function called when cmd_axis_x_mode2 is parsed successfully */
static void cmd_axis_x_mode2_parsed(void *parsed_result, void *show)
{
	struct cmd_axis_x_mode2_result *res = parsed_result;

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

prog_char str_axis_x_mode2_arg0[] = "axis_z#axis_y";
parse_pgm_token_string_t cmd_axis_x_mode2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_axis_x_mode2_result, arg0, str_axis_x_mode2_arg0);
prog_char str_axis_x_mode2_arg1[] = "set#offset";
parse_pgm_token_string_t cmd_axis_x_mode2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_axis_x_mode2_result, arg1, str_axis_x_mode2_arg1);
parse_pgm_token_num_t cmd_axis_x_mode2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_axis_x_mode2_result, arg3, INT16);

prog_char help_axis_x_mode2[] = "Axis positioning commands (mode2)";
parse_pgm_inst_t cmd_axis_x_mode2 = {
	.f = cmd_axis_x_mode2_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_axis_x_mode2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_axis_x_mode2_arg0, 
		(prog_void *)&cmd_axis_x_mode2_arg1, 
		(prog_void *)&cmd_axis_x_mode2_arg3,
		NULL,
	},
};

/* show */

/* this structure is filled when cmd_axis_x_mode2_show is parsed successfully */
struct cmd_axis_x_mode2_show_result {
	fixed_string_t show;
};

prog_char str_axis_x_mode2_show_arg0[] = "show";
parse_pgm_token_string_t cmd_axis_x_mode2_show_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_axis_x_mode2_show_result, show, str_axis_x_mode2_show_arg0);

prog_char help_axis_x_mode2_show[] = "Show Axis X offset and range";
parse_pgm_inst_t cmd_axis_x_mode2_show = {
	.f = cmd_axis_x_mode2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_axis_x_mode2_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_axis_x_mode2_arg0, 
		(prog_void *)&cmd_axis_x_mode2_arg1, 
		(prog_void *)&cmd_axis_x_mode2_show_arg3,
		NULL,
	},
};