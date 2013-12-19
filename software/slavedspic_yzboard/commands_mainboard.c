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

void hard_stop(struct cs_block *csb, void * enc_id)
{
	cs_set_consign(&csb->cs, encoders_dspic_get_value(enc_id));

	csb->qr.previous_var = 0;
	csb->qr.previous_out = encoders_dspic_get_value(enc_id);
}

int8_t test_pos_end(struct cs_block *csb, void * enc_id)
{
	uint8_t ret=0;

	/* test traj end */
	if((cs_get_consign(&csb->cs) == cs_get_filtered_consign(&csb->cs)) &&
		(ABS(cs_get_error(&csb->cs)) < 10) ){
		ret = 1;
		NOTICE(E_USER_APP, "Positioning ends OK");
		return 1;
	}

	/* test FC */
	if((enc_id == ENCODER_Y) && (sensor_get(S_Y_FC_L)||sensor_get(S_Y_FC_R))) {

		hard_stop(csb, enc_id);
		pid_reset(&csb->pid);
		bd_reset(&csb->bd);

		NOTICE(E_USER_APP, "Positioning ends at FC");
		return -1;
	}

	if((enc_id == ENCODER_Z) && (sensor_get(S_Z_FC_UP)||sensor_get(S_Z_FC_DOWN))) {

		hard_stop(csb, enc_id);
		pid_reset(&csb->pid);
		bd_reset(&csb->bd);

		NOTICE(E_USER_APP, "Positioning ends at FC");
		return -1;
	}	

	/* test blocking */
	ret = bd_get(&csb->bd);
	if(ret){

		hard_stop(csb, enc_id);
		pid_reset(&csb->pid);
		bd_reset(&csb->bd);

		ERROR(E_USER_APP, "Positioning ends BLOCKING!!");
		return -1;
	}

	return ret;
}

int8_t wait_pos_end(struct cs_block *csb, void * enc_id) {
	uint8_t ret=0;
	while(ret == 0)
		ret = test_pos_end(csb, enc_id);

	return ret;
}


#define AUTOPOS_Y_SPEED	600
#define AUTOPOS_Z_SPEED	15000

#if 0
void axis_autopos(void * enc_id)
{
	/* goto calib sensor side, left and down */
	enc_id == ENCODER_Y? pwm_mc_set(PWM_MC_Y, -Y_PWM_VALUE_MAX): dac_mc_set(DAC_MC_Z, -Z_DAC_VALUE_MAX);
	DEBUG(E_USER_APP, "Goto calib sensor side (left and down)");
	

	
	/* wait for sensor activation and stop */
	while(!(enc_id == ENCODER_Y? sensor_get(S_Y_CALIB): sensor_get(S_Z_CALIB)) &&
			!(enc_id == ENCODER_Y? sensor_get(S_Y_FC_L): sensor_get(S_Z_FC_DOWN)));

	enc_id == ENCODER_Y? pwm_mc_set(PWM_MC_Y, 0): dac_mc_set(DAC_MC_Z, 0);
	DEBUG(E_USER_APP, "Calib sensor reached, stoped");
	
	if(enc_id == ENCODER_Y? sensor_get(S_Y_FC_L): sensor_get(S_Z_FC_DOWN)) {
		ERROR(E_USER_APP, "FC reached");
		return;
	}

	/* enable calib event goto opposite side */
	enc_id == ENCODER_Y? sensor_axis_y_enable_calib(): sensor_axis_z_enable_calib();
	enc_id == ENCODER_Y? pwm_mc_set(PWM_MC_Y, AUTOPOS_Y_SPEED): dac_mc_set(DAC_MC_Z, AUTOPOS_Z_SPEED);
	DEBUG(E_USER_APP, "Calib edge event enabled, goto reach it");

	while( (enc_id == ENCODER_Y? sensor_get(S_Y_CALIB): sensor_get(S_Z_CALIB)) &&
			!(enc_id == ENCODER_Y? sensor_get(S_Y_FC_R): sensor_get(S_Z_FC_UP)));

	enc_id == ENCODER_Y? pwm_mc_set(PWM_MC_Y, 0): dac_mc_set(DAC_MC_Z, 0);
	enc_id == ENCODER_Y? sensor_axis_y_disable_calib(): sensor_axis_z_disable_calib();

	if(enc_id == ENCODER_Y? sensor_get(S_Y_FC_R): sensor_get(S_Z_FC_UP)) {
		ERROR(E_USER_APP, "FC reached");
		return;
	}

	DEBUG(E_USER_APP, "Calib sensor desactivated, encoders should be reseted");



	DEBUG(E_USER_APP, "Calibration ends");	
}
#endif

void axis_yz_autopos(void) 
{
	int8_t ret=0, ret2=0;

	/* check left Y FC */
	if(sensor_get(S_Y_FC_L)) {
		NOTICE(E_USER_APP, "Axis Y has reach lelft FC, never must be reached!!");

		NOTICE(E_USER_APP, "Trying to calibrate");
		pwm_mc_set(PWM_MC_Y, Y_PWM_VALUE_MAX);

		/* calib sensor ON */
		ret = WAIT_COND_OR_TIMEOUT(sensor_get(S_Y_CALIB), 10000);
		if(ret == 0) {
			pwm_mc_set(PWM_MC_Y, 0);
			ERROR(E_USER_APP, "Calib sensor Y not found in path");
			return;
		} 

		/* calib sensor OFF */
		ret = WAIT_COND_OR_TIMEOUT(!sensor_get(S_Y_CALIB), 10000);
		if(ret == 0) {
			pwm_mc_set(PWM_MC_Y, 0);
			ERROR(E_USER_APP, "Calib sensor Y turn OFF timeout");
			return;
		} 

		/* continua a bit and stotp */
		wait_ms(2000);
		pwm_mc_set(PWM_MC_Y, 0);
	}

	/* check down Z FC */
	if(sensor_get(S_Z_FC_DOWN)) {
		NOTICE(E_USER_APP, "Axis Z has reach down FC, never must be reached!!");

		NOTICE(E_USER_APP, "Trying to calibrate");
		dac_mc_set(DAC_MC_Z, Z_DAC_VALUE_MAX);

		/* calib sensor ON */
		ret = WAIT_COND_OR_TIMEOUT(sensor_get(S_Z_CALIB), 10000);
		if(ret == 0) {
			dac_mc_set(DAC_MC_Z, 0);
			ERROR(E_USER_APP, "Calib sensor Z not found in path");
			return;
		} 

		/* calib sensor OFF */
		ret = WAIT_COND_OR_TIMEOUT(!sensor_get(S_Z_CALIB), 10000);
		if(ret == 0) {
			dac_mc_set(DAC_MC_Z, 0);
			ERROR(E_USER_APP, "Calib sensor Z not turn OFF");
			return;
		} 

		/* continua a bit and stotp */
		wait_ms(2000);
		dac_mc_set(DAC_MC_Z, 0);
	}

	DEBUG(E_USER_APP, "Ready for axes calibration");

	/* AVOID CRASSING */
	dac_mc_set(DAC_MC_Z, Z_DAC_VALUE_MAX);
	WAIT_COND_OR_TIMEOUT(sensor_get(S_Z_FC_UP), 3000);
	dac_mc_set(DAC_MC_Z, 0);

	pwm_mc_set(PWM_MC_Y, -Y_PWM_VALUE_MAX); 
	DEBUG(E_USER_APP, "Goto calib sensor side (left and down)");
	
	/* wait for sensor activation and stop */
	while(!sensor_get(S_Y_CALIB)  && !sensor_get(S_Y_FC_L));

	/* goto calib sensor side, left and down */
	pwm_mc_set(PWM_MC_Y, -Y_PWM_VALUE_MAX); 
	dac_mc_set(DAC_MC_Z, -Z_DAC_VALUE_MAX);
	DEBUG(E_USER_APP, "Goto calib sensor side (left and down)");
	
	/* wait for sensor activation and stop */
	while( (!sensor_get(S_Y_CALIB) || !sensor_get(S_Z_CALIB)) &&
			!sensor_get(S_Y_FC_L) && 
			!sensor_get(S_Z_FC_DOWN)) {
	
		if(sensor_get(S_Y_CALIB))
			pwm_mc_set(PWM_MC_Y, 0);

		if(sensor_get(S_Z_CALIB))
			dac_mc_set(DAC_MC_Z, 0);
	}

	pwm_mc_set(PWM_MC_Y, 0); 
	dac_mc_set(DAC_MC_Z, 0);
	
	if(sensor_get(S_Y_FC_L)) {
		ERROR(E_USER_APP, "Y left FC reached");
		return;
	}
	if(sensor_get(S_Z_FC_DOWN)) {
		ERROR(E_USER_APP, "Z down FC reached");
		return;
	}

	DEBUG(E_USER_APP, "Calib sensors reached, stopped");

	/* calib Y */
	sensor_axis_y_enable_calib();
	pwm_mc_set(PWM_MC_Y, AUTOPOS_Y_SPEED);
	DEBUG(E_USER_APP, "Calib Y edge event enabled, goto reach it");

	ret = WAIT_COND_OR_TIMEOUT(!sensor_get(S_Y_CALIB), 10000);

	pwm_mc_set(PWM_MC_Y, 0);
	sensor_axis_y_disable_calib();

	if(ret == 0) {
		ERROR(E_USER_APP, "Calib sensor Y turn OFF timeout");
		return;
	} 

	/* set Y range */
	slavedspic.y_offset_mm = 0;	slavedspic.y_pos_max_imp = (int32_t)((slavedspic.y_offset_mm*DIST_IMP_MM) + Y_POS_MAX_IMP);	slavedspic.y_pos_min_imp = (int32_t)((slavedspic.y_offset_mm*DIST_IMP_MM) + Y_POS_MIN_IMP);	printf("Axis Y range is [%ld %ld] mm\n\r",	 (int32_t)(slavedspic.y_pos_min_imp/DIST_IMP_MM),	 (int32_t)(slavedspic.y_pos_max_imp/DIST_IMP_MM));

	DEBUG(E_USER_APP, "Axis Y calibration ends");

	/* calib Z */
	sensor_axis_z_enable_calib();
	dac_mc_set(DAC_MC_Z, AUTOPOS_Z_SPEED);
	DEBUG(E_USER_APP, "Calib Z edge event enabled, goto reach it");

	ret = WAIT_COND_OR_TIMEOUT(!sensor_get(S_Z_CALIB), 10000);

	dac_mc_set(DAC_MC_Z, 0);
	sensor_axis_z_disable_calib();

	if(ret == 0) {
		ERROR(E_USER_APP, "Calib sensor Z turn OFF timeout");
		return;
	} 

	/* set Z range */
	slavedspic.z_offset_mm = 0;	slavedspic.z_pos_max_imp = (int32_t)((slavedspic.z_offset_mm*DIST_IMP_MM) + Z_POS_MAX_IMP);	slavedspic.z_pos_min_imp = (int32_t)((slavedspic.z_offset_mm*DIST_IMP_MM) + Z_POS_MIN_IMP);	printf("Axis Z range is [%ld %ld] mm\n\r",	 (int32_t)(slavedspic.z_pos_min_imp/DIST_IMP_MM),	 (int32_t)(slavedspic.z_pos_max_imp/DIST_IMP_MM));

	DEBUG(E_USER_APP, "Axis Z calibration ends");


	/* reset CS stuff and enable it */
	cs_set_consign(&slavedspic.y.cs, CALIB_Y_MM);
	cs_set_consign(&slavedspic.z.cs, CALIB_Z_MM);

	hard_stop(&slavedspic.y, ENCODER_Y);
	pid_reset(&slavedspic.y.pid);
	bd_reset(&slavedspic.y.bd);

	hard_stop(&slavedspic.z, ENCODER_Z);
	pid_reset(&slavedspic.z.pid);
	bd_reset(&slavedspic.z.bd);


	slavedspic.flags |= DO_CS;
	DEBUG(E_USER_APP, "Control Systems reset and enabled");

	/* wait trajectories end */
	ret = ret2 = 0;
	while( (ret == 0) || (ret2 == 0)) {
		if(!ret)
			ret = test_pos_end(&slavedspic.y, ENCODER_Y);
		if(!ret2)
			ret2 = test_pos_end(&slavedspic.z, ENCODER_Z);
	}


	/* set calibrate flag */
	if(ret2 == 1)
		slavedspic.z_calib = 1;
	else
		ERROR(E_USER_APP, "Axis Z CS fails");

	if(ret == 1)
		slavedspic.y_calib = 1;
	else
		ERROR(E_USER_APP, "Axis Y CS fails");


	DEBUG(E_USER_APP, "Calibration ends SUCESSFULY");	
}


void axis_z_set(int32_t dist_mm)
{
	int32_t val_imp;

	/* check if axis is calibrated */	
	if(!slavedspic.z_calib){
		printf("Axis is not calibrated yet\n\r");
		return;	
	}		

	/* value in pulses */
	val_imp = (int32_t)(dist_mm*DIST_IMP_MM);

	/* check range */
	if((val_imp < slavedspic.z_pos_min_imp) || (val_imp > slavedspic.z_pos_max_imp)){
		printf("Consign out of range\n\r");
		return;	
	}

	/* set consign */
	cs_set_consign(&slavedspic.z.cs, (int32_t)((dist_mm-slavedspic.z_offset_mm)*DIST_IMP_MM));
	DEBUG(E_USER_APP, "Set axis Z consign to %ld deg (%ld imp)",
		dist_mm, (int32_t)(dist_mm*DIST_IMP_MM));

	/* wait trajectory end */
	wait_pos_end(&slavedspic.z, ENCODER_Z);
}

void axis_y_set(int32_t dist_mm)
{
	int32_t val_imp;

	/* check if axis is calibrated */	
	if(!slavedspic.y_calib){
		printf("Axis is not calibrated yet\n\r");
		return;	
	}		

	/* value in pulses */
	val_imp = (int32_t)(dist_mm*DIST_IMP_MM);

	/* check range */
	if((val_imp < slavedspic.y_pos_min_imp) || (val_imp > slavedspic.y_pos_max_imp)){
		printf("Consign out of range\n\r");
		return;	
	}

	/* set consign */
	cs_set_consign(&slavedspic.y.cs, (int32_t)((dist_mm-slavedspic.y_offset_mm)*DIST_IMP_MM));
	DEBUG(E_USER_APP, "Set axis Y consign to %ld deg (%ld imp)",
		dist_mm, (int32_t)(dist_mm*DIST_IMP_MM));

	/* wait trajectory end */
	wait_pos_end(&slavedspic.y, ENCODER_Y);
}

double axis_z_get(void)
{
	/* check if axis is calibrated */
	if(!slavedspic.z_calib){
		printf("Axis is not calibrated yet\n\r");
		return 0.0;	
	}		

	return ((double)(slavedspic.z_offset_mm + (encoders_dspic_get_value(ENCODER_Z)/DIST_IMP_MM)));
}

double axis_y_get(void)
{
	/* check if axis is calibrated */
	if(!slavedspic.y_calib){
		printf("Axis is not calibrated yet\n\r");
		return 0.0;	
	}		

	return ((double)(slavedspic.y_offset_mm + (encoders_dspic_get_value(ENCODER_Y)/DIST_IMP_MM)));
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

	if (!strcmp_P(res->arg0, "axis_z")) {

		if (!strcmp_P(res->arg1, "autopos")) {
			printf("Not yet implemented, use 'axis_yz autopos' instead\n\r");
		}
		else if (!strcmp_P(res->arg1, "get")) {
			ret = axis_z_get();
			printf("axis_z pos = %.4f mm\n\r", ret);
		}
	}
	else if (!strcmp_P(res->arg0, "axis_y")) {

		if (!strcmp_P(res->arg1, "autopos")) {
			printf("Not yet implemented, use 'axis_yz autopos' instead\n\r");
		}
		else if (!strcmp_P(res->arg1, "get")) {
			ret = axis_y_get();
			printf("axis_y pos = %.4f mm\n\r", ret);
		}
	}
	else if (!strcmp_P(res->arg0, "axis_yz")) {

		if (!strcmp_P(res->arg1, "autopos")) {
			axis_yz_autopos();
		}
		else if (!strcmp_P(res->arg1, "get")) {
			printf("Not yet implemented, use 'axis_y/axis_z get' instead\n\r");
		}
	}

	printf("Done\n\r");
}

prog_char str_axis_mode1_arg0[] = "axis_y#axis_z#axis_yz";
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

	if (!strcmp_P(res->arg0, "axis_y"))
	{		

		if(!show){
			printf("Axis Y offset = %ld mm\n\r", slavedspic.y_offset_mm);
			printf("Axis Y range is [%ld %ld] mm\n\r",
			 (int32_t)(slavedspic.y_pos_min_imp/DIST_IMP_MM),
			 (int32_t)(slavedspic.y_pos_max_imp/DIST_IMP_MM));
			goto end;
		}

		if (!strcmp_P(res->arg1, "set")) {
			axis_y_set(res->arg3);
		}
		else if (!strcmp_P(res->arg1, "offset")) {

			if(res->arg3 < 0) {
				printf_P(PSTR("Bad arguments\r\n"));
				return;
			}

			slavedspic.y_offset_mm = res->arg3;
			slavedspic.y_pos_max_imp = (int32_t)((slavedspic.y_offset_mm*DIST_IMP_MM) + Y_POS_MAX_IMP);			slavedspic.y_pos_min_imp = (int32_t)((slavedspic.y_offset_mm*DIST_IMP_MM) + Y_POS_MIN_IMP);
	
			printf("Axis Y range is [%ld %ld] mm\n\r",
			 (int32_t)(slavedspic.y_pos_min_imp/DIST_IMP_MM),
			 (int32_t)(slavedspic.y_pos_max_imp/DIST_IMP_MM));
		}
	}
	else if (!strcmp_P(res->arg0, "axis_z"))
	{		
		if(!show){
			printf("Axis Z offset = %ld mm\n\r", slavedspic.z_offset_mm);
			printf("Axis Z range is [%ld %ld] mm\n\r",
			 (int32_t)(slavedspic.z_pos_min_imp/DIST_IMP_MM),
			 (int32_t)(slavedspic.z_pos_max_imp/DIST_IMP_MM));
			goto end;
		}

		if (!strcmp_P(res->arg1, "set")) {
			axis_z_set(res->arg3);
		}
		else if (!strcmp_P(res->arg1, "offset")) {
	
			if(res->arg3 < 0) {
				printf_P(PSTR("Bad arguments\r\n"));
				return;
			}
				
			slavedspic.z_offset_mm = res->arg3;
			slavedspic.z_pos_max_imp = (int32_t)((slavedspic.z_offset_mm*DIST_IMP_MM) + Z_POS_MAX_IMP);			slavedspic.z_pos_min_imp = (int32_t)((slavedspic.z_offset_mm*DIST_IMP_MM) + Z_POS_MIN_IMP);
	
			printf("Axis Y range is [%ld %ld] mm\n\r",
			 (int32_t)(slavedspic.z_pos_min_imp/DIST_IMP_MM),
			 (int32_t)(slavedspic.z_pos_max_imp/DIST_IMP_MM));
		}
	}


 end:
	printf("Done\n\r");
}

prog_char str_axis_mode2_arg0[] = "axis_y#axis_z";
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

prog_char help_axis_mode2_show[] = "Show Axis offset and range";
parse_pgm_inst_t cmd_axis_mode2_show = {
	.f = cmd_axis_mode2_parsed,  /* function to call */
	.data = (void *)0,      /* 2nd arg of func */
	.help_str = help_axis_mode2_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_axis_mode2_arg0, 
		(prog_void *)&cmd_axis_mode2_arg1, 
		(prog_void *)&cmd_axis_mode2_show_arg3,
		NULL,
	},
};


