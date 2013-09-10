/*  
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
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

#ifndef __MAIN_H__
#define __MAIN_H__

#include <aversive.h>#include <aversive/error.h>
#include <time.h>#include <rdline.h>
#include <encoders_dspic.h>#include <dac_mc.h>
#include <ax12.h>
#include <pid.h>#include <quadramp.h>#include <control_system_manager.h>#include <blocking_detection_manager.h>

/* LEDS utils */
#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)


#define LED1_ON() 		cbi(LATC, 9)
#define LED1_OFF() 		sbi(LATC, 9)
#define LED1_TOGGLE() 	LED_TOGGLE(LATC, 9)

/** Motor systems */
#define BRAKE_ON()	 do { ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x00); _LATA7 = 0;} while(0);
#define BRAKE_OFF()	 do { ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x01); _LATA7 = 1;} while(0);

#define ENC_Z_IMP_REV 3600
#define GEAR_Z_MM_REV 1.0
#define DIST_Z_IMP_MM ((ENC_Z_IMP_REV *4) / GEAR_Z_MM_REV)

#define ENC_Y_IMP_MM  10
#define DIST_Y_IMP_MM ((ENC_Y_IMP_MM *4))

#define ENCODER_Z		((void *)1)
#define ENCODER_Y    ((void *)2)

#define AX12_Y			1
#define DAC_MC_Z     ((void *)&gen.dac_mc_left)

/** ERROR NUMS */
#define E_USER_APP         194
#define E_USER_SENSOR      195
#define E_USER_CS          196
#define E_USER_AX12        197

/** EVENTS PRIORITY */
#define LED_PRIO           170
#define TIME_PRIO          160
#define SENSOR_PRIO        120
#define CS_PRIO            100

/** EVENTS PERIOD */
#define LED_PERIOD         100000L
#define SENSOR_PERIOD      10000L
#define CS_PERIOD          5000L

/** LOGS */
#define NB_LOGS 10

/* OTHER STUFF */
#define NORMAL_SPEED	3

/* generic to all boards */
struct genboard{
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* ax12 interface */
	AX12 ax12;

	/* brushless motors */	struct dac_mc dac_mc_left;

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};


/* application specific */
struct cs_block {
	uint8_t on;
  	struct cs cs;
  	struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

struct slavedspic {

#define DO_ENCODERS  1
#define DO_CS        2
#define DO_BD        4
#define DO_POWER     8

	/* misc flags */
	uint8_t flags;                

	/* control systems */
  	struct cs_block y;
  	struct cs_block z;

	/* pantil status */
	int32_t y_pos;
	int32_t y_pos_max_imp;
	int32_t y_pos_min_imp;
	int8_t  y_calib;

	int32_t z_pos;
	int32_t z_pos_max_imp;
	int32_t z_pos_min_imp;
	int8_t  z_calib;

	int8_t position_bd;

	volatile int16_t ax12_y_speed;
};


extern struct genboard gen;
extern struct slavedspic slavedspic;

#define WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
	if (__ret)					      \
		DEBUG(E_USER_STRAT, "cond is true at line %d",\
		      __LINE__);			      \
	else						      \
		DEBUG(E_USER_STRAT, "timeout at line %d",     \
		      __LINE__);			      \
							      \
        __ret;                                                \
})

#endif
