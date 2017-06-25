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


#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)


#define LED1_ON() 	cbi(LATA, 4)
#define LED1_OFF() 	sbi(LATA, 4)
#define LED1_TOGGLE() 	LED_TOGGLE(LATA, 4)

#define LED2_ON() 	cbi(LATA, 8)
#define LED2_OFF() 	sbi(LATA, 8)
#define LED2_TOGGLE() 	LED_TOGGLE(LATA, 8)

#define LED3_ON() 	cbi(LATC, 2)
#define LED3_OFF() 	sbi(LATC, 2)
#define LED3_TOGGLE() 	LED_TOGGLE(LATC, 2)

#define LED4_ON() 	cbi(LATC, 8)
#define LED4_OFF() 	sbi(LATC, 8)
#define LED4_TOGGLE() 	LED_TOGGLE(LATC, 8)

#define BRAKE_ON()      do {_LATB11 = 0;} while(0)
#define BRAKE_OFF()     do {_LATB11 = 1;} while(0)

#define DIST_IMP_MM 	20.0

#define X_ENCODER       ((void *)1)
#define X_DAC           ((void *)&gen.dac_mc_x)

#define P_CONST	850
#define I_CONST	0
#define D_CONST	7000

#define NORMAL_SPEED	10

/** ERROR NUMS */
#define E_USER_APP         194
#define E_USER_SENSOR      195
#define E_USER_CS          196

#define LED_PRIO           170
#define TIME_PRIO          160
#define SENSOR_PRIO        120
#define CS_PRIO            100

#define CS_PERIOD 5000L

#define NB_LOGS 10

/* generic to all boards */
struct genboard{
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct dac_mc dac_mc_x;

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};

struct cs_block {
	uint8_t on;
  struct cs cs;
  struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* maindspic specific */
struct maindspic {

#define DO_ENCODERS  1
#define DO_CS        2
#define DO_BD        4
#define DO_POWER     8

	/* misc flags */
	uint8_t flags;                

	/* control systems */
  	struct cs_block axis_x;

	/* x positionning */
	int32_t pos_x;
	int32_t pos_x_max_imp;
	int32_t pos_x_min_imp;
	int32_t offset_x_mm;
	int8_t 	calib_x;

	int8_t 	position_bd;

	/* platform status */
	volatile int16_t speed_x;     /* current x axis speed */
	int32_t dac_x;                /* current x dac */
};


extern struct genboard gen;
extern struct maindspic maindspic;

///* start the bootloader */
//void bootloader(void);

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
