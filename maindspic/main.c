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

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <configuration_bits_config.h>

#include <uart.h>

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
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"

struct genboard gen;
struct maindspic maindspic;

void hard_stop(struct cs_block *csb, void * enc_id);

/***********************/

void do_led_blink(void *dummy)
{
#if 0	 /* loopback beacon uart */
	char c;
	c = uart_recv(1);
	uart_send(1,c);
#endif
	
#if 1 /* simple blink */
	LED1_TOGGLE();

	/* test blocking */
	if(maindspic.position_bd){
		if(bd_get(&maindspic.axis_x.bd)){
			hard_stop(&maindspic.axis_x, X_ENCODER);
			pid_reset(&maindspic.axis_x.pid);
			bd_reset(&maindspic.axis_x.bd);
			ERROR(E_USER_APP, "Positioning end BLOCKING!!");
		}
	}

#endif
}

static void main_timer_interrupt(void)
{
	sei();
	scheduler_interrupt();	
}

void timer_init(void)
{
	/* Init Timer1 */
	unsigned int match_value;
	ConfigIntTimer1(T1_INT_ON);
	WriteTimer1(0);
	match_value = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP &
              T1_PS_1_1 & T1_SYNC_EXT_OFF &
              T1_SOURCE_INT, match_value);
}

/* Timer 1 interrupt handler */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
  /* Interrupt Service Routine code goes here */
  IFS0bits.T1IF=0;
    
  /* Execute scheduler interrupt */
  main_timer_interrupt();
}

void io_pins_init(void)
{
	
	/* leds */
	_TRISA4 = 0;	// MAIN_LED1
	_TRISA8 = 0;	// MAIN_LED2
	_TRISC2 = 0;	// MAIN_LED3
	_TRISC8 = 0;	// MAIN_LED4
	
	/* brushless motors */	
	_TRISB10 = 0; 	// X_MOT_REV	_TRISB11 = 0; 	// X_MOT_BREAK
	_LATB11 = 0;
	
	/* encoders */	
	_QEA1R 	= 21;	// QEA1 <- RP21(RC5) <- R_ENC_CHA
	_TRISC5 = 1;	
	_QEB1R 	= 20;	// QEB1 <- RP20(RC4) <- R_ENC_CHB
	_TRISC4	= 1;
	
	/* uarts */
	_U1RXR 	= 8;	// U1RX <- RP8(RB8) <- MAIN_UART_RX
	_TRISB8 = 1;	// U1RX is input
  	_RP7R 	= 3;	// U1TX -> RP7(RB7) -> MAIN_UART_TX
	_TRISB7	= 0;	// U1TX is output
	
	_U2RXR 	= 6;	// U2RX <- RP6(RB6) <- SLAVE_UART_RX
	_TRISB6 = 1;	// U2RX is input  	_RP5R 	= 5;	// U2TX -> RP5(RB5) -> SLAVE_UART_TX	_TRISB5	= 0;	// U2TX is output
}


int main(void)
{

	/* remapeable pins */
	io_pins_init();
	
	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();

	memset(&gen, 0, sizeof(gen));
	memset(&maindspic, 0, sizeof(maindspic));
	maindspic.flags = DO_ENCODERS | DO_POWER | DO_BD;
	
	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* ENCODERS */
	encoders_dspic_init();

	/* TIMER */
	timer_init();

	/* DAC_MC */
	dac_mc_channel_init(&gen.dac_mc_x, 1, CHANNEL_R,										DAC_MC_MODE_SIGNED,										&LATB, 10, NULL, 0);
	dac_mc_set(&gen.dac_mc_x, 0);


	/* SCHEDULER */
	scheduler_init();

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						1000000L / SCHEDULER_UNIT, 
						LED_PRIO);

	/* all cs management */
	dspic_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

	/* TIME */
	time_init(TIME_PRIO);

	wait_ms(500);

	/* strat */
 	gen.logs[0] = E_USER_APP;
 	gen.log_level = 5;

	sei();

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("I program in solder\r\n"));

	cmdline_interact();

	return 0;
}










