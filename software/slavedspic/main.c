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
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "ax12_user.h"
#include "cs.h"

struct genboard gen;
struct slavedspic slavedspic;

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
	if(slavedspic.position_bd){
		if(bd_get(&slavedspic.alpha.bd)){
			hard_stop(&slavedspic.alpha, ALPHA_ENCODER);
			pid_reset(&slavedspic.alpha.pid);
			bd_reset(&slavedspic.alpha.bd);
			ERROR(E_USER_APP, "Positioning end BLOCKING!!");
		}
		if(bd_get(&slavedspic.beta.bd)){
			hard_stop(&slavedspic.beta, BETA_ENCODER);
			pid_reset(&slavedspic.beta.pid);
			bd_reset(&slavedspic.beta.bd);
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
	_TRISC9 = 0;	// SLAVE_LED1
		
	/* encoders */	
	/* XXX ALPHA_ENC_CHA and ALPHA_ENC_CHB swaped respect schematic */
	_QEA1R 	= 20;	// QEA1 <- RP20(RC4) <- ALPHA_ENC_CHA
	_TRISC4	= 1;
	_QEB1R 	= 21;	// QEB1 <- RP21(RC5) <- ALPHA_ENC_CHB
	_TRISC5 = 1;	
	
	_QEA2R 	= 19;	// QEA2 <- RP19(RC3) <- BETA_ENC_CHA
	_TRISC3 = 1;	
	_QEB2R 	= 4;	// QEB1 <- RP4(RB4)  <- BETA_ENC_CHB
	_TRISB4	= 1;
	

	/* uarts */
#if 0
	_U1RXR 	= 8;	// U1RX <- RP8(RB8) <- SLAVE_UART_RX
	_TRISB8 = 1;	// U1RX is input
  	_RP7R 	= 3;	// U1TX -> RP7(RB7) -> SLAVE_UART_TX
	_TRISB7	= 0;	// U1TX is output
#else	
	_U1RXR 	= 5;	// U1RX <- RP5(RB5) <- SLAVE_UART_RX
	_TRISB5 = 1;	// U1RX is input  	_RP6R 	= 3;	// U1TX -> RP6(RB6) -> SLAVE_UART_TX	_TRISB6	= 0;	// U1TX is output
#endif

	_U2RXR 	= 9;	// U2RX <- RP9 <- AX12_UART	_RP9R 	= 5;	// U2TX -> RP9 -> AX12_UART	_TRISB9	= 0;	// U2TX is output 	_ODCB9 	= 1;	// For half-duplex mode RP9 is open collector

}

void angle_autopos(struct cs_block *csb, uint16_t ax12_id, void * enc_id, uint8_t reverse);

int main(void)
{

	/* remapeable pins */
	io_pins_init();
	
	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();

	memset(&gen, 0, sizeof(gen));
	memset(&slavedspic, 0, sizeof(slavedspic));
	slavedspic.flags = DO_ENCODERS | DO_POWER | DO_BD;
	
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

	/* ax12 */
	ax12_user_init();

	wait_ms(500);

	/* strat */
 	gen.logs[0] = E_USER_APP;
 	gen.log_level = 5;

	sei();

	/* ax12 servos init, set free running mode */
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x00);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x04);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x00);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x00);


	printf_P(PSTR("\r\n"));
	printf_P(PSTR("I program in solder\r\n"));

	cmdline_interact();

	return 0;
}










