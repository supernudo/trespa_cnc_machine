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
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
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
#ifdef notyet
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
	/***************************************
 	*  IO portmap and config
 	*/
	
	/* XXX: after reset all pins are inputs */
	/* XXX: after reset all ANALOG pins are analog
		and has disabled the read operation
	*/
	
	/* leds */
	_TRISC9 = 0;	// SLAVE_LED1

	/* sensors */
	AD1PCFGL = 0xFF;	// all analog pins are digital
	_TRISB11 = 1;		// SENSOR1 (CALIB_Z)
	_TRISB10 = 1;		// SENSOR2 (FC_Z_UP) 
	_TRISB2 	= 1;		// SENSOR3 (FC_Z_DOWN)
	_TRISA8 	= 1;		// SENSOR4 
	_TRISC3 	= 1;		// SENSOR5
	_TRISB4 	= 1;		// SENSOR6 (NEW AX12 UART)
	_TRISC2 	= 1;		// SENSOR7 (BROKEN PIN)

	_TRISC1  = 1;     // SLAVE_SERVO_PWM_1 (CALIB_Y)
	_TRISB12  = 1;		// SLAVE_MOT_1_INA (FC_Y_LEFT)
	_TRISB13  = 1;		// SLAVE_MOT_1_INB (FC_Y_RIGHT)


	/* brushless motor (MOTOR_Z) */
	_TRISA10 = 0; 	// SLAVE_MOT_BRUSH_REV
	_TRISA7  = 0; 	// SLAVE_MOT_BRUSH_BREAK
	_LATA7   = 0;	// initialy breaked
		
	/* encoders */
	_QEA1R 	= 21;	// QEA1 <- RP21(RC5) <- ENC_Z_CHA
	_TRISC5	= 1;
	_QEB1R 	= 20;	// QEB1 <- RP20(RC4) <- ENC_Z_CHB
	_TRISC4  = 1;	
	
	_QEA2R 	= 16;	// QEA2 <- RP16(RC0) <- ENC_Y_CHA (SLAVE_SERVO_PWM_2)
	_TRISC0  = 1;	
	_QEB2R 	= 3;	// QEB1 <- RP4(RB3)  <- ENC_Y_CHB (SLAVE_SERVO_PWM_3)
	_TRISB3	= 1;
	
	
	/* uarts */
	_U1RXR 	= 8;	// U1RX <- RP8 <- SLAVE_UART_RX
	_TRISB8  = 1;	// U1RX is input
  	_RP7R 	= 3;	// U1TX -> RP7 -> SLAVE_UART_TX
	_TRISB7	= 0;	// U1TX is output
#ifdef deprecated
	/* AX12 motors (MOTOR_Y) */
#ifdef OLD_SERVO_AX12	
	_U2RXR 	= 9;	// U2RX <- RP9 <- SERVOS_AX12_UART
  	_RP9R 	= 5;	// U2TX -> RP9 -> SERVOS_AX12_UART
	_TRISB9	= 0;	// U2TX is output
 	_ODCB9 	= 1;	// For half-duplex mode RP9 is open collector
#else	_U2RXR 	= 9;	// U2RX <- RP4 <- SERVOS_AX12_UART
  	_RP4R 	= 5;	// U2TX -> RP4 -> SERVOS_AX12_UART
	_TRISB4	= 0;	// U2TX is output
 	_ODCB4 	= 1;	// For half-duplex mode RP4 is open collector
#endif
#endif

	/* DC motors (MOTOR_Y) */
	_TRISC6 = 0;	// SLAVE_MOT_2_INA
	_TRISC7 = 0;	// SLAVE_MOT_2_INB
	_LATC6  = 0;	// initialy breaked
	_LATC7  = 0;

}

void angle_autopos(struct cs_block *csb, uint16_t ax12_id, void * enc_id, uint8_t reverse);

int main(void)
{
	/* disable interrupts */
	cli();

	/* remapeable pins */
	io_pins_init();
	
	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();

	/* clear structures */
	memset(&gen, 0, sizeof(gen));
	memset(&slavedspic, 0, sizeof(slavedspic));

	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* DAC_MC */
	dac_mc_channel_init(DAC_MC_Z, 1, CHANNEL_L,
							  DAC_MC_MODE_SIGNED | DAC_MC_MODE_SIGN_INVERTED, &LATA, 10, NULL, 0);
	dac_mc_set(&gen.dac_mc_left, 0);

	


	/* ENCODERS */
	encoders_dspic_init();

	/* PWM_MC */
	pwm_mc_channel_init(&gen.pwm_mc_mod2_ch1,
	                    PWM_MC_MODE_BIPOLAR, 
	                    2, 1, NULL, 0, NULL, 0);
	pwm_mc_init(&gen.pwm_mc_mod2_ch1, 19000, 
							CH1_COMP&PEN1H&PEN1L);
	pwm_mc_set(&gen.pwm_mc_mod2_ch1, 0);

	/* DO FLAGS */
	/* note: cs is enabled after calibration */
	slavedspic.flags = DO_ENCODERS | DO_POWER; // TODO | DO_BD;

	/* TIMER */
	timer_init();

	/* SCHEDULER */
	scheduler_init();

	/* TIME event */
	time_init(TIME_PRIO);

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						100000L / SCHEDULER_UNIT, LED_PRIO);


	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						10000L / SCHEDULER_UNIT, SENSOR_PRIO);

	scheduler_add_periodical_event_priority(do_cs, NULL, 
						50000L / SCHEDULER_UNIT, CS_PRIO);


	/* all cs management */
	dspic_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

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










