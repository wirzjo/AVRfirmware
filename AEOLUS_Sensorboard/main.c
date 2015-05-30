/*
 * main.c
 *
 * This software handles a LIDAR Sensor and uses SPI to communicate with the PIXHAWK Autopilote in ETH's 
 * autonomous sailing project. 
 *
 * Created: 31.03.2015 14:43:33
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


/* TODO: 
 * 
 *	- Add Watchdog
 *  - Add Sleep-Mode
 *  - Add proper scheduling 
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "config.h"
#include "port.h"
#include "servo.h"
#include "lidar.h"
#include "serial.h"
#include "pixhawk.h"
#include "measure.h"

#include <util/delay.h>

int main(void) {
	
	/************************************************************************/
	/* BOOT                                                                 */
	/************************************************************************/
	
	bool boot_state = true;		//true, if everything is fine during the boot process 
	
	//Disable any Interrupts 
	cli(); 
	
	
	//Init the input/output ports 
	boot_state = boot_state && port_init(); 
	
	//Init the use of a Servo
	boot_state = boot_state && servo_init(); 
	
	//Init the use of the LIDAR 
	//boot_state = boot_state && lidar_init();     //DEBUG: Remove true, this is only, because no lidar is present by now 
	
	//Init the use of the Pixhawk 
	//boot_state = boot_state && pixhawk_init();					//DEBUG: add this init ot the bool boot_state
	
	//Init the measurement 
	boot_state = boot_state && measure_init(); 
	
	//Allow for Interrupts (e.g. for serial communication) 
	//sei(); 
	
	
	
	//Write a message to the serial interface, that the boot-process was successful
	char str[] = {"OK"}; 
	serial_send_string(str); 
	
	
	/************************************************************************/
	/* MAIN WHILE LOOP                                                      */
	/************************************************************************/
    while(1) {
		
		if(boot_state) {
			//The handlers and other regular tasks are only executed if the boot-process was successful
			//this is for safety reasons! 
		
		
			//***SEND DATA TO PIXHAWK 
			//The Pixhawk requests for data by sending commands. These commands are processed in the 
			//Interrupt routine of the UART. 
			//The answer to a request is sent when we have time in the pixhawk_handler(). 
			//pixhawk_handler(); 
		
		
			//***MEASUREMENTS WITH THE LIDAR  
			// 
			#if DEBUG_MATLAB == 0
			measure_handler(); 
			#endif
			
			//_delay_ms(500); 
			
			//_delay_ms(100);
			
			/*servo_set(90);  //525 was ok
			_delay_ms(1000);
			_delay_ms(1000);
			_delay_ms(1000); 
			servo_set(180); 
			_delay_ms(1000);
			_delay_ms(1000);
			_delay_ms(1000);*/
			
			
			/*uint8_t i; 
			for(i=0;i<180;i=i+5) {
				servo_set(i); 
				_delay_ms(200); 
			}
			
						_delay_ms(1000);
						_delay_ms(1000);
						_delay_ms(1000);
									_delay_ms(1000);
									_delay_ms(1000);
									_delay_ms(1000);
		
		
			//port_led_blink(2); 
			//_delay_ms(1000);  */
		
		
		
			//DISPLAY THE LIDAR DISTANCE IN SERIAL INTERFACE
			//uint16_t dist = lidar_get_distance();
		
			//char buffer[10]; 
			//sprintf(buffer,"Dist: %d",dist);
			//serial_send_string(buffer);
		
		} else {
			//Something went wrong during the boot-process => signal this state with a LED being constantly on
			
			port_led(true); 
		} //if boot_state 
		
	} //while(1)
	
} //main(void)