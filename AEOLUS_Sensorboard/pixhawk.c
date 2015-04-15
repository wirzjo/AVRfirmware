/*
 * pixhawk.c
 *
 * This file implements the communication with the Pixhawk. It specifies a protocol, 
 * such that this hardware could be easily replaced by an other sensor (for example a camera). 
 *
 * Further, it acts as the interface between the Pixhawk and the LIDAR Sensor 
 *
 * The protocol is as follows: 
 * 1) Read request from Pixhawk: 
 *    0x02 | 0x02 | Command-Byte | heading0 | heading1 | 0x03
 *    Note: heading0 and heading1 are the high and the low byte of the heading wrt. true north of the boat 
 * 2) Answer to a read request 
 *    0x02 | 0x02 | Command-Byte | number of bytes |...variable length of data bytes dependent on the command... | 0x03
 *
 *
 *
 * Created: 02.04.2015 11:56:17
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 

/* 
 * TODO: 
 *   - Do not check, if we received Start or End-char when the heading bytes are sent! 
 */


#include <stdbool.h>

#include "config.h"
#include "pixhawk.h"
#include "serial.h"


/************************************************************************/
/* V A R I A B L E S                                                    */
/************************************************************************/

typedef enum{IDLE,STARTCHAR, COMMAND, HEAD0, HEAD1, ENDCHAR, ERROR} state_enum;	
static state_enum rx_state = IDLE;  //State for the receive-finite state machine

static uint8_t cmd = 0x00;			//Last Command transmitted by the message
static uint8_t head0 = 0x00;		//High byte of the heading 
static uint8_t head1 = 0x00;		//Low byte of the heading 

#define MAXNROFOBSTACLES 10			//Maximum number of obstacles that can be detected and reported to Pixhawk 


static struct {
	uint8_t numofobstacles;			//Current number of obstacles known 
	obstacle obstacles[MAXNROFOBSTACLES]; //Obstacles represented as Obstacle-Objects 
	uint16_t heading;				//Current heading of the boat known from Pixhawk 
} state = {
	.numofobstacles = 0, 
	.heading = 0
};

static bool flag_send = false; 




/************************************************************************/
/* P R O T O C O L                                                      */
/************************************************************************/

#define MSG_START		0x02	//Start character for a message
#define MSG_END			0x03	//End character for a message

#define CMD_OBSTACLES	0x4F	//Send the bearings and distances to every obstacle in range
								//Note: bearing (high/low byte) and then the distance is sent
#define CMD_NUMOBSTACLES 0x4E   //Number of obstacles currently in range 
#define CMD_LASTDIST    0x4A    //Latest known distance from the LIDAR 	




/************************************************************************/
/* F U N C T I O N    P R O T O T Y P E S                               */
/************************************************************************/

/* @brief Send data to pixhawk */ 
bool send2pixhawk(uint8_t cmd); 



/************************************************************************/
/* P U B L I C    F U N C T I O N S                                     */
/************************************************************************/

/**
 * Init the communication with the Pixhawk 
 *
 * @return true, if initialization was successful 
 */
bool pixhawk_init(void) {
	
	//Set the state of the parser
	rx_state = IDLE; 
	
	
	//Init the serial communication 
	//serial_init(38400);	//for use with PIXHAWK
	serial_init(19200); 
	
	return true; 
}


/**
 * Parsing new data that is available from the serial interface 
 * Note: This function is called by the "Receive completed interrupt"
 *
 * @param Pointer to a circular buffer 
 */
bool pixhawk_parse(uint8_t data) {
	
	//Text starts with STX = 0x02 and ends with ETX = 0x03
	//A message from the Pixhawk must have the following form: 
	// 0x02 | 0x02 | 0xXX (Command byte) | 0x03 
	
	//Turn on LED to signal Data transfer 
	port_led(true); 
	
	switch(rx_state) {
		case IDLE: {
			//The state machine is idle and waits for chars to be sent 
			
			if(data == MSG_START) {
				//We received the first Start-Character
				
				rx_state = STARTCHAR; 			 
			}
			
			break; 
		}
		case STARTCHAR: {
			//The first Start-Character was sent and we are waiting for the second one now 
			
			if(data == MSG_START) {
				//We received the second Start-Character
				
				rx_state = COMMAND;
				 
			} else {
				//No second Start-Character was sent => return to IDLE
				
				rx_state = IDLE; 
			}	
			
			break;		
		}
		case COMMAND: {
			//The second Start-Character was sent, now we expect to receive the Command 
			
			if(data == MSG_START || data == MSG_END) {
				//We received again a Start or End Character or a 0 => ERROR 
				//return to IDLE
				
				rx_state = IDLE; 
			} else {
				//The char is valid => store it 
				
				cmd = data; 
				
				rx_state = HEAD0; 
			}
			
			break; 
		}
		case HEAD0: {
			//The command was sent => expect to receive the "heading0" char 
			//NOTE: We do not check, if we received a start or an End-Char, because it could happen that the heading contains one of these characters 

			//The char is valid => store it
			head0 = data; 
			rx_state = HEAD1;
			
			break; 
		}
		case HEAD1: {
			//The first heading byte was receives => expect to receive the second one 
			//NOTE: We do not check, if we received a start or an End-Char, because it could happen that the heading contains one of these characters 
			
			//The char is valid => store it
			head1 = data;
			rx_state = ENDCHAR;
						
			break; 
		}
		case ENDCHAR: {
			//The command byte was read and we wait for the end-byte 
			
			if(data == MSG_END) {
				//We received the End Character => Data is valid 
				
				flag_send = true; 
				
				//Store the heading transmitted with the request
				state.heading = (uint16_t)(head0<<8) || (uint16_t)(head1);
				
				rx_state = IDLE; 
			
			} else {
				//Some error occurred => return to IDLE
		 				
				rx_state = IDLE;
			}
			
			break; 
		}
		default: {
			//This code should never be reached
			//Nothing we can do about, if we reach it...just go back to IDLE
			
			rx_state = IDLE; 
			
			break; 
		}
	}
	
	//Unlit LED
	port_led(false); 
	
	
	return true; 
}



/**
 * Get the last known Heading of the boat
 *
 */
uint16_t pixhawk_get_heading(void) {
	return state.heading; 
}


/**
 * Handle repetitive tasks like sending data 
 * Note: This function should be called in every program loop 
 * 
 */
void pixhawk_handler(void) {

	if(flag_send) {
		//Data needs to be sent 
		//port_led_blink(2);
		
		//serial_send_byte(cmd); 
		send2pixhawk(cmd); 
		
		cmd = 0x00;
		flag_send = false;  
	}
}




/************************************************************************/
/* P R I V A T E    F U N C T I O N S                                   */
/************************************************************************/

/**
 * Send data to Pixhawk 
 *
 * @param 
 */
bool send2pixhawk(uint8_t cmd) {
	
	//Send start-sequence 
	serial_send_byte(MSG_START);
	serial_send_byte(MSG_START);
	
	//Send Command number 
	serial_send_byte(cmd); 
	
	//Send individual data 
	switch(cmd) {
		case CMD_OBSTACLES: {
			
			//Send number of bytes that will be transmitted 
			serial_send_byte(state.numofobstacles*2);	
			
			//Send the data for the obstacles 
			uint8_t i; 
			for(i = 0; i<state.numofobstacles; i++) {
				//serial_send_byte((uint8_t)(state.obstacles[i].bearing>>8));  //High byte of bearing
				//serial_send_byte((uint8_t)(state.obstacles[i]&0x00FF));		 //Low byte of bearing
				//serial_send_byte((uint8_t)(state.obstacles[i].distance>>8)); //High byte of distance 
				//serial_send_byte((uint8_t)(state.obstacles[i]&0x00FF));		 //Low byte of distance 
			}
			
			break; 
		}
		case CMD_NUMOBSTACLES: {
			
			//Send the number of bytes that will be transmitted
			serial_send_byte(0x01); 
			
			//Send the number of Obstacles 
			serial_send_byte(state.numofobstacles);
			
			break; 
		}
		case CMD_LASTDIST: {
			//Return the last measured distance by the LIDAR in two bytes (high-byte first) 
			
			//Send the number of bytes that will be transmitted 
			serial_send_byte(0x02); 
			
			//Get the distance from the LIDAR
			uint16_t dist = lidar_get_distance();
			
			//Send the distance as high and low byte to the Pixhawk 
			serial_send_byte((uint8_t)(dist>>8)); 
			serial_send_byte((uint8_t)(dist)); 
			
			break; 
		}
		default: {
			//An invalid command was sent => might flag unhappy...
			
			//port_led_blink(2); 
			
			return false; 
		}
	}
	
	//Send end of Message 
	serial_send_byte(MSG_END);
	
}

