/*
 * pixhawk.c
 *
 * This file implements the communication with the Pixhawk. It specifies a protocol, 
 * such that this hardware could be easily replaced by an other sensor (for example a camera). 
 *
 * Created: 02.04.2015 11:56:17
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#include <stdbool.h>

#include "config.h"
#include "pixhawk.h"


/************************************************************************/
/* V A R I A B L E S                                                    */
/************************************************************************/

typedef enum{IDLE,STARTCHAR, COMMAND, ENDCHAR, COMPLETE, ERROR} state_enum;
static state_enum rx_state = IDLE; 
	
#define MSG_START 0x02	//Start character for a message 
#define MSG_END   0x03  //End character for a message 

static uint8_t cmd = MSG_START;		//Last Command transmitted by the message




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
	
	
	return true; 
}


/**
 * Parsing new data that is available from the serial interface 
 *
 * @param Pointer to a circular buffer 
 */
bool pixhawk_parse(uint8_t data) {
	
	//Text starts with STX = 0x02 and ends with ETX = 0x03
	//A message from the Pixhawk must have the following form: 
	// 0x02 | 0x02 | 0xXX (Command byte) | 0x03
	
	switch(rx_state) {
		case IDLE: {
			if(data == MSG_START) {
				//We received the first Start-Character
				
				rx_state = STARTCHAR; 			 
			}
		}
		case STARTCHAR: {
			if(data == MSG_START) {
				//We received the second Start-Character
				
				rx_state = COMMAND;
				 
			} else {
				//No second Start-Character was sent => return to IDLE
				
				rx_state = IDLE; 
			}			
			
		}
		case COMMAND: {
			if(data == MSG_START || data == MSG_END) {
				//We received again a Start or End Character or a 0 => ERROR 
				//return to IDLE
				
				rx_state = IDLE; 
			} else {
				//The char is valid => store it 
				
				cmd = data; 
				
				rx_state = ENDCHAR; 
			}
		}
		case ENDCHAR: {
			if(data == MSG_END) {
				//We received the End Character => Data is valid 
				
				rx_state = COMPLETE;
			} else {
				//Some error occurred => return to IDLE
		 				
				rx_state = IDLE;
			}
		}
		case COMPLETE: {
			//A complete message was received => send requested data to pixhawk 
			send2pixhawk(cmd); 
			
			//Return to state IDLE in order to wait for the next command 
			rx_state = IDLE; 
		}
		default: {
			//This code should never be reached
			//Nothing we can do about, if we reach it...just go back to IDLE
			
			rx_state = IDLE; 
		}
	}
	
	return true; 
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
	
	switch(cmd) {
		case 0x00: {
			//TODO: 0x00 is invalid, replace by true character 
			
		}
		default: {
			//An invalid command was sent => might flag unhappy...
			
			return false; 
		}
	}
	
}

