/*
 * buffer.h
 *
 * This file holds public function definitions for the implementation of a circular Buffer 
 *
 * Created: 04.05.2015 14:22:59
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>


/** Struct for a Circluar Buffer */
typedef struct {
	uint16_t *bufferData1_p;		//Pointer to an Array containing the first column of Buffer-Data
	uint16_t *bufferData2_p;		//Pointer to an Array containing the second column of Buffer Data
	uint8_t head;					//Position of the head of the buffer (index in the array)
	uint8_t tail;              		//Position of the tail of the buffer (index in the array)
	uint8_t maxBuffersize;     		//Maximum possible Buffersize
	uint8_t buffersize;        		//Current size of the buffer
} CircularBuffer;



#endif /* BUFFER_H_ */