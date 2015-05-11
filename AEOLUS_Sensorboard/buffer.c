/*
 * buffer.c
 *
 * This file implements a dynamic circular buffer 
 *
 * Created: 04.05.2015 14:22:47
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 

#include "buffer.h"



/** @brief Update the Size of an existing Circular Buffer */
bool buffer_updateSize(CircularBuffer *buffer, uint8_t buffersize);

/** @brief Delete the oldest value from the Buffer */
bool buffer_delete_oldest(CircularBuffer *buffer);






/**
 * Init a new Buffer of a given Size
 *
 * @return a circular Buffer
 */
CircularBuffer buffer_init(uint8_t buffersize) {

	CircularBuffer ret; 	//Create new Circular Buffer

	buffer_updateSize(&ret, buffersize);		//Set the correct length of the buffer

	return ret;
}



/**
 * Update the Size of a given Buffer
 *
 * @return true, if the size of the buffer was successfully changed
 */
bool buffer_updateSize(CircularBuffer *buffer, uint8_t buffersize) {

	//if(buffer == NULL) {
	//  //For some reason the buffer does not exist => create it.
	//	CircularBuffer newBuffer = buffer_init(Config.windowSize);
	//	buffer = &newBuffer;
	//}

	if(buffersize != buffer->maxBuffersize) {
		//The Size of the buffer has changed => initialize the new buffer

		//Delete the old buffer
		free(buffer->bufferData1_p);
		free(buffer->bufferData2_p);

		//Allocate memory for the new buffer
		buffer->bufferData1_p = malloc(sizeof(float) * buffersize);	//Allocate memory for the new buffer
		buffer->bufferData2_p = malloc(sizeof(float) * buffersize);

		//Fill the new buffer with zeros
		//memset(buffer->bufferData_p, 0, buffer->buffersize);
		for(uint8_t i = 0; i < buffersize; i++) {
		        buffer->bufferData1_p[i] = 0;
				buffer->bufferData2_p[i] = 0;
		}

		//Set the new maximum Buffersize
		buffer->maxBuffersize = buffersize + 1;
		buffer->buffersize = 0;
		buffer->head = 0;
		buffer->tail = 0;

		return true;
	}

	return false;
}


/**
 * Add a new Value to the buffer
 *
 * @param *buffer: Pointer to a Circluar Buffer
 * @param value1: Value1 to be added to the buffer's first column
 * @param value2: Value2 to be added to the buffer's second column
 * @return true, if the value is added successfully
 */
bool buffer_add(CircularBuffer *buffer, uint16_t value1, uint16_t value2) {

	uint8_t next = (unsigned int)(buffer->head + 1) % buffer->maxBuffersize;

	if (next != buffer->tail) {
		//The buffer is not full => add value to the buffer

		buffer->bufferData1_p[buffer->head] = value1;
		buffer->bufferData2_p[buffer->head] = value2;
		buffer->head = next;
		buffer->buffersize += 1;
	} else {
		//The buffer is full => delete oldest element and add the new value
		buffer_delete_oldest(buffer);
		buffer_add(buffer,value1,value2);
	}

	return true;
}



/**
 * Delete the oldest value from the RingBuffer
 *
 * @param *buffer: A pointer to a circular Buffer
 * @return true, if the value was successfully deleted
 */
bool buffer_delete_oldest(CircularBuffer *buffer) {

	if(buffer->head == buffer->tail) {
		//The buffer is empty
		return false;
	} else {
		//There is at least one element in the buffer
		buffer->tail = (uint8_t) (buffer->tail + 1) % buffer->maxBuffersize;
		buffer->buffersize -= 1;

		return true;
	}

} //End of buffer_deleteOldest



/**
 * Get the values that are at the head of the buffer, then "delete" this element 
 *
 * Note: This function does not check if the element exists! It always returns a value,
 * even it lays outside of the buffer (due to the circularity).
 *
 * @param  *buffer: Pointer to a Circluar Buffer
 * @param  pos: Position in the buffer
 * @param  value1: Value at pos from the first column
 * @param  value2: Value at pos from the second column 
 * @return the value at the given position in the buffer
 */
bool buffer_get_values(CircularBuffer *buffer, uint16_t *value1, uint16_t *value2) {

	//uint8_t ind = (uint8_t)(buffer->tail + pos) % buffer->maxBuffersize;
	
	//Return the 
	uint8_t ind = buffer->tail; 

	*value1 = buffer->bufferData1_p[ind];
	*value2 = buffer->bufferData2_p[ind]; 
	
	return buffer_delete_oldest(buffer); 

} //End of buffer_getValue


/**
 * Get the state of the buffer. 
 *
 * @param buffer: Pointer to a circular buffer
 * @return true, iff the buffer is empty, false else 
 */
bool buffer_is_empty(CircularBuffer *buffer) {
	
	if(buffer->head == buffer->tail) {
		return true; 
	} else {
		return false; 
	}
	
}


/** 
 * Get the size of the buffer 
 *
 * @param buffer: Pointer to the circular buffer
 * @return size of the buffer 
 */
uint8_t buffer_get_size(CircularBuffer *buffer) {
	return buffer->buffersize; 
}
