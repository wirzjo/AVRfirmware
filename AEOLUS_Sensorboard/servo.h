/*
 * servo.h
 *
 * Created: 31.03.2015 14:52:59
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#include <stdbool.h>


/* @brief Init the use of a Servo */ 
bool servo_init(void);


/* @brief Set the servo to a given angle in Degrees */
void servo_set(float deg); 



#endif /* SERVO_H_ */