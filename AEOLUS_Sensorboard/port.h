/*
 * port.h
 *
 * Created: 31.03.2015 14:55:01
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#ifndef PORT_H_
#define PORT_H_


/* @brief Init the Ports */ 
bool port_init(void); 

/* @brief Toggle the state of the LED */ 
void port_led(bool state); 

/* @brief Blink with the LED a number of times */ 
void port_led_blink(uint8_t times);



#endif /* PORT_H_ */