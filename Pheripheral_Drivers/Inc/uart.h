/*
 * uart.h
 *
 *  Created on: Feb 17, 2022
 *      Author: skylivingston
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#define UART_GYRO_BIAS 0x47 //G
#define UART_AUTOCORRECT_TOGGLE 0x43 //C
#define UART_SPEED 0x56  //V

#define UART_FORWARD 0x57 //W
#define UART_REVERSE 0x53 //S
#define UART_FORWARD_RIGHT 0x45 //E
#define UART_FORWARD_LEFT 0x51 //Q
#define UART_REVERSE_RIGHT 0x44 //D
#define UART_REVERSE_LEFT 0x41 //A
#define UART_BREAK 0x42  //B

#define UART_PROPORTION 0x50 //P
#define UART_INTEGRAL 0x49 //I
#define UART_POLL 0x55 //U
#define UART_START_TASK2 0x4C //L







#include "main.h"
#include "stdlib.h"

typedef struct{
	UART_HandleTypeDef *uart;
	uint8_t *TxBuffer;
	uint8_t *RxBuffer;
}uart_control;



uart_control* uart_init(UART_HandleTypeDef *uart);
void set_TxBuffer(uart_control* uart_ctl,uint8_t* TxBuffer);
uint8_t*  get_RxBuffer(uart_control* uart_ctl);
void uart_transmit(uart_control* uart_ctl);
uint8_t* uart_recieve(uart_control* uart_ctl, int buffersize);


#endif /* INC_UART_H_ */
