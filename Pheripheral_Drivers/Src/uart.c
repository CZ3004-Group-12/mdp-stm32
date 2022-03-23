/*
 * uart.c
 *
 *  Created on: Feb 17, 2022
 *      Author: skylivingston
 */


#include "uart.h"



uart_control* uart_init(UART_HandleTypeDef *uart){
	uart_control* uart_ctl = malloc(sizeof(uart_control));
	uart_ctl->uart = uart;
	return uart_ctl;
}

void set_TxBuffer(uart_control* uart_ctl,uint8_t* TxBuffer){
	uart_ctl->TxBuffer = TxBuffer;

}

uint8_t*  get_RxBuffer(uart_control* uart_ctl){
	return uart_ctl->RxBuffer;

}

void uart_transmit(uart_control* uart_ctl){
	HAL_UART_Transmit(uart_ctl->uart, uart_ctl->TxBuffer, sizeof(uart_ctl->TxBuffer),1000);

}

uint8_t* uart_recieve(uart_control* uart_ctl, int buffersize){
	HAL_UART_Receive(uart_ctl->uart, uart_ctl->RxBuffer, buffersize,1000);
	return get_RxBuffer(uart_ctl);
}



