/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
#include "i2c.h"
#define CORE_FREQ   (600000000U)
#define BAUDRATE    (100000U)
// in the FOR, one loop cost 10 ticks
#define LOOP        (CORE_FREQ / 10 / BAUDRATE)
void delay(){
	volatile int i;
	for(i=0;i<LOOP;i++){
	}
}

/*
	START->ADDR->ACK->DATA->NACK
*/

#define ACK() \
	SCL_LOW(); \
	SDA_LOW(); \
	delay();  \
	SCL_HIGH(); \
	delay(); \
	SCL_LOW(); \
	delay();

#define NACK() \
	SCL_LOW(); \
	SDA_HIGH(); \
	delay(); \
	SCL_HIGH(); \
	delay(); \
	SCL_LOW(); \
	delay();
	
#define START()      \
	SDA_HIGH(); \
	SCL_HIGH(); \
	delay(); \
	SDA_LOW(); \
	delay(); \
	SCL_LOW(); \
	delay();
	
#define STOP()   \
	NACK(); \
	SDA_LOW(); \
	SCL_HIGH(); \
	delay(); \
	SDA_HIGH();	
	
typedef enum {
	WRITE = 0,
	READ
}DIR;
typedef enum {
	NACK = 0,
	ACK,
	ERR
}RESPONSE;

void init(){
	gpio_pin_config_t config = {
		.direction = kGPIO_DigitalOutput,
		.outputLogic = 1
	};
	GPIO_PinInit(PIN_PORT, SCL_PIN, &config);
	GPIO_PinInit(PIN_PORT, SDA_PIN, &config);
 // if test ov7725, must config the MCLK-pin and the csi clock
	
}

RESPONSE check_ack(){
	SCL_LOW();
	SDA_HIGH();
	delay();
	SCL_HIGH();
	delay();
	SDA_AS_INPUT();
	int value = SDA_STATE();
	SDA_AS_OUTPUT();
	SCL_LOW();
	delay();
	if(value == 0) return ACK;
	if(1 == value) return NACK;	
	// invalid state
	return ERR;
}
void bus_write(uint8_t v){
	for(int i=0;i<8;i++){
		SCL_LOW();
		if(v & 0x80){
			SDA_HIGH();
		}else{
			SDA_LOW();
		}
		delay();
		SCL_HIGH();
		delay();
		v <<= 1;
	}
}
void bus_read(uint8_t* v){
	uint8_t tmp = 0;
	for(int i=0;i<8;i++){
		tmp <<= 1;
		SCL_HIGH();
		delay();
		uint8_t state = SDA_STATE();
		tmp |= state & 0x1;
		SCL_LOW();
		delay();
	}
	*v = tmp;
}
RESPONSE i2c_transfer(uint8_t addr, void* data, uint32_t len, DIR dir){
	RESPONSE state;
	START();
	addr = (addr << 1) | (dir & 0x1);
	bus_write(addr);
	if(ACK == check_ack()){
		state =  ACK;
		switch(dir){
			case READ:
				SDA_AS_INPUT();
				if(len == 1){
					bus_read((uint8_t*)data);
				}
				if(len > 1){
					for(int i=0;i<len;i++){
						bus_read(((uint8_t*)data++));
						ACK();
					}
				}
				SDA_AS_OUTPUT();
				STOP();
				break;
			case WRITE:
				for(int i=0;i<len;i++){
					bus_write(((uint8_t*)data)[i]);
					if(ACK != check_ack())
						break;
				}
				STOP();
				break;
			default:
				break;
		}		
	}else{
		STOP();
		state =  NACK;
	}
	return state;
}

#define mem_write(slave_addr, reg_buf, len)  i2c_transfer(slave_addr, buf, len, WRITE)
RESPONSE mem_read(uint8_t slave_addr, uint8_t reg, uint8_t* buf, uint8_t len){
	RESPONSE state;
	if(NACK == i2c_transfer(SLAVE_ADDR, &reg, 1, WRITE)){
		return NACK;
	}
	state = i2c_transfer(SLAVE_ADDR, buf, len, READ); 
	return state;
}

RESPONSE is_DeviceReady(uint8_t addr){
	START();
	addr = (addr << 1);
	bus_write(addr);
	if(ACK == check_ack()){
		STOP();
		return ACK;
	}else{
		STOP();
		return NACK;
	}
}

int main(void)
{
    char ch;

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
	
	init();
	
	#if 0
		for(uint8_t i=0;i<=0x7f;i++){
			if(0 == i2c_ack_check(CVT_ADDR(i, WRITE_ADDR)))
				PRINTF("0x%02x ", i);
			if( 0 == ((i+1) & 0x0f))
				PRINTF("\r\n");
		}
		i2c_stop();
		i2c_start();
		i2c_write_single_byte(CVT_ADDR(0x21, WRITE_ADDR));
		i2c_write_single_byte(0x0A);
		i2c_stop();
		i2c_start();
		i2c_write_single_byte(CVT_ADDR(0x21, READ_ADDR)); // read
		uint8_t id = i2c_read_single_byte();
	#endif
	uint8_t buf[2] = {0x0c, 0x34};
	uint8_t recv = 0;
	for(int i=0;i<0x7f;i++){
		if(is_DeviceReady(0x1a) == ACK){
			PRINTF("0x%02x ", i);
		}
	}
	while(1){
		if(NACK == mem_write(SLAVE_ADDR, buf, sizeof(buf))){
			PRINTF("Invalid device!\r\n");
		}
		if(NACK == mem_read(SLAVE_ADDR, 0x0c, &recv, 1)){
			PRINTF("Invalid device!\r\n");
		}
		if(buf[1] == recv){
			PRINTF("SAME\r\n");
		}
		buf[1]++;
		if(NACK ==  mem_read(SLAVE_ADDR, 0x0a, &recv, 1)){
			PRINTF("Invalid device!\r\n");
		}
	}
}
