
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include "delay.h"
#include <string.h>
#include <time.h>
#include <math.h>

#define scl PEout(0)
#define wsda PEout(1)
#define rsda PEin(1)
#define uchar unsigned char
#define uint unsigned int
#define SDA_IN()  {GPIOE->CRL&=0X0FFFFFFF;GPIOE->CRL|=(u32)8<<28;}
#define SDA_OUT() {GPIOE->CRL&=0X0FFFFFFF;GPIOE->CRL|=(u32)3<<28;}

void astart()
{
	SDA_OUT();
	wsda=1;
	delay_us(4);
	scl=1;
	delay_us(4);
	wsda=0;
	delay_us(4);
	scl=0;
	delay_us(4);
}

void astop()
{
	SDA_OUT();
	wsda=0;
	delay_us(4);
	scl=1;
	delay_us(4);
	wsda=1;                  
	delay_us(4);
}

void aACK()
{
	uchar i;
	SDA_IN();
	scl=1;
	delay_us(4);
	while((rsda==1)&&(i<255))         
		i++;                                       
	scl=0;                                  
	delay_us(4);
}

void awrite_byte(uchar byte)
{
	uchar i;
	SDA_OUT();
	for(i=8;i;i--)
	{
		scl=0;                  
		delay_us(4);
		wsda=(byte&(1<<(i-1)))?1:0;                 
		delay_us(4);
		scl=1;          
		delay_us(4);
	}
	scl=0;                  
	delay_us(4);
	wsda=1;                 
	delay_us(4);
}

uchar aread_byte()
{
	uchar i,j,k;
	SDA_IN();
	scl=0;
	delay_us(4);
	wsda=1;
	delay_us(4);
	for(i=0;i<8;i++)       
	{
		delay_us(4);
		scl=1;
		delay_us(4);
		if(rsda==1)
		{
			j=1;
		}
		else j=0;
		k=(k<< 1)|j;  
		scl=0;           
	}
	delay_us(4);
	return k;
}

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	return 0; // to be implemented
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	return 0; // to be implemented
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	astart();
	awrite_byte(dev);
	aACK();                          
	awrite_byte(index);
	aACK();
	awrite_byte(data);
	aACK();
	astop();
	return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	VL53L1_WrByte(dev,index,data>>8);
	VL53L1_WrByte(dev,index,data&0xff);
	return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	VL53L1_WrByte(dev,index,data>>24);
	VL53L1_WrByte(dev,index,(data>>16)&0xff);
	VL53L1_WrByte(dev,index,(data>>8)&0xff);
	VL53L1_WrByte(dev,index,data&0xff);
	return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	astart();
	awrite_byte(dev);
	aACK();
	awrite_byte(index);
	aACK();
	astart();
	awrite_byte(dev|0x01);
	aACK();
	*data=aread_byte();
	astop();
	return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	uchar tmp;
	VL53L1_RdByte(dev,index,&tmp);
	*data=(tmp<<8);
	VL53L1_RdByte(dev,index,&tmp);
	*data|=tmp;
	return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	uchar tmp;
	VL53L1_RdByte(dev,index,&tmp);
	*data=(tmp<<24);
	VL53L1_RdByte(dev,index,&tmp);
	*data|=(tmp<<16);
	VL53L1_RdByte(dev,index,&tmp);
	*data|=(tmp<<8);
	VL53L1_RdByte(dev,index,&tmp);
	*data|=tmp;
	return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	return 0; // to be implemented
}
void VL53L1X_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	 //使能PA端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;				 //PA 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA
	wsda=1;
	delay_us(1);
	scl=1;
	delay_us(1);
}
