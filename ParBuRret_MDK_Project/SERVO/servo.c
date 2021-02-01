#include "servo.h"
#include "delay.h"
#include "math.h"

typedef  unsigned char  uchar;       
typedef  unsigned int   uint;       

#define PCA9685_adrr 0x80
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD
#define scl PAout(6)
#define wsda PAout(5)
#define rsda PAin(5)

void start()
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

void stop()
{
	SDA_OUT();
	wsda=0;
	delay_us(4);
	scl=1;
	delay_us(4);
	wsda=1;                  
	delay_us(4);
}

void ACK()
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

void write_byte(uchar byte)
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

uchar read_byte()
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

void PCA9685_write(uchar address,uchar date)
{
	start();
	write_byte(PCA9685_adrr);
	ACK();                          
	write_byte(address);
	ACK();
	write_byte(date);
	ACK();
	stop();
}

uchar PCA9685_read(uchar address)
{
	uchar date;
	start();
	write_byte(PCA9685_adrr);
	ACK();
	write_byte(address);
	ACK();
	start();
	write_byte(PCA9685_adrr|0x01);
	ACK();
	date=read_byte();
	stop();
	return date;
}

void setPWMFreq(float freq)
{
	uint prescale,oldmode,newmode;
	float prescaleval;
	freq *= 0.92;
	prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	prescale = floor(prescaleval + 0.5);
 
	oldmode = PCA9685_read(PCA9685_MODE1);
	newmode = (oldmode&0x7F) | 0x10;
	PCA9685_write(PCA9685_MODE1, newmode);
	PCA9685_write(PCA9685_PRESCALE, prescale);
	PCA9685_write(PCA9685_MODE1, oldmode);
	delay_ms(2);
	PCA9685_write(PCA9685_MODE1, oldmode | 0xa1);
}

void setPWM(uint num, uint on, uint off)
{
	PCA9685_write(LED0_ON_L+4*num,on);
	PCA9685_write(LED0_ON_H+4*num,on>>8);
	PCA9685_write(LED0_OFF_L+4*num,off);
	PCA9685_write(LED0_OFF_H+4*num,off>>8);
}

void Servo_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;				 //PA 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA
	wsda=1;
	delay_us(1);
	scl=1;
	delay_us(1);
	
	PCA9685_write(PCA9685_MODE1,0x0);
	setPWMFreq(50); 
}

void RotateClockwiseX(void)                                             //Run Servo_Init() at the beginning.
{
	setPWM(0,0,74);
}

void RotateStopX(void)
{
	setPWM(0,0,274);
}

void RotateCounterClockwiseX(void)
{
	setPWM(0,0,478);
}
void RotateClockwiseY(void)
{
	setPWM(1,0,74);
}

void RotateStopY(void)
{
	setPWM(1,0,274);
}

void RotateCounterClockwiseY(void)
{
	setPWM(1,0,478);
}
