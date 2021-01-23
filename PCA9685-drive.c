#include <reg52.h>          
#include <intrins.h>  
#include <stdio.h>
#include <math.h>

typedef  unsigned char  uchar;       
typedef  unsigned int   uint;       

sbit scl=P3^6;                   // I/O ports for IIC bus
sbit sda=P3^7;

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

void delayms(uint z)
{
  uint x,y;
  for(x=z;x>0;x--)
      for(y=148;y>0;y--);
}

void delayus()
{
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
}

void init()
{
	sda=1;
	delayus();
	scl=1;
	delayus();
}

void start()
{
	sda=1;
	delayus();
	scl=1;
	delayus();
	sda=0;
	delayus();
	scl=0;
	delayus();
}

void stop()
{
	sda=0;
	delayus();
	scl=1;
	delayus();
	sda=1;                  
	delayus();
}

void ACK()
{
	uchar i;
	scl=1;
	delayus();
	while((sda==1)&&(i<255))         
		i++;                                       
	scl=0;                                  
	delayus();
}

void write_byte(uchar byte)
{
	uchar i,temp;
	temp=byte;
	for(i=0;i<8;i++)
	{
		temp=temp<<1;  
		scl=0;                  
		delayus();
		sda=CY;                 
		delayus();
		scl=1;          
		delayus();
	}
	scl=0;                  
	delayus();
	sda=1;                 
	delayus();
}

uchar read_byte()
{
	uchar i,j,k;
	scl=0;
	delayus();
	sda=1;
	delayus();
	for(i=0;i<8;i++)       
	{
		delayus();
		scl=1;
		delayus();
		if(sda==1)
		{
			j=1;
		}
		else j=0;
		k=(k<< 1)|j;  
		scl=0;           
	}
	delayus();
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

void reset(void)
{
	PCA9685_write(PCA9685_MODE1,0x0);
}

void begin(void)
{
	reset();
}

void setPWMFreq(float freq)
{
	uint prescale,oldmode,newmode;
	float prescaleval;
	freq *= 0.915;
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
	delayms(2);
	PCA9685_write(PCA9685_MODE1, oldmode | 0xa1);
}

void setPWM(uint num, uint on, uint off)
{
	PCA9685_write(LED0_ON_L+4*num,on);
	PCA9685_write(LED0_ON_H+4*num,on>>8);
	PCA9685_write(LED0_OFF_L+4*num,off);
	PCA9685_write(LED0_OFF_H+4*num,off>>8);
}

void main()
{
	begin();
	setPWMFreq(50);  
	while(1)
	{
		int i;
		for(i=0;i<=4096;i+=100)                   //PWM rate for servo control remain undetermined.
		{
			setPWM(0, 0, i);
			delayms(1500);
		}
	}               
}