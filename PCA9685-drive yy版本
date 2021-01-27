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

void delayus()
{
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
}

void delayms(uchar x)
{
	uchar i,j;
	for(i=x;i>0;i--)
		for(j=0;j<110;j++);
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
	delayms(2);
	PCA9685_write(PCA9685_MODE1, oldmode | 0xa1);
}

void ServoInit(void)
{
	PCA9685_write(PCA9685_MODE1,0x0);
	setPWMFreq(50); 
}

void setPWM(uint num, uint on, uint off)
{
	PCA9685_write(LED0_ON_L+4*num,on);
	PCA9685_write(LED0_ON_H+4*num,on>>8);
	PCA9685_write(LED0_OFF_L+4*num,off);
	PCA9685_write(LED0_OFF_H+4*num,off>>8);
}

void RotateClockwiseX(void)                                             //Run ServoInit() at the beginning.
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

int main()
{
	int state=1,Status;
	while(state)//检验传感器是否启动
	{
		Status=VL53L1X_BootState(dev,&state);
		HAL_delay(2);
	}
	
	Status=VL53L1X_SensorInit();//初始化传感器
	
	VL53L1X_SetDistanceMode(dev,2);//设置测量模式（长程）

	Status=VL53L1X_SetInterMeasurementPeriod();
	Status=VL53L1X_SetOffset();

	while(1)
	{
		while(dataReady==0)Status=VL53L1X_CheckForDataReady(dev,&dataReady);
		dataReady=0;
		Status=VL53L1X_GetRangeStatus();
		Status=VL53L1X_Getdistance();
		Status=VL53L1X_ClearInterrupt();
	}
}
