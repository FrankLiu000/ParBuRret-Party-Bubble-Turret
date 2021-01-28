#include <reg52.h>          
#include <intrins.h>  
#include <math.h>
#include "VL53L1X_api.h"
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include "vl53l1_types.h"
#include "vl53l1_platform.h"

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
#define uchar unsigned char
#define uint unsigned int

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

#if 0
uint8_t VL51L1X_NVM_CONFIGURATION[] = {
0x00, /* 0x00 : not user-modifiable */
0x29, /* 0x01 : 7 bits I2C address (default=0x29), use SetI2CAddress(). Warning: after changing the register value to a new I2C address, the device will only answer to the new address */
0x00, /* 0x02 : not user-modifiable */
0x00, /* 0x03 : not user-modifiable */
0x00, /* 0x04 : not user-modifiable */
0x00, /* 0x05 : not user-modifiable */
0x00, /* 0x06 : not user-modifiable */
0x00, /* 0x07 : not user-modifiable */
0x00, /* 0x08 : not user-modifiable */
0x50, /* 0x09 : not user-modifiable */
0x00, /* 0x0A : not user-modifiable */
0x00, /* 0x0B : not user-modifiable */
0x00, /* 0x0C : not user-modifiable */
0x00, /* 0x0D : not user-modifiable */
0x0a, /* 0x0E : not user-modifiable */
0x00, /* 0x0F : not user-modifiable */
0x00, /* 0x10 : not user-modifiable */
0x00, /* 0x11 : not user-modifiable */
0x00, /* 0x12 : not user-modifiable */
0x00, /* 0x13 : not user-modifiable */
0x00, /* 0x14 : not user-modifiable */
0x00, /* 0x15 : not user-modifiable */
0x00, /* 0x16 : Xtalk calibration value MSB (7.9 format in kcps), use SetXtalk() */
0x00, /* 0x17 : Xtalk calibration value LSB */
0x00, /* 0x18 : not user-modifiable */
0x00, /* 0x19 : not user-modifiable */
0x00, /* 0x1a : not user-modifiable */
0x00, /* 0x1b : not user-modifiable */
0x00, /* 0x1e : Part to Part offset x4 MSB (in mm), use SetOffset() */
0x50, /* 0x1f : Part to Part offset x4 LSB */
0x00, /* 0x20 : not user-modifiable */
0x00, /* 0x21 : not user-modifiable */
0x00, /* 0x22 : not user-modifiable */
0x00, /* 0x23 : not user-modifiable */
}
#endif

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xcc, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : Intermeasurement period */
0x0f, /* 0x6e : Intermeasurement period */
0x89, /* 0x6f : Intermeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
	255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
	255, 255, 11, 12
};

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdataa, uint32_t count) {
	return 0; // to be implemented
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdataa, uint32_t count){
	return 0; // to be implemented
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t dataa) {
	return 0; // to be implemented
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t dataa) {
	return 0; // to be implemented
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t dataa) {
	return 0; // to be implemented
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *dataa) {
	return 0; // to be implemented
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *dataa) {
	return 0; // to be implemented
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *dataa) {
	return 0; // to be implemented
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	return 0; // to be implemented
}

VL53L1X_ERROR VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion)
{
	VL53L1X_ERROR Status = 0;

	pVersion->major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
	pVersion->minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
	pVersion->build = VL53L1X_IMPLEMENTATION_VER_SUB;
	pVersion->revision = VL53L1X_IMPLEMENTATION_VER_REVISION;
	return Status;
}

VL53L1X_ERROR VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address)
{
	VL53L1X_ERROR status = 0;

	status = VL53L1_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
	return status;
}

VL53L1X_ERROR VL53L1X_SensorInit(uint16_t dev)
{
	VL53L1X_ERROR status = 0;
	uint8_t Addr = 0x00, tmp;

	for (Addr = 0x2D; Addr <= 0x87; Addr++){
		status = VL53L1_WrByte(dev, Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
	}
	status = VL53L1X_StartRanging(dev);
	tmp  = 0;
	while(tmp==0){
			status = VL53L1X_CheckForDataReady(dev, &tmp);
	}
	status = VL53L1X_ClearInterrupt(dev);
	status = VL53L1X_StopRanging(dev);
	status = VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	status = VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
	return status;
}

VL53L1X_ERROR VL53L1X_ClearInterrupt(uint16_t dev)
{
	VL53L1X_ERROR status = 0;

	status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
	return status;
}

VL53L1X_ERROR VL53L1X_StartRanging(uint16_t dev)
{
	VL53L1X_ERROR status = 0;

	status = VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40);	/* Enable VL53L1X */
	return status;
}

VL53L1X_ERROR VL53L1X_StopRanging(uint16_t dev)
{
	VL53L1X_ERROR status = 0;

	status = VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x00);	/* Disable VL53L1X */
	return status;
}

VL53L1X_ERROR VL53L1X_CheckForDataReady(uint16_t dev, uint8_t *isDataReady)
{
	uint8_t Temp;
	uint8_t IntPol;
	VL53L1X_ERROR status = 0;

	status = VL53L1X_GetInterruptPolarity(dev, &IntPol);
	status = VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, &Temp);
	/* Read in the register to check if a new value is available */
	if (status == 0){
		if ((Temp & 1) == IntPol)
			*isDataReady = 1;
		else
			*isDataReady = 0;
	}
	return status;
}

VL53L1X_ERROR VL53L1X_BootState(uint16_t dev, uint8_t *state)
{
	VL53L1X_ERROR status = 0;
	uint8_t tmp = 0;

	status = VL53L1_RdByte(dev,VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
	*state = tmp;
	return status;
}

VL53L1X_ERROR VL53L1X_GetDistance(uint16_t dev, uint16_t *distance)
{
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = (VL53L1_RdWord(dev,
			VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
	*distance = tmp;
	return status;
}

VL53L1X_ERROR VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t *signalRate)
{
	VL53L1X_ERROR status = 0;
	uint16_t SpNb=1, signal;

	status = VL53L1_RdWord(dev,
		VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
	status = VL53L1_RdWord(dev,
		VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
	*signalRate = (uint16_t) (200.0*signal/SpNb);
	return status;
}

VL53L1X_ERROR VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t *ambPerSp)
{
	VL53L1X_ERROR status = 0;
	uint16_t AmbientRate, SpNb = 1;

	status = VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
	status = VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
	*ambPerSp=(uint16_t) (200.0 * AmbientRate / SpNb);
	return status;
}

VL53L1X_ERROR VL53L1X_GetSignalRate(uint16_t dev, uint16_t *signal)
{
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev,
		VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
	*signal = tmp*8;
	return status;
}

VL53L1X_ERROR VL53L1X_GetSpadNb(uint16_t dev, uint16_t *spNb)
{
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev,
			      VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
	*spNb = tmp >> 8;
	return status;
}

VL53L1X_ERROR VL53L1X_GetAmbientRate(uint16_t dev, uint16_t *ambRate)
{
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
	*ambRate = tmp*8;
	return status;
}

VL53L1X_ERROR VL53L1X_GetRangeStatus(uint16_t dev, uint8_t *rangeStatus)
{
	VL53L1X_ERROR status = 0;
	uint8_t RgSt;

	*rangeStatus = 255;
	status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
	RgSt = RgSt & 0x1F;
	if (RgSt < 24)
		*rangeStatus = status_rtn[RgSt];
	return status;
}

VL53L1X_ERROR VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t *pResult)
{
	VL53L1X_ERROR status = 0;
	uint8_t Temp[17];
	uint8_t RgSt = 255;

	status = VL53L1_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, Temp, 17);
	RgSt = Temp[0] & 0x1F;
	if (RgSt < 24)
		RgSt = status_rtn[RgSt];
	pResult->Status = RgSt;
	pResult->Ambient = (Temp[7] << 8 | Temp[8]) * 8;
	pResult->NumSPADs = Temp[3];
	pResult->SigPerSPAD = (Temp[15] << 8 | Temp[16]) * 8;
	pResult->Distance = Temp[13] << 8 | Temp[14];

	return status;
}

VL53L1X_ERROR VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter)
{
	VL53L1X_ERROR status = 0;
	status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
	return status;
}

VL53L1X_ERROR VL53L1X_SetROI(uint16_t dev, uint16_t X, uint16_t Y)
{
	uint8_t OpticalCenter;
	VL53L1X_ERROR status = 0;

	status =VL53L1_RdByte(dev, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &OpticalCenter);
	if (X > 16)
		X = 16;
	if (Y > 16)
		Y = 16;
	if (X > 10 || Y > 10){
		OpticalCenter = 199;
	}
	status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
	status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
		       (Y - 1) << 4 | (X - 1));
	return status;
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

char VL53L1X_BootState(uint16_t dev, uint8_t *state);

void main(void)
{
	uchar state;
	char Status;
	ServoInit();
	state = 1;
	while(state)//检验传感器是否启动
	{
		Status = VL53L1X_BootState(0x29,&state);
		delayms(2);
	}
	Status = VL53L1X_SensorInit(0x29);//初始化传感器
	Status = VL53L1X_StartRanging(0x29);//开始测距
	while(1)
	{
		while(state == 0) Status = VL53L1X_CheckForDataReady(0x29,&state);
		state = 0;
	//	Status = VL53L1X_GetRangeStatus();
	//	Status = VL53L1X_Getdistance();
		Status = VL53L1X_ClearInterrupt(0x29);
	}
}
