#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define SDA_IN()  {GPIOA->CRL&=0X0FFFFFFF;GPIOA->CRL|=(u32)8<<28;}
#define SDA_OUT() {GPIOA->CRL&=0X0FFFFFFF;GPIOA->CRL|=(u32)3<<28;}

void Servo_Init(void);//³õÊ¼»¯
void RotateClockwiseX(void);
void RotateCounterClockwiseX(void);
void RotateStopX(void);
void RotateClockwiseY(void);
void RotateStopY(void);
void RotateCounterClockwiseY(void);
void ServoInit(void);
		 				    
#endif
