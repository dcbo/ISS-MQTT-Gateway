/************************************************************
 * In this File everything which is related to the 
 * Hardware Design is defined
 * - I2C Interface
 * - Interupt Pin
 ************************************************************/ 
#ifndef _MYHWCONFIG_H_
#define _MYHWCONFIG_H_

/************************************************************
 * I2C 
 * - 800kHz
 * - GPIO Routing
 ************************************************************/ 
#define I2CSPEED         800000  
#define I2C_SDA            12                          // GPIO of SDA Signal
#define I2C_CLK            15                          // GPIO of CLK Signal


/************************************************************
 * RFM69
 * - 
 * - 
 ************************************************************/ 
#define RFM_CS    5
#define RFM_IRQ   2
#define RFM_MOSI 23
#define RFM_MISO 19
#define RFM_CLK  18
#define RFM_HW   false


#endif // _MYHWCONFIG_H_
