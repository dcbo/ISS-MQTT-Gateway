/*!
 * @file prototypes.h
 */

#ifndef _prototypes_H_
#define _prototypes_H_

/************************************************************
 * Prototypes 
 ************************************************************/ 
String composeClientID(void);
void   cronjob(void);
void   dbgout(String);
void   loop(void);
String macToStr(const uint8_t*);
void   monitorConnections(void);
void   mqttCallback(char*, byte* , unsigned int);
void   mqttPub(String, String, boolean);
void   oncePerMinute(void);
void   oncePerSecond(void);
void   oncePerTenSeconds(void);
void   oncePerThirtySeconds(void);
void   resetHandler(void);
void   sendCPUState(boolean);
void   sendNetworkState(boolean);
void   sendSketchState(boolean);
void   setup(void);
void   setupGlobalVars(void);
void   setupGPIO(void);
void   setupIRQ(void);
void   setupMQTT(void);
void   setupOTA(void);
void   setupWIFI(void);
void   setupRadio(void);
void   pollRadio(void);
void   parseIssData(void);
void   sendIssMqtt(void);
#endif
