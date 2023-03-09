/*!
 * @file debugOptions.h
 */
#ifndef _DEBUGOPTIONS_H_
#define _DEBUGOPTIONS_H_

/************************************************************
 * Delay during Setup to slow down Setup
 ************************************************************/ 
#define DEBUG_SETUP_DELAY     00 // Debug Delay during setup


/************************************************************
 * Serial Debugging Config
 ************************************************************/ 
#define DEBUG                 1  // Debug main 
#define DEBUG_ERROR           1  // Error Messages
#define DEBUG_HEARTBEAT       0  // Debug Heartbeat
#define DEBUG_IRQ             0  // Debug IRQ
#define DEBUG_MQTT            0  // Debug MQTT Callback
#define DEBUG_MONITOR         0  // Debug Wifi & MQTT Monitoring
#define DEBUG_SETUP           1  // Debug Setup 
#define DEBUG_PARSER          1  // Debug Command Parser
#define DEBUG_RFM             1  // Debug RFM
#define DEBUG_ISS             1  // Debug ISS Parser

/************************************************************
 * Debugging Macros use Macro "DBG...." instead of "Serial"
 ************************************************************/ 
#define DEBUG_STATE       (DEBUG_HEARTBEAT || DEBUG_IRQ || DEBUG_STATE_CHANGE)

#define DBG               if(DEBUG)Serial 
#define DBG_ERROR         if(DEBUG_ERROR)Serial 
#define DBG_HEARTBEAT     if(DEBUG_HEARTBEAT)Serial 
#define DBG_IRQ           if(DEBUG_IRQ)Serial 
#define DBG_MQTT          if(DEBUG_MQTT)Serial 
#define DBG_MONITOR       if(DEBUG_MONITOR)Serial 
#define DBG_SETUP         if(DEBUG_SETUP)Serial 
#define DBG_PARSER        if(DEBUG_PARSER)Serial 
#define DBG_RFM           if(DEBUG_RFM)Serial 
#define DBG_ISS           if(DEBUG_ISS)Serial 
#endif  // _DEBUGOPTIONS_H_