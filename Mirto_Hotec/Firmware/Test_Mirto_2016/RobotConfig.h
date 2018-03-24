/*
 * Configuration data
 */

#ifndef config_h
#define config_h

#include <stdint.h>

const int16_t  CONFIG_ID      = 0x135; // unique number indicating that this is valid config
const uint16_t CONFIG_VERSION = 0x2;   // version number incremented when config format changes 

typedef struct {
  int16_t  configId;            // a magic value to identify valid config data
  uint16_t version;             // the configuration version number    
} configPreamble_t;

typedef struct {
   int16_t Kp;
   int16_t Ki;
   int16_t Kd;
   int16_t Ko;      // scaling parameter
   uint8_t rfu[32];    // bytes reserved for future use    
} pidCfg_t;

typedef struct {
   int16_t actionIndex;
   int16_t speedIndex;
   int16_t distanceIndex;
   uint8_t rfu[32];    // bytes reserved for future use    
} actionCfg_t;

typedef struct {
  configPreamble_t  preamble;
  pidCfg_t     pid; 
  actionCfg_t  action;   
}  robotCfgData_t;


//#define CONFIG_DEBUG

void saveConfig();
bool restoreConfig();
void setConfigDefaults( );
pidCfg_t    *getPidConfig();
actionCfg_t *getActionConfig();
bool cfgIsChanged( actionCfg_t * cfgPtr);
void updateConfig(pidCfg_t *cfgPtr);
void updateConfig(actionCfg_t *cfgPtr);
void showConfig();

 
#endif


