/*
 * RobotCoonfig.cpp
 */

#include "RobotConfig.h"
#include <EEPROM.h>
#include <Arduino.h> // for printf

#define SerialDebug Serial   // Stream to send serial debug messages
#define  CONFIG_DEBUG 

 static pidCfg_t    pidConfig;
 static actionCfg_t actionConfig;
 
// write header and save core data
void saveConfig()
{
#ifdef  CONFIG_DEBUG 
  SerialDebug.printf("saving config\n");
#endif   
  uint8_t *cfgPtr;  
  uint16_t offset;
  configPreamble_t preamble = {CONFIG_ID, CONFIG_VERSION};
  cfgPtr = (uint8_t*)&preamble; 
 
  for(offset=0; offset < sizeof( preamble); offset++)  {
     EEPROM.write( offset,  cfgPtr[offset] );    // preamble is stored at the start of EEPROM memory space          
  } 
  cfgPtr = (uint8_t*)&pidConfig;    
  SerialDebug.printf("in Save pid offset = %d\n",offset );  
  for(uint16_t i=0; i < sizeof( pidConfig); i++)  
     EEPROM.write( offset++ , cfgPtr[i] );   
   cfgPtr = (uint8_t*)&actionConfig;    
   SerialDebug.printf("in Save action offset = %d\n",offset );
   for(uint16_t i=0; i < sizeof( actionConfig); i++)  
       EEPROM.write( offset++ , cfgPtr[i] );      
}


// returns true if valid config data can be restored from EEPROM
// will return false if eeprom has not been configured or invalid version 
bool restoreConfig()
{
#ifdef  CONFIG_DEBUG 
  SerialDebug.printf("restoring config\n");
#endif  
  bool ret = false;
  uint8_t *cfgPtr; 
  uint16_t offset;
  // first check if config is valid 
  configPreamble_t preamble;
  cfgPtr = (uint8_t*)&preamble;
  //SerialDebug.printf("reading: ");
  for(offset=0; offset < sizeof( preamble); offset++)  {
     cfgPtr[offset] = EEPROM.read( offset);    // preamble is stored at the start of EEPROM memory space          
   }      
   if(preamble.configId == CONFIG_ID)
   {
#ifdef  CONFIG_DEBUG     
      SerialDebug.printf("Using config from EEPROM\n");
#endif        
      if(preamble.version == CONFIG_VERSION)
      {                 
          uint8_t *cfgPtr = (uint8_t*)&pidConfig;
          SerialDebug.printf("in Restore pid offset = %d\n",offset );  
          for(uint16_t i=0; i < sizeof( pidCfg_t); i++)  {
             cfgPtr[i] = EEPROM.read(offset++);                    
          }         
          cfgPtr = (uint8_t*)&actionConfig;
          SerialDebug.printf("in Restore action offset = %d\n",offset );  
          for(uint16_t i=0; i < sizeof( actionCfg_t); i++)  {
             cfgPtr[i] = EEPROM.read( offset++ );                    
          }                                
#ifdef  CONFIG_DEBUG 
         SerialDebug.printf("after reading eeprom  Kp= %d,Ki= %d,Kd= %d,Ko= %d",pidConfig.Kp,pidConfig.Ki,pidConfig.Kd,pidConfig.Ko );  
         SerialDebug.printf("  Action= %d, speedIndex= %d, distanceIndex= %d\n", actionConfig.actionIndex, actionConfig.speedIndex,actionConfig.distanceIndex );          
#endif 
      ret = true; 
      }
      else
      {  
        SerialDebug.printf("Config version mismatch!\n");  
      }    
   } 
   else
   {  
       SerialDebug.printf("Using default config !!!\n");   
       setConfigDefaults();      
   }        
   return ret;
}

void setConfigDefaults( )
{
  pidConfig.Kp = 20;
  pidConfig.Kd = 12;
  pidConfig.Ki = 0;
  pidConfig.Ko = 20;  

  actionConfig.actionIndex  = 0; // move
  actionConfig.speedIndex   = 1; // 20cm/s
  actionConfig.distanceIndex = 1; // 50cm
}

pidCfg_t    *getPidConfig()
{
  return &pidConfig;
}

actionCfg_t *getActionConfig()
{
  return &actionConfig;
}

bool cfgIsChanged( actionCfg_t *cfgPtr)
{
         SerialDebug.printf("  Action= %d", cfgPtr->actionIndex );
         SerialDebug.printf(", speedIndex= %d", cfgPtr->speedIndex );
         SerialDebug.printf(", distanceIndex= %d\n", cfgPtr->distanceIndex ); 
         SerialDebug.printf("  Action= %d", actionConfig.actionIndex );
         SerialDebug.printf(", speedIndex= %d", actionConfig.speedIndex );
         SerialDebug.printf(", distanceIndex= %d\n", actionConfig.distanceIndex ); 
         
  return (cfgPtr->actionIndex != actionConfig.actionIndex ||
          cfgPtr->speedIndex  != actionConfig.speedIndex ||
          cfgPtr->distanceIndex  != actionConfig.distanceIndex);
}



bool cfgIsChanged( pidCfg_t *cfgPtr){

         SerialDebug.printf(" Kp= %d",  cfgPtr->Kp );
         SerialDebug.printf(", Ki= %d", cfgPtr->Ki );
         SerialDebug.printf(", Kd= %d", cfgPtr->Kd );
         SerialDebug.printf(", Ko= %d", cfgPtr->Ko ); 
         
  return (cfgPtr->Kp != pidConfig.Kp ||
          cfgPtr->Ki != pidConfig.Ki ||
          cfgPtr->Kd != pidConfig.Kd ||
          cfgPtr->Ko != pidConfig.Ko );
}

void updateConfig(actionCfg_t *cfgPtr)
{
   actionConfig.actionIndex   = cfgPtr->actionIndex;
   actionConfig.speedIndex    = cfgPtr->speedIndex; 
   actionConfig.distanceIndex = cfgPtr->distanceIndex ;
}

void updateConfig(pidCfg_t *cfgPtr)
{
   pidConfig.Kp = cfgPtr->Kp;
   pidConfig.Ki =  cfgPtr->Ki; 
   pidConfig.Kd =   cfgPtr->Kd; 
   pidConfig.Ko  = cfgPtr->Ko;
}

void showConfig()
{
   cfgIsChanged(&pidConfig);
   cfgIsChanged(&actionConfig);
}

