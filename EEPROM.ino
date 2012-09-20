#include <avr/eeprom.h>

#define EEPROM_CONF_VERSION 164
#define EEPROM_BASE_ADDR 0

uint8_t validEEPROM() {
  struct __eeprom_conf temp;

  eeprom_read_block((void*)&temp, (void*)EEPROM_BASE_ADDR, sizeof(temp));

  /* check version number */
  if (EEPROM_CONF_VERSION != temp.version) {
    return 0;
  }
   /* check size and magic numbers */
  if (temp.size != sizeof(temp) || temp.magic_be != 0xBE || temp.magic_ef != 0xEF) {
    return 0;
  }

  /* verify integrity of temporary copy */
  uint8_t chk = 0;
  for (uint8_t *p = (uint8_t *)&temp; p < ((uint8_t *)&temp + sizeof(temp)); p++) {
    chk ^= *p;
  }
  if (chk != 0) {
    /* checksum failed */
    return 0;
  }
  /* looks good, let's roll! */
  return 1;
}

void readEEPROM() {
  uint8_t i;

  /* do not load values if EEPROM content is invalid */
  if (! validEEPROM()) {
    return;
  }

  eeprom_read_block((void*)&conf, (void*)EEPROM_BASE_ADDR, sizeof(conf));
  for(i=0;i<6;i++) {
    lookupPitchRollRC[i] = (2500+conf.rcExpo8*(i*i-25))*i*(int32_t)conf.rcRate8/2500;
  }
  for(i=0;i<11;i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = MINTHROTTLE + (int32_t)(MAXTHROTTLE-MINTHROTTLE)* lookupThrottleRC[i]/1000;            // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
  }

  #if defined(POWERMETER)
    pAlarm = (uint32_t) conf.powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) conf.pleveldiv; // need to cast before multiplying
  #endif
  #ifdef FLYING_WING
    #ifdef LCD_CONF
      conf.wing_left_mid  = constrain(conf.wing_left_mid, WING_LEFT_MIN,  WING_LEFT_MAX); //LEFT
      conf.wing_right_mid = constrain(conf.wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
    #else // w.o LCD support user may not find this value stored in eeprom, so always use the define value
      conf.wing_left_mid  = WING_LEFT_MID;
      conf.wing_right_mid = WING_RIGHT_MID;
    #endif
  #endif
  #ifdef TRI
    #ifdef LCD_CONF
      conf.tri_yaw_middle = constrain(conf.tri_yaw_middle, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
    #else // w.o LCD support user may not find this value stored in eeprom, so always use the define value
      conf.tri_yaw_middle = TRI_YAW_MIDDLE;
    #endif
  #endif
  #if GPS
    if (f.I2C_INIT_DONE) GPS_set_pids();
  #endif
  #ifdef POWERMETER_HARD
    conf.pleveldivsoft = PLEVELDIVSOFT;
  #endif
  #ifdef POWERMETER_SOFT
     conf.pleveldivsoft = conf.pleveldiv;
  #endif
}

void writeParams(uint8_t blink) {
  conf.version = EEPROM_CONF_VERSION; // make sure we write the current version into eeprom
  /* set size and magic numbers */
  conf.size = sizeof(conf);
  conf.magic_be = 0xBE;
  conf.magic_ef = 0xEF;
  /* recalculate checksum before writing */
  conf.chk = 0;
  uint8_t chk = 0;
  for (uint8_t *p = (uint8_t *)&conf; p < ((uint8_t *)&conf + sizeof(conf)); p++) {
    chk ^= *p;
  }
  conf.chk = chk;

  eeprom_write_block((const void*)&conf, (void*)EEPROM_BASE_ADDR, sizeof(conf));
  readEEPROM();
  if (blink){
    blinkLED(15,20,1);
    #if defined(BUZZER)
      beep_confirmation = 1;
    #endif
  }
  
}

void checkFirstTime() {
  /* check the EEPROM integrity before resetting values */
  if (! validEEPROM()) {
    resetConf();
  }
}

void resetConf() {
  memset(&conf, 0, sizeof(conf));
  conf.P8[ROLL]  = 40;  conf.I8[ROLL] = 30; conf.D8[ROLL]  = 23;
  conf.P8[PITCH] = 40; conf.I8[PITCH] = 30; conf.D8[PITCH] = 23;
  conf.P8[YAW]   = 85;  conf.I8[YAW]  = 45;  conf.D8[YAW]  = 0;
  conf.P8[PIDALT]   = 16; conf.I8[PIDALT]   = 15; conf.D8[PIDALT]   = 7;
  
  conf.P8[PIDPOS]  = POSHOLD_P * 100;     conf.I8[PIDPOS]    = POSHOLD_I * 100;       conf.D8[PIDPOS]    = 0;
  conf.P8[PIDPOSR] = POSHOLD_RATE_P * 10; conf.I8[PIDPOSR]   = POSHOLD_RATE_I * 100;  conf.D8[PIDPOSR]   = POSHOLD_RATE_D * 1000;
  conf.P8[PIDNAVR] = NAV_P * 10;          conf.I8[PIDNAVR]   = NAV_I * 100;           conf.D8[PIDNAVR]   = NAV_D * 1000;

  conf.P8[PIDLEVEL] = 70; conf.I8[PIDLEVEL] = 10; conf.D8[PIDLEVEL] = 100;
  conf.P8[PIDMAG] = 40;
  
  conf.P8[PIDVEL] = 0;  conf.I8[PIDVEL] = 0;  conf.D8[PIDVEL] = 0;
  
  conf.rcRate8 = 90; conf.rcExpo8 = 65;
  conf.rollPitchRate = 0;
  conf.yawRate = 0;
  conf.dynThrPID = 0;
  conf.thrMid8 = 50; conf.thrExpo8 = 0;
  for(uint8_t i=0;i<CHECKBOXITEMS;i++) {conf.activate[i] = 0;}
  conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
  conf.powerTrigger1 = 0;
  #ifdef FLYING_WING
    conf.wing_left_mid  = WING_LEFT_MID; 
    conf.wing_right_mid = WING_RIGHT_MID; 
  #endif
  #ifdef FIXEDWING
    conf.dynThrPID = 50;
    conf.rcExpo8   =  0;
  #endif
  #ifdef TRI
    conf.tri_yaw_middle = TRI_YAW_MIDDLE;
  #endif
  #if defined HELICOPTER || defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
    {
      int16_t s[8] = SERVO_OFFSET;
      for(uint8_t i=0;i<8;i++) conf.servoTrim[i] = s[i];
    }
  #endif
  #if defined(GYRO_SMOOTHING)
    {
      uint8_t s[3] = GYRO_SMOOTHING;
      for(uint8_t i=0;i<3;i++) conf.Smoothing[i] = s[i];
    }
  #endif
  #if defined (FAILSAFE)
    conf.failsave_throttle = FAILSAVE_THROTTLE;
  #endif
  #ifdef VBAT
    conf.vbatscale = VBATSCALE;
    conf.vbatlevel1_3s = VBATLEVEL1_3S;
    conf.vbatlevel2_3s = VBATLEVEL2_3S;
    conf.vbatlevel3_3s = VBATLEVEL3_3S;
    conf.vbatlevel4_3s = VBATLEVEL4_3S;
    conf.no_vbat = NO_VBAT;
  #endif
  #ifdef POWERMETER
    conf.psensornull = PSENSORNULL;
    //conf.pleveldivsoft = PLEVELDIVSOFT; // not neccessary; this gets set in the eeprom read function
    conf.pleveldiv = PLEVELDIV;
    conf.pint2ma = PINT2mA;
  #endif
#ifdef CYCLETIME_FIXATED
  conf.cycletime_fixated = CYCLETIME_FIXATED;
#endif
  writeParams(0); // this will also (p)reset conf version with the current version number again.
}
