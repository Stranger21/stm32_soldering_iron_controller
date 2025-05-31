/*
 * iron.h
 *
 *  Created on: Jan 12, 2021
 *      Author: David    Original work by Jose Barros (PTDreamer), 2017
 */

#ifndef IRON_H_
#define IRON_H_
#include "pid.h"
#include "settings.h"

#define TIP_DETECT_TIME         5                           // Pulse before reading adc, to detect tip presence. In uS
#define RUNAWAY_DEPTH           3                           // History count of power values stored to compute the average power for runaway monitor
#define RUNAWAY_RESET_CYCLES    3                           // Cycles to bypass runaway check when temperature was changed
#define SMARTACTIVE_BFSZ        32                          // Buffer size for SmartActive (Each element is sampled every 200ms)
typedef void (*setTemperatureReachedCallback)(uint16_t);
typedef void (*currentModeChanged)(uint8_t);

typedef enum{
  wakeSrc_Shake,
  wakeSrc_Button,
  wakeSrc_Stand,
  wakeSrc_Smart,
}wakeSrc_t;

typedef union{
  uint8_t Flags;                                            // Flag for errors (wrong iron connection, NTC, internal failure...)
  struct{
    unsigned  noIron:1;                                     // No iron detected
    unsigned  NTC_high:1;                                   // NTC too high
    unsigned  NTC_low:1;                                    // NTC too low
    unsigned  V_low:1;                                      // Voltage too low
    unsigned  safeMode:1;                                   // Shut down pwm by some reason. Error condition, first boot...
    unsigned  unused_b5:1;
    unsigned  unused_b6:1;
    unsigned  active:1;                                     // Errors active flag
  };
}IronError_t;

#define ErrorMask    (uint8_t)0b11111                      // mask for used error bit fields (skipping global flag)

#define FLAG_NOERROR      0
#define FLAG_NO_IRON      1
#define FLAG_NTC_HIGH     2
#define FLAG_NTC_LOW      4
#define FLAG_V_LOW        8
#define FLAG_SAFE_MODE    16
#define FLAG_ACTIVE       128

uint8_t AutoSwitchProfile(void);
#define T12_volt        110     // 11.0V 115 ставим около 11 до 12.5 для 115
#define C245_volt       190     // 19.0V
#define C210_volt       140     // 14.0V 210 ставим около 13 до 15 для 210
#define Volt_Tolerance  14      // 1.4V сколько вольт коридор для определения

#define T12_volt_NTCmax        2145     // Напряжения в отсчетах АЦП по формуле ADC = (4095*res)/(pullup+res) pullup равен 4.7 на схеме 2.1s около 10ком 1900 =9ком 2145=11ком
#define T12_volt_NTCmin        1900
#define C245_volt_NTCmax       800     //  245 замкнуто , или не больше 1 КОМ что  то около 800 
#define C245_volt_NTCmin       0
#define C210_volt_NTCmax       4095     // 210 открыто , что около 4095 или около 
#define C210_volt_NTCmin       3900 


void readWake(void);
bool IronWake(wakeSrc_t src);
void resetIronError(void);
void checkIronError(void);
IronError_t getIronErrorFlags(void);
void updatePowerLimit(void);
void runAwayCheck(void);
void setSafeMode(bool mode);
bool getSafeMode(void);
void setCurrentMode(uint8_t mode, uint16_t beep_time);
void setModefromStand(uint8_t mode);
void setUserTemperature(int16_t temperature);
uint16_t getUserTemperature(void);
uint8_t getCurrentMode(void);
int8_t getCurrentPower(void);
void initTimers(void);
void setPwmMul(uint16_t mult);
void setReadDelay(uint16_t delay);
void setReadPeriod(uint16_t period);
void setNoIronValue(uint16_t noiron);
void setSystemTempUnit(bool unit);
bool getSystemTempUnit(void);
void addSetTemperatureReachedCallback(setTemperatureReachedCallback callback);
void addModeChangedCallback(currentModeChanged callback);
void handleIron(void);
void ironInit(TIM_HandleTypeDef *delaytimer, TIM_HandleTypeDef *pwmtimer, uint32_t pwmchannel);
uint8_t getIronOn(void);
void configurePWMpin(uint8_t mode);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *_htim);
TIM_HandleTypeDef* getIronReadTimer(void);
TIM_HandleTypeDef* getIronPwmTimer(void);
void ironSchedulePwmUpdate(void);
bool getBootCompleteFlag(void);
void setBootCompleteFlag(void);
bool getIronError(void);
uint32_t getIronLastErrorTime(void);
void setIronCalibrationMode(uint8_t mode);
bool getIronCalibrationMode(void);
uint32_t getIronPwmOutValue(void);
uint16_t getIronTargetTemperature(void);
uint32_t getIronCurrentModeTimer(void);
bool isIronTargetTempReached(void);
bool getIronShakeFlag(void);
void clearIronShakeFlag(void);
uint32_t getIronLastShakeTime(void);
wakeSrc_t getIronWakeSource(void);
void waitForNextConversion(void);

#endif /* IRON_H_ */
