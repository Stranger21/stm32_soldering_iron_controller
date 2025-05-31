/*
 * screen_common.h
 *
 *  Created on: Jul 30, 2021
 *      Author: David
 */

#ifndef _GUI_COMMON_H_
#define _GUI_COMMON_H_


#include "settings.h"
#include "board.h"
#include "screens.h"
#include "oled.h"
#include "gui.h"

#define SCREEN_AUTORETURN_TIMEOUT 15000

typedef void (*setterFn)(void*);

typedef struct{
  uint8_t enabled;
  uint8_t index;
  uint8_t update;
  uint32_t timeStep;
  int16_t d[100];
  uint32_t timer;
}plotData_t;

typedef struct{
  uint8_t enabled;
  uint8_t update;
  int8_t xAdd,yAdd;
  int16_t x,y;
  uint32_t timer;
}slide_t;

extern slide_t screenSaver;
extern plotData_t plot;

extern int32_t temp;
extern uint8_t newTip, status, profile, Selected_Tip, lang, backupMode, backupTempUnit, current_lang;
extern int16_t backupTemp;
extern tipData_t backupTip;

void handleOledDim(void);
void wakeOledDim(void);
int longClickReturn(widget_t *w);
int autoReturn_ProcessInput(screen_t * scr, RE_Rotation_t input, RE_State_t *state);
void updatePlot(void);
uint8_t update_GUI_Timer(void);
void set_GUI_profile(uint8_t p);
void resetScreenTimer(void);
void updateScreenTimer(RE_Rotation_t input);
uint8_t checkScreenTimer(uint32_t time);
uint8_t isScreenTimerExpired(void);
void updateAmbientTemp(void);
#endif /* GRAPHICS_GUI_SCREENS_SCREEN_COMMON_H_ */
