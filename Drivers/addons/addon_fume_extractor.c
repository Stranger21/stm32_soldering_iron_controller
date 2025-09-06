/*
 * addon_fume_extractor.c
 *
 *  Created on: Apr 18, 2022
 *      Author: KocsisV
 */

#include "main.h"
#include "addon_fume_extractor.h"
#include "iron.h"
#include "settings.h"
#include "buzzer.h"

#ifdef ENABLE_ADDON_FUME_EXTRACTOR

void handleAddonFumeExtractor()
{
	static uint32_t lastActiveTime    = 0u;
	static uint32_t debounce_timer = 0;     // Время начала дребезга
	static bool     was_pressed = false;
    static bool     iron_activated = false; // Паяльник включён (первое нажатие)
    static bool     extractorRequired = false;    // Текущее состояние насоса

    uint32_t now = HAL_GetTick();
    bool wake_pressed = !WAKE_input();  // Текущее состояние кнопки

  switch (getAddons()->fumeExtractorMode)
  {
    case fume_extractor_mode_disabled:
    default:
    {
      // addon disabled
      extractorRequired = false;
      HAL_GPIO_WritePin(EXTRACTOR_GPIO_Port, EXTRACTOR_Pin, GPIO_PIN_RESET);
      break;
    }

    case fume_extractor_mode_auto:
    {
      uint32_t const currentTimeStamp = HAL_GetTick();

      if(getCurrentMode() >= mode_run)
      {
        // if in a mode where extraction is required
        lastActiveTime = currentTimeStamp;
        extractorRequired = true;
      }

      if(extractorRequired &&
         ((lastActiveTime + (5000u * getAddons()->fumeExtractorAfterrun)) < currentTimeStamp))
      {
        // iron has not been in the active state for the configured amount of time, turn it off
        extractorRequired = false;
      }

      HAL_GPIO_WritePin(EXTRACTOR_GPIO_Port, EXTRACTOR_Pin, extractorRequired ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    }

    case fume_extractor_mode_always_on:
    {
      // extractor is always on
      extractorRequired = false; // set to false to override after run
      HAL_GPIO_WritePin(EXTRACTOR_GPIO_Port, EXTRACTOR_Pin, GPIO_PIN_SET);
      break;
    }
	
	case fume_extractor_mode_vacpump:
    {
      // extractor is vacpump 
	  // Сброс состояния, если вышли из рабочего режима
            if (getCurrentMode() < mode_run)
            {
                // Вне рабочего режима — сбрасываем всё
                iron_activated = false;
                extractorRequired = false;
                HAL_GPIO_WritePin(EXTRACTOR_GPIO_Port, EXTRACTOR_Pin, GPIO_PIN_RESET);
                break;
            }

            // Антидребезг
			
            if (wake_pressed != was_pressed)
            {
                debounce_timer = now + 5; // Запускаем таймер 5 мс
                was_pressed = wake_pressed;
            }

            // Если таймер антидребезга не истёк — выходим
            if (now < debounce_timer)
            {
                break;
            }

            // Теперь состояние стабильно — обрабатываем     
                // Кнопка нажата
				if (was_pressed){
                if (getCurrentMode() >= mode_run && !iron_activated)
                {
                    // Это первое нажатие
                    iron_activated = true;   
                }
				
				if (extractorRequired)
                {
                    extractorRequired = false;
					//buzzer_beep(SHORT_BEEP);
                    HAL_GPIO_WritePin(EXTRACTOR_GPIO_Port, EXTRACTOR_Pin, GPIO_PIN_SET);
                }
				}else
            
                // Кнопка отпущена
                if (iron_activated)
                {
                    extractorRequired = true;
                    HAL_GPIO_WritePin(EXTRACTOR_GPIO_Port, EXTRACTOR_Pin, GPIO_PIN_RESET);
                }
			break;
        
    }
  }

}

#endif
