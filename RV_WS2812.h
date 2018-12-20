/*******************
 rvrobotics.ru
 RV_WS2812_ver_1.0
 Ruslan Vorovchenko
*******************/
#ifndef RV_WS2812_H_
#define RV_WS2812_H_

#include "stm32f4xx_hal.h"
#define LED_COUNT 300
#define HIGH 66
#define LOW 26

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define DELAY 45
#define LENGTH DELAY + LED_COUNT * 24
#define BitIsSet(reg, bit) ((reg & (1 << bit)) != 0)

typedef struct rgb {
	uint8_t R;
	uint8_t G;
	uint8_t B;
} Color;

void RV_WS2812_INIT(void);//инициализация
void RV_WS2812_RGB_TO_BUF(Color _Color, uint16_t _Num);//запись данных одного светодиода в DMA буфер
void RV_WS2812_SET(Color _Leds[LED_COUNT]);//запись данных всех светодиодов в буфер
void RV_WS2812_SHOW(void);//отображение буфера данных

#endif
