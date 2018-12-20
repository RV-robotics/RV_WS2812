/*******************
 rvrobotics.ru
 RV_WS2812_ver_1.0
 Ruslan Vorovchenko
*******************/
#include "RV_WS2812.h"

extern TIM_HandleTypeDef htim1;

uint16_t BUF_DMA[LENGTH] = {0};
Color LEDS[LED_COUNT] = {0};

//инициализация
void RV_WS2812_INIT(void) {
  for (int i = DELAY; i < LENGTH; i++) 
		BUF_DMA[i] = LOW;
}
//запись данных одного светодиода в DMA буфер
void RV_WS2812_RGB_TO_BUF(Color _Color, uint16_t _Num) {
  for(volatile uint16_t i = 0; i < 8; i++) {
    if (BitIsSet(_Color.R, (7 - i)) == 1)
      BUF_DMA[DELAY + _Num * 24 + i + 8] = HIGH;
    else
      BUF_DMA[DELAY + _Num * 24 + i + 8] = LOW;
    if (BitIsSet(_Color.G, (7 - i)) == 1)
      BUF_DMA[DELAY + _Num * 24 + i] = HIGH;
    else
      BUF_DMA[DELAY + _Num * 24 + i] = LOW;
    if (BitIsSet(_Color.B, (7 - i)) == 1)
      BUF_DMA[DELAY + _Num * 24 + i + 16] = HIGH;
    else
      BUF_DMA[DELAY + _Num * 24 + i + 16] = LOW;
  }
}
//запись данных всех светодиодов в буфер
void RV_WS2812_SET(Color _Leds[LED_COUNT]) {
  for(uint32_t i = 0; i < LED_COUNT; i++)
    RV_WS2812_RGB_TO_BUF(_Leds[i], i);
}
//отображение буфера данных
void RV_WS2812_SHOW(void) {
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) &BUF_DMA, LENGTH);
}
