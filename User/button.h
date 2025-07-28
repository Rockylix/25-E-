#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f4xx_hal.h" 

#define debounce_ms 50 // 消抖时间，单位毫秒
#define long_press_ms 1000 // 长按时间阈值，单位毫秒

// button event
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_CLICK,
    BUTTON_EVENT_LONG_PRESS
} ButtonEvent;

//button state
typedef enum {
    BUTTON_STATE_IDLE = 0,
    BUTTON_STATE_DEBOUNCE_PRESS,
    BUTTON_STATE_PRESSED,
    BUTTON_STATE_DEBOUNCE_RELEASE
} ButtonState;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t active_level;     // 0 或 1：低电平触发 or 高电平触发
    uint32_t last_tick;       // 上次状态更新时间
    uint32_t press_start_tick;// 按下开始时间
    ButtonState state;
    ButtonEvent event;
    uint8_t debounce_time_ms;   // 消抖时间
    uint16_t long_press_time_ms; // 长按阈值
} Button;

void Button_Init(Button* btn,GPIO_TypeDef* port,uint16_t pin,uint8_t active_level);
void Button_Update(Button* btn);  // 每隔 debounce_ms 调用一次
ButtonEvent Button_GetEvent(Button* btn);

#endif
