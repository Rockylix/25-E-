#include "button.h"

//read button state
static uint8_t Read_Button_Pin(Button* btn)
{
    return HAL_GPIO_ReadPin(btn->port, btn->pin);
}

// 按键结构体初始化
void Button_Init(Button* btn, GPIO_TypeDef* port, uint16_t pin, uint8_t active_level)
{
    btn->port = port;
    btn->pin = pin;
    btn->active_level = active_level;
    btn->debounce_time_ms = debounce_ms;
    btn->long_press_time_ms = long_press_ms;
    btn->last_tick = HAL_GetTick();
    btn->state = BUTTON_STATE_IDLE;
    btn->event = BUTTON_EVENT_NONE;
}

// 
void Button_Update(Button* btn)
{
    uint32_t now = HAL_GetTick();
    if (now - btn->last_tick < btn->debounce_time_ms)
        return;

    btn->last_tick = now;
    uint8_t level = Read_Button_Pin(btn);
    uint8_t pressed = (level == btn->active_level);

    switch (btn->state)
    {
    case BUTTON_STATE_IDLE:
        if (pressed) {
            btn->state = BUTTON_STATE_DEBOUNCE_PRESS;
        }
        break;

    case BUTTON_STATE_DEBOUNCE_PRESS:
        if (pressed) {
            btn->state = BUTTON_STATE_PRESSED;
            btn->press_start_tick = now;
        } else {
            btn->state = BUTTON_STATE_IDLE;
        }
        break;

    case BUTTON_STATE_PRESSED:
        if (!pressed) {
            btn->state = BUTTON_STATE_DEBOUNCE_RELEASE;
        } else if ((now - btn->press_start_tick) >= btn->long_press_time_ms) {
            btn->event = BUTTON_EVENT_LONG_PRESS;
            btn->state = BUTTON_STATE_DEBOUNCE_RELEASE; // 长按后等抬起
        }
        break;

    case BUTTON_STATE_DEBOUNCE_RELEASE:
        if (!pressed) {
            if ((now - btn->press_start_tick) < btn->long_press_time_ms) {
                btn->event = BUTTON_EVENT_CLICK;
            }
            btn->state = BUTTON_STATE_IDLE;
        } else {
            btn->state = BUTTON_STATE_PRESSED; // 可能抖动回去了
        }
        break;
    }
}

ButtonEvent Button_GetEvent(Button* btn)
{
    ButtonEvent evt = btn->event;
    btn->event = BUTTON_EVENT_NONE;  // 清除事件
    return evt;
}
