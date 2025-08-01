/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sin_table.h"
#include "harmonic3_table.h"
#include "oled.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "button.h"
#include "flash.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DMA_SIZE 1600
#define PI 3.1415926f
#define V_TAR_DEF 32.0f
#define FLASH_PARAM_ADDR  ((uint32_t)0x08060000)
#define ADC_FILTER_SIZE 10

typedef struct
{
	uint16_t index_raw;
	uint16_t index1;
	uint16_t index2;
	uint16_t index3;
} Index_TypeDef;


typedef struct
{
	uint16_t duty1;
	uint16_t duty2;
	uint16_t duty3;
	uint16_t dutya;
	uint16_t dutyb;
	uint16_t dutyc;
} Duty_TypeDef;

typedef struct
{
	float data[5];
	uint8_t tail[4];
} Tx_Buf_TypeDef;


typedef struct
{
	float v_filter_buf[DMA_SIZE/(2*ADC_FILTER_SIZE)];
	uint16_t v_buf[DMA_SIZE/2];
	uint16_t i_buf[DMA_SIZE/2];
	float Vrms;
	float Irms;
	float Iavr;
	float Vavr;
	float Vtar;
	float sin_k;
	float sin_n;
	
	float kp;
	float ki;
	float err;
	float err_prev;
	float w0;
	float kv;
} V_Ctrl_TypeDef;

typedef struct
{
	uint8_t sin_k[16];
	uint8_t vrms[8];
	uint8_t irms[8];
	uint8_t sin_n[16];

	uint8_t kp[8];
	uint8_t ki[8];
	uint8_t vtar[8];
	uint8_t kv[8];
	uint8_t w0[8];
} OLED_String_Group;


typedef enum
{
	info_page = 0,
	setting_page,
	in_setting
} PAGE_NUM;

typedef enum
{
	sel_kp = 0,
	sel_ki,
	sel_var,
	sel_kv,
	sel_w0,
	sel_sin_n
} SEL_NUM;


typedef struct
{
	uint8_t page_num;
	uint8_t sel_num;
} OLED_STATE;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
V_Ctrl_TypeDef v_ctrl = {0};

const OLED_String_Group oled_string_group = {
	.sin_k	= "    M:",
	.vrms		= "V_RMS:",
	.irms		= "I_RMS:",
	.sin_n      = "SIN_N :",

	.kp			= "  kp:",
	.ki			= "  ki:",
	.vtar		= "V_Tar:",
	.kv			= "  kv:",
	.w0			= "  f:"
};

volatile uint8_t dma_flag = 0;
volatile uint8_t tim2_flag = 0;
volatile uint16_t tim2_count = 0;
volatile uint8_t tim_1s = 0;
uint16_t dma_buf[DMA_SIZE] = {0};


OLED_STATE oled_state = {0};

Button btn0 = {0};
Button btn1 = {0};
Button btn2 = {0};
Button btn3 = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*-----------------------设置PWM的函数------------------------------*/
void pwm_start(void);
void get_index(Index_TypeDef* index, float w0t);
void get_duty(Index_TypeDef* index, Duty_TypeDef* duty, V_Ctrl_TypeDef* v_ctrl);
void set_compare(Duty_TypeDef* duty);


/*--------------------PI控制电压所用函数--------------------------*/
void V_Ctrl_Init(V_Ctrl_TypeDef* v_ctrl);
void calculate_aver(V_Ctrl_TypeDef* v_ctrl);
void calculate_rms(V_Ctrl_TypeDef* v_ctrl);
void v_pid_update(V_Ctrl_TypeDef* v_ctrl);
void split_buf(V_Ctrl_TypeDef* v_ctrl);


/*------------------------屏幕显示所用函数--------------------------------*/
void display_info_page_title(const OLED_String_Group* oled_string_group);
void display_info(const V_Ctrl_TypeDef* v_ctrl);
void display_setting_page_title(const OLED_String_Group* oled_string_group);
void display_array(const OLED_STATE* oled_state);
void display_star(const OLED_STATE* oled_state);
void display_setting_info(V_Ctrl_TypeDef* v_ctrl);


/*--------------------按键控制所用函数--------------------------*/
void button_group_init(void);
void button_group_update(void);
void button_ui_update(void);

/*------------flash------------*/



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	//初始化PID控制
	V_Ctrl_Init(&v_ctrl);
	
	//开启PWM波                                         
	pwm_start();
	
	//用来中断对w0积分
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim5);
	
//	HAL_TIM_Base_Start_IT(&htim8);
	
	//用来中断显示屏幕
//	WriteFlashData(FLASH_PARAM_ADDR, (uint8_t *)0xffff, sizeof(uint32_t));
	HAL_TIM_Base_Start_IT(&htim2);
	
	//开启ADC采样
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buf, DMA_SIZE);
	OLED_Init();
	OLED_Display_On();
	display_info_page_title(&oled_string_group);
	
	//初始化按键
	button_group_init();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		button_group_update();
		button_ui_update();
		
		if(tim2_flag){
				if(tim2_count >= 100&& tim_1s == 0 )
				{
					tim_1s = 1;
					tim2_count = 0;
				}
				
				tim2_flag =0;
				if(oled_state.page_num == info_page) display_info(&v_ctrl);
				if(oled_state.page_num == in_setting)
				{
					//添加一行Vrms显示，方便调变比
					char v_rms[8];
					OLED_ShowString(0, 7, (uint8_t*)oled_string_group.vrms, 12);
					sprintf(v_rms, "%.3f", v_ctrl.Vrms);
					OLED_ShowString(50, 7, (uint8_t *)v_rms, 12);
				}
		}
		
		if(dma_flag == 1)
		{
			dma_flag = 0;
			split_buf(&v_ctrl);
			calculate_aver(&v_ctrl);
			calculate_rms(&v_ctrl);
			
			v_pid_update(&v_ctrl); 
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buf, DMA_SIZE);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM8)
	{
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	}
	
	
	if(htim->Instance == TIM5)
	{
		static float w0t = 0.0f;
		static Index_TypeDef INDEX = {0};
		static Duty_TypeDef DUTY = {0};
		static float Ts = 1.0f/15000.0f; //15kHz

		w0t += v_ctrl.w0*2*PI * Ts; //15kHz
		if(w0t > 2*PI) w0t -= 2*PI;
		else if(w0t < 0) w0t += 2*PI;
		
		get_index(&INDEX, w0t);
		get_duty(&INDEX, &DUTY, &v_ctrl);
		set_compare(&DUTY);
	}
	if(htim->Instance == TIM2)
	{
		tim2_flag = 1;
		if(tim_1s == 0)
		{
			tim2_count ++ ;
		}
		
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		dma_flag = 1;
	}
}


void get_index(Index_TypeDef* index, float w0t)
{
	index->index_raw = (uint16_t)roundf(w0t / (2.0f * PI) * 1200);
	index->index1 = index->index_raw;
	index->index2 = (index->index_raw + 400) % 1200;
	index->index3 = (index->index_raw + 800) % 1200;

	return;
}

void get_duty(Index_TypeDef* index, Duty_TypeDef* duty, V_Ctrl_TypeDef* v_ctrl)
{
	if(tim_1s == 0)
	{
		duty->duty1 =v_ctrl->sin_k * (sin_table[index->index1] + harmonic3_table[index->index1] / 6.0f) + 1050;
		duty->duty2 = v_ctrl->sin_k* (sin_table[index->index2] + harmonic3_table[index->index2] / 6.0f) + 1050;
		duty->duty3 =v_ctrl->sin_k * (sin_table[index->index3] + harmonic3_table[index->index3] / 6.0f) + 1050;
		
	//	duty->duty1 = 0.7  * (sin_table[index->index1] + harmonic3_table[index->index1] / 6.0f) + 1050;
	//	duty->duty2 = 0.7  * (sin_table[index->index2] + harmonic3_table[index->index2] / 6.0f) + 1050;
	//	duty->duty3 = 0.7  * (sin_table[index->index3] + harmonic3_table[index->index3] / 6.0f) + 1050;

		duty->dutya =  v_ctrl->sin_k * (sin_table[index->index1] + harmonic3_table[index->index1] / 6.0f) + 1050;
		duty->dutyb =  v_ctrl->sin_k* (sin_table[index->index2] + harmonic3_table[index->index2] / 6.0f) + 1050;
		duty->dutyc =  v_ctrl->sin_k * (sin_table[index->index3] + harmonic3_table[index->index3] / 6.0f) + 1050;
	}

	else {
		duty->duty1 = v_ctrl->sin_k  * (sin_table[index->index1] + harmonic3_table[index->index1] / 6.0f) + 1050;
		duty->duty2 = v_ctrl->sin_k  * (sin_table[index->index2] + harmonic3_table[index->index2] / 6.0f) + 1050;
		duty->duty3 = v_ctrl->sin_k  * (sin_table[index->index3] + harmonic3_table[index->index3] / 6.0f) + 1050;
		
		duty->dutya = v_ctrl->sin_n*(sin_table[index->index1] + harmonic3_table[index->index1] / 6.0f) + 1050;
		duty->dutyb = v_ctrl->sin_n*(sin_table[index->index2] + harmonic3_table[index->index2] / 6.0f) + 1050;
		duty->dutyc = v_ctrl->sin_n*(sin_table[index->index3] + harmonic3_table[index->index3] / 6.0f) + 1050;
	}
	
}


void set_compare(Duty_TypeDef* duty)
{
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, duty->duty1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, duty->duty2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, duty->duty3);
	//开启tim1
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duty->dutya);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, duty->dutyb);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, duty->dutyc);
	
	return;
}

void pwm_start(void)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
	
	//开启tim1 整流pwm
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	
	return;
}

void V_Ctrl_Init(V_Ctrl_TypeDef* v_ctrl)
{
	//统一赋0，需要改的单独列出来改
	memset((void *)v_ctrl, 0x0, sizeof(V_Ctrl_TypeDef));
	float kv_flash = *(float*)(FLASH_PARAM_ADDR + 0);
	float   Vtar_flash = *(float*)(FLASH_PARAM_ADDR + 4);
	float   kp_flash   = *(float*)(FLASH_PARAM_ADDR + 8);
	float   ki_flash   = *(float*)(FLASH_PARAM_ADDR + 12);
	float   w0_flash   = *(float*)(FLASH_PARAM_ADDR + 16);
	v_ctrl->sin_k = 0.2f;
	v_ctrl->sin_n = 0.4f;
	
	if (*(uint32_t*)&kv_flash != 0xFFFFFFFF)
			v_ctrl->kv = kv_flash;
	else
			v_ctrl->kv = 43.0f;
	if (*(uint32_t*)&Vtar_flash != 0xFFFFFFFF)
			v_ctrl->Vtar = Vtar_flash;
	else
			v_ctrl->Vtar = V_TAR_DEF;

	if (*(uint32_t*)&kp_flash != 0xFFFFFFFF)
			v_ctrl->kp = kp_flash;
	else
			v_ctrl->kp = 0.02f;

	if (*(uint32_t*)&ki_flash != 0xFFFFFFFF)
			v_ctrl->ki = ki_flash;
	else
			v_ctrl->ki = 0.01f;
	if(*(uint32_t*)&v_ctrl->w0 != 0xFFFFFFFF)
			v_ctrl->w0 = w0_flash;
	else
			v_ctrl->w0 = 50;
}
void split_buf(V_Ctrl_TypeDef* v_ctrl)
{	
	//将DMA采样电压分为I和V
	for(int i=0; i<DMA_SIZE; i++)
	{
		if(i %2 ==0)
		{
			v_ctrl->v_buf[i/2] = dma_buf[i];
		}
		else
		{
			v_ctrl->i_buf[i/2] = dma_buf[i];
		}
	}
	int filter_block = 0;
    for (int i = 0; i + ADC_FILTER_SIZE <= DMA_SIZE/2; i += ADC_FILTER_SIZE)
    {
        v_ctrl->v_filter_buf[filter_block++] = DMA_U16_Filter_ClippedAverage_MedianBase(&v_ctrl->v_buf[i], ADC_FILTER_SIZE, 600);
    }

}
void calculate_aver(V_Ctrl_TypeDef* v_ctrl)
{
	//计算平均值（偏置）
	v_ctrl->Vavr = 0;
	int block_count = DMA_SIZE / 2 / ADC_FILTER_SIZE;

    for (int i = 0; i < block_count; i++) {
        v_ctrl->Vavr += v_ctrl->v_filter_buf[i];
    }

    for (int i = 0; i < DMA_SIZE / 2; i++) {
        float i_v = v_ctrl->i_buf[i] * 3.3f / 4096.0f;
        v_ctrl->Iavr += i_v;
    }

    v_ctrl->Vavr /= block_count;
    v_ctrl->Iavr /= (DMA_SIZE / 2);
}

void calculate_rms(V_Ctrl_TypeDef* v_ctrl)
{
		//去偏置再计算 RMS
		v_ctrl->Vrms = 0;
    v_ctrl->Irms = 0;

    int block_count = DMA_SIZE / 2 / ADC_FILTER_SIZE;

    for (int i = 0; i < block_count; i++) {
        float ac = v_ctrl->v_filter_buf[i] - v_ctrl->Vavr;
        v_ctrl->Vrms += (ac * ac);
    }

    for (int i = 0; i < DMA_SIZE / 2; i++) {
        float i_v = v_ctrl->i_buf[i] * 3.3f / 4096.0f;
        float ac = i_v - v_ctrl->Iavr;
        v_ctrl->Irms += (ac * ac);
    }

    v_ctrl->Vrms = sqrtf(v_ctrl->Vrms / block_count);
    v_ctrl->Vrms = v_ctrl->Vrms * v_ctrl->kv;
    v_ctrl->Irms = sqrtf(v_ctrl->Irms / (DMA_SIZE / 2));
}

void v_pid_update(V_Ctrl_TypeDef* v_ctrl)
{
	v_ctrl->err = v_ctrl->Vtar - v_ctrl->Vrms;
	v_ctrl->sin_k += v_ctrl->kp * (v_ctrl->err - v_ctrl->err_prev) + v_ctrl->ki * v_ctrl->err;

	// 限幅处理
	if (v_ctrl->sin_k < 0.05f) v_ctrl->sin_k = 0.05f;
	if (v_ctrl->sin_k > 0.95f) v_ctrl->sin_k = 0.95f;

	v_ctrl->err_prev = v_ctrl->err;
}


void display_info_page_title(const OLED_String_Group* oled_string_group)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)oled_string_group->sin_k, 16);
	OLED_ShowString(0, 2, (uint8_t*)oled_string_group->vrms, 16);
	OLED_ShowString(0, 4, (uint8_t*)oled_string_group->irms, 16);
	OLED_ShowString(0, 6, (uint8_t*)oled_string_group->sin_n, 16);
}

void display_setting_page_title(const OLED_String_Group* oled_string_group)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)oled_string_group->kp, 12);
	OLED_ShowString(0, 1, (uint8_t*)oled_string_group->ki, 12);
	OLED_ShowString(0, 2, (uint8_t*)oled_string_group->vtar, 12);
	OLED_ShowString(0, 3, (uint8_t*)oled_string_group->kv, 12);
	OLED_ShowString(0, 4, (uint8_t*)oled_string_group->w0, 12);
	OLED_ShowString(0, 5, (uint8_t*)oled_string_group->sin_n, 12);
}

void display_info(const V_Ctrl_TypeDef* v_ctrl)
{
	static char sin_k[8];
	static char v_rms[8];
	static char i_rms[8];
	static char sin_n[8];
	
	sprintf(sin_k, "%.3f", v_ctrl->sin_k);
	sprintf(v_rms, "%.3f", v_ctrl->Vrms);
	sprintf(i_rms, "%.3f", v_ctrl->Irms);
	sprintf(sin_n, "%.3f", v_ctrl->sin_n);
	OLED_ShowString(50, 0, (uint8_t *)sin_k, 16);
	OLED_ShowString(50, 2, (uint8_t *)v_rms, 16);
	OLED_ShowString(50, 4, (uint8_t *)i_rms, 16);
	OLED_ShowString(50, 6, (uint8_t *)sin_n, 16);
}

void display_array(const OLED_STATE* oled_state)
{
	
	//先清除所有箭头
	OLED_ShowString(100, 0, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 1, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 2, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 3, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 4, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 5, (uint8_t*)"   ", 12);
	static uint8_t array[4] = "<--";
	switch(oled_state->sel_num)
	{
		case sel_kp:
			OLED_ShowString(100, 0, array, 12);
			break;
		case sel_ki:
			OLED_ShowString(100, 1, array, 12);
			break;
		case sel_var:
			OLED_ShowString(100, 2, array, 12);
			break;
		case sel_kv:
			OLED_ShowString(100, 3, array, 12);
			break;
		case sel_w0:
			OLED_ShowString(100, 4, array, 12);
			break;
		case sel_sin_n:
			OLED_ShowString(100, 5, array, 12);
			break;
	}
}

void display_star(const OLED_STATE* oled_state)
{
	//先清除所有星星
	OLED_ShowString(100, 0, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 1, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 2, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 3, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 4, (uint8_t*)"   ", 12);
	OLED_ShowString(100, 5, (uint8_t*)"   ", 12);
	static uint8_t array[4] = "*";
	switch(oled_state->sel_num)
	{
		case sel_kp:
			OLED_ShowString(100, 0, array, 12);
			break;
		case sel_ki:
			OLED_ShowString(100, 1, array, 12);
			break;
		case sel_var:
			OLED_ShowString(100, 2, array, 12);
			break;
		case sel_kv:
			OLED_ShowString(100, 3, array, 12);
			break;
		case sel_w0:
			OLED_ShowString(100, 4, array, 12);
			break;
		case sel_sin_n:
			OLED_ShowString(100, 5, array, 12);
			break;
	}
}



void display_setting_info(V_Ctrl_TypeDef* v_ctrl)
{
	static char kp[8];
	static char ki[8];
	static char vtar[8];
	static char kv[8];
	static char w0[8];
	static char sin_n[8];
	
	sprintf(kp, "%.2f", v_ctrl->kp);
	sprintf(ki, "%.2f", v_ctrl->ki);
	sprintf(vtar, "%.2f", v_ctrl->Vtar);
	sprintf(kv, "%5.2f", v_ctrl->kv);
	sprintf(w0, "%.2f", v_ctrl->w0);
	sprintf(sin_n, "%.2f", v_ctrl->sin_n);
	OLED_ShowString(50, 0, (uint8_t *)kp, 12);
	OLED_ShowString(50, 1, (uint8_t *)ki, 12);
	OLED_ShowString(50, 2, (uint8_t *)vtar, 12);
	OLED_ShowString(50, 3, (uint8_t *)kv, 12);
	OLED_ShowString(50, 4, (uint8_t *)w0, 12);
	OLED_ShowString(50, 5, (uint8_t *)sin_n, 12);
}



void button_group_init(void)
{
	Button_Init(&btn0, GPIOE, GPIO_PIN_0, 0);
	Button_Init(&btn1, GPIOE, GPIO_PIN_1, 0);
	Button_Init(&btn2, GPIOE, GPIO_PIN_2, 0);
	Button_Init(&btn3, GPIOE, GPIO_PIN_3, 0);
}
void button_group_update(void)
{
	Button_Update(&btn0);
	Button_Update(&btn1);
	Button_Update(&btn2);
	Button_Update(&btn3);
}


void button_ui_update(void)
{
	//*号键
		ButtonEvent evt0 = Button_GetEvent(&btn0);
		ButtonEvent evt1 = Button_GetEvent(&btn1);
		ButtonEvent evt2 = Button_GetEvent(&btn2);
		ButtonEvent evt3 = Button_GetEvent(&btn3);
		if(evt0  == BUTTON_EVENT_CLICK)
		{
			//切换页面
			if(oled_state.page_num != in_setting)
			{
				if(oled_state.page_num == 1) oled_state.page_num = 0;
				else oled_state.page_num ++;
			}

			//显示页面标题
			if(oled_state.page_num == info_page) display_info_page_title(&oled_string_group);
			else if(oled_state.page_num == setting_page)
			{
				display_setting_page_title(&oled_string_group);
				display_setting_info(&v_ctrl);
				display_array(&oled_state);
			}
		}
		
		//#号键
		if(evt1 == BUTTON_EVENT_CLICK)
		{
			if(oled_state.page_num == setting_page)
			{
				oled_state.page_num = in_setting;
				display_star(&oled_state);
			}
			else if(oled_state.page_num == in_setting)
			{
				oled_state.page_num = setting_page;
				display_array(&oled_state);
				WriteFlashData(FLASH_PARAM_ADDR, (uint8_t *)&(v_ctrl.kv), sizeof(uint32_t));
				WriteFlashData((FLASH_PARAM_ADDR+4),(uint8_t *)&(v_ctrl.Vtar), sizeof(uint32_t));
				WriteFlashData((FLASH_PARAM_ADDR+8),(uint8_t *)&(v_ctrl.kp), sizeof(uint32_t));
				WriteFlashData((FLASH_PARAM_ADDR+12),(uint8_t *)&(v_ctrl.ki), sizeof(uint32_t));
				WriteFlashData((FLASH_PARAM_ADDR+16),(uint8_t *)&(v_ctrl.w0), sizeof(uint32_t));
			}		
		}
		
		//下箭头
		if(evt2 == BUTTON_EVENT_CLICK)
 		{
			
			if(oled_state.page_num == setting_page)
			{	
				if(oled_state.sel_num == 5) oled_state.sel_num = 0;
				else oled_state.sel_num ++;
				display_array(&oled_state);
			}
			else if(oled_state.page_num == in_setting)
			{
				switch(oled_state.sel_num)
				{
					case sel_kp:
						v_ctrl.kp -= 0.01;
						if(v_ctrl.kp < 0.0f) v_ctrl.kp = 0.0f;
						break;
					case sel_ki:
						v_ctrl.ki -= 0.01;
						if(v_ctrl.ki < 0.0f) v_ctrl.ki = 0.0f;
						break;
					case sel_var:
						v_ctrl.Vtar -= 1;
						if(v_ctrl.Vtar < 0.0f) v_ctrl.Vtar = 0.0f;
						break;
					case sel_kv:
						v_ctrl.kv -= 0.1;
						if(v_ctrl.Vtar < 0.0f) v_ctrl.kv = 0.0f;
						break;
					case sel_w0:
						v_ctrl.w0 -= 1;
						if(v_ctrl.w0 < 0.0f) v_ctrl.w0 = 0.0f;
					  break;
					case sel_sin_n:
						v_ctrl.sin_n -= 0.01;
						if(v_ctrl.sin_n < 0.0f) v_ctrl.sin_n = 0.0f;
					break;
				}
				
				display_setting_info(&v_ctrl);
			}
		}
		else if (evt2== BUTTON_EVENT_LONG_PRESS)
		{
			if(oled_state.page_num == in_setting)
			{
				switch(oled_state.sel_num)
				{
					case sel_kp:
						v_ctrl.kp -= 0.1f;
						break;
					case sel_ki:
						v_ctrl.ki -= 0.1f;
						break;
					case sel_var:
						v_ctrl.Vtar -= 1.0f;
						break;
					case sel_kv:
						v_ctrl.kv -= 1.0f;
						break;
					case sel_w0:
						v_ctrl.w0 -= 1.0f;
					  break;
					case sel_sin_n:
						v_ctrl.sin_n -= 0.1;
						if(v_ctrl.sin_n < 0.0f) v_ctrl.sin_n = 0.0f;
					break;
				}
				display_setting_info(&v_ctrl);
			}
		}
		//上箭头
		if(evt3 == BUTTON_EVENT_CLICK)
		{
			if(oled_state.page_num == setting_page)
			{	
				if(oled_state.sel_num == 0) oled_state.sel_num = 5;
				else oled_state.sel_num --;
				display_array(&oled_state);
			}
			else if(oled_state.page_num == in_setting)
			{
				switch(oled_state.sel_num)
				{
					case sel_kp:
						v_ctrl.kp += 0.01;
						break;
					case sel_ki:
						v_ctrl.ki += 0.01;
						break;
					case sel_var:
						v_ctrl.Vtar += 1;
						break;
					case sel_kv:
						v_ctrl.kv += 0.1;
						break;
					case sel_w0:
						v_ctrl.w0 += 1;
						break;
					case sel_sin_n:
						v_ctrl.sin_n += 0.01;
						break;
				}
				
				display_setting_info(&v_ctrl);
			}
		}
		else if (evt3 == BUTTON_EVENT_LONG_PRESS)
		{
			if(oled_state.page_num == in_setting)
			{
				switch(oled_state.sel_num)
				{
					case sel_kp:
						v_ctrl.kp += 0.1f;
						break;
					case sel_ki:
						v_ctrl.ki += 0.1f;
						break;
					case sel_var:
						v_ctrl.Vtar += 1.0f;
						break;
					case sel_kv:
						v_ctrl.kv += 1.0f;
						break;
					case sel_w0:
						v_ctrl.w0 += 1.0f;
					  	break;
				}
				display_setting_info(&v_ctrl);
			}
		}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
