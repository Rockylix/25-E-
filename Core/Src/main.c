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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define V_BUFFER_SIZE 840
#define V_FILTER_SIZE 5

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
} Duty_TypeDef;

typedef struct
{
	float data[5];
	uint8_t tail[4];
} Tx_Buf_TypeDef;


typedef struct
{
	uint16_t v_buf[V_BUFFER_SIZE];
	float v_filted[V_BUFFER_SIZE/V_FILTER_SIZE];
	float Vrms;
	float Vavr;
	float Vtar;
	float sin_k;
	
	float kp;
	float ki;
	float err;
	float err_prev;
} V_Ctrl_TypeDef;

typedef struct
{
	uint8_t sin_k[16];
	uint8_t vrms[8];

	uint8_t kp[8];
	uint8_t ki[8];
	uint8_t vtar[8];
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
	sel_var
} SEL_NUM;


typedef struct
{
	uint8_t page_num;
	uint8_t sel_num;
} OLED_STATE;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926f
#define V_TAR_DEF 12.0f;
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

	.kp			= "   kp:",
	.ki			= "   ki:",
	.vtar		= "V_Tar:"
};

volatile uint8_t dma_flag = 0;

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
  /* USER CODE BEGIN 2 */
	
	//初始化PID控制
	V_Ctrl_Init(&v_ctrl);
	
	//开启PWM波
	pwm_start();
	
	//用来中断对w0积分
	HAL_TIM_Base_Start_IT(&htim5);
	
	//用来中断对w0积分
	HAL_TIM_Base_Start_IT(&htim8);
	
	//开启ADC采样
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)v_ctrl.v_buf, V_BUFFER_SIZE);
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
		
		//*号键
		if(Button_GetEvent(&btn0) == BUTTON_EVENT_CLICK)
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
		if(Button_GetEvent(&btn1) == BUTTON_EVENT_CLICK)
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
			}
		}
		
		//下箭头
		if(Button_GetEvent(&btn2) == BUTTON_EVENT_CLICK)
		{
			
			if(oled_state.page_num == setting_page)
			{	
				if(oled_state.sel_num == 3) oled_state.sel_num = 0;
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
						v_ctrl.Vtar -= 0.1;
						if(v_ctrl.Vtar < 0.0f) v_ctrl.Vtar = 0.0f;
						break;
				}
				
				display_setting_info(&v_ctrl);
			}

		}
		
		//上箭头
		if(Button_GetEvent(&btn3) == BUTTON_EVENT_CLICK)
		{
			if(oled_state.page_num == setting_page)
			{	
				if(oled_state.sel_num == 0) oled_state.sel_num = 3;
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
						v_ctrl.Vtar += 0.1;
						break;
				}
				
				display_setting_info(&v_ctrl);
			}
		}
		
		
		if(dma_flag == 1)
    {
      dma_flag = 0;

			calculate_aver(&v_ctrl);
			calculate_rms(&v_ctrl);
			
			v_pid_update(&v_ctrl); 
			
			if(oled_state.page_num == info_page) display_info(&v_ctrl);
			
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)v_ctrl.v_buf, V_BUFFER_SIZE);
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
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	}
	
	
	if(htim->Instance == TIM5)
	{
		static float w0 = 2.0f * PI * 50.0f;
		static float w0t = 0.0f;
		static Index_TypeDef INDEX = {0};
		static Duty_TypeDef DUTY = {0};
		
		w0t += w0 * 0.0001f; //TIM5的周期为10K
		if(w0t > 2*PI) w0t -= 2*PI;
		else if(w0t < 0) w0t += 2*PI;
		
		get_index(&INDEX, w0t);
		get_duty(&INDEX, &DUTY, &v_ctrl);
		set_compare(&DUTY);
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
	index->index_raw = (uint16_t)roundf(w0t / (2.0f * PI) * 600);
	index->index1 = index->index_raw;
	index->index2 = (index->index_raw + 200) % 600;
	index->index3 = (index->index_raw + 400) % 600;

	return;
}

void get_duty(Index_TypeDef* index, Duty_TypeDef* duty, V_Ctrl_TypeDef* v_ctrl)
{
	duty->duty1 = v_ctrl->sin_k * (sin_table[index->index1] + harmonic3_table[index->index1] / 6.0f) + 500;
	duty->duty2 = v_ctrl->sin_k * (sin_table[index->index2] + harmonic3_table[index->index2] / 6.0f) + 500;
	duty->duty3 = v_ctrl->sin_k * (sin_table[index->index3] + harmonic3_table[index->index3] / 6.0f) + 500;
	
	return;
}


void set_compare(Duty_TypeDef* duty)
{
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, duty->duty1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, duty->duty2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, duty->duty3);
	
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
	
	return;
}

void V_Ctrl_Init(V_Ctrl_TypeDef* v_ctrl)
{
	//统一赋0，需要改的单独列出来改
	memset((void *)v_ctrl, 0x0, sizeof(V_Ctrl_TypeDef));
	v_ctrl->Vtar = V_TAR_DEF;
	v_ctrl->kp = 0.01f;
	v_ctrl->ki = 0.01f;
	v_ctrl->sin_k = 0.5f;
}

void calculate_aver(V_Ctrl_TypeDef* v_ctrl)
{
	//计算平均值（偏置）
	v_ctrl->Vavr = 0;
	for(int i=0; i<(V_BUFFER_SIZE); i++)
	{
			float v = v_ctrl->v_buf[i] * 3.3f / 4096.0f;
			v_ctrl->Vavr += v;
	}

	v_ctrl->Vavr /= V_BUFFER_SIZE;
}

void calculate_rms(V_Ctrl_TypeDef* v_ctrl)
{
	//去偏置再计算 RMS
	v_ctrl->Vrms = 0;
	for(int i=0; i<V_BUFFER_SIZE; i++)
	{
			float v = v_ctrl->v_buf[i] * 3.3f / 4096.0f;
			float ac = v - v_ctrl->Vavr;
			v_ctrl->Vrms += (ac * ac);
	}
	v_ctrl->Vrms = sqrtf(v_ctrl->Vrms/V_BUFFER_SIZE);
	v_ctrl->Vrms = v_ctrl->Vrms*100;
}

void v_pid_update(V_Ctrl_TypeDef* v_ctrl)
{
	v_ctrl->err = v_ctrl->Vtar - v_ctrl->Vrms;
	v_ctrl->sin_k += v_ctrl->kp * (v_ctrl->err - v_ctrl->err_prev) + v_ctrl->ki * v_ctrl->err;

	// 限幅处理
	if (v_ctrl->sin_k < 0.2f) v_ctrl->sin_k = 0.2f;
	if (v_ctrl->sin_k > 0.8f) v_ctrl->sin_k = 0.8f;

	v_ctrl->err_prev = v_ctrl->err;
}


void display_info_page_title(const OLED_String_Group* oled_string_group)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)oled_string_group->sin_k, 16);
	OLED_ShowString(0, 2, (uint8_t*)oled_string_group->vrms, 16);
}

void display_setting_page_title(const OLED_String_Group* oled_string_group)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)oled_string_group->kp, 16);
	OLED_ShowString(0, 2, (uint8_t*)oled_string_group->ki, 16);
	OLED_ShowString(0, 4, (uint8_t*)oled_string_group->vtar, 16);
}

void display_info(const V_Ctrl_TypeDef* v_ctrl)
{
	static char sin_k[8];
	static char v_rms[8];
	
	sprintf(sin_k, "%.3f", v_ctrl->sin_k);
	sprintf(v_rms, "%.3f", v_ctrl->Vrms);
	OLED_ShowString(50, 0, (uint8_t *)sin_k, 16);
	OLED_ShowString(50, 2, (uint8_t *)v_rms, 16);
}

void display_array(const OLED_STATE* oled_state)
{
	
	//先清除所有箭头
	OLED_ShowString(100, 0, (uint8_t*)"   ", 16);
	OLED_ShowString(100, 2, (uint8_t*)"   ", 16);
	OLED_ShowString(100, 4, (uint8_t*)"   ", 16);
	OLED_ShowString(100, 6, (uint8_t*)"   ", 16);
	static uint8_t array[4] = "<--";
	switch(oled_state->sel_num)
	{
		case sel_kp:
			OLED_ShowString(100, 0, array, 16);
			break;
		case sel_ki:
			OLED_ShowString(100, 2, array, 16);
			break;
		case sel_var:
			OLED_ShowString(100, 4, array, 16);
			break;
	}
}

void display_star(const OLED_STATE* oled_state)
{
	//先清除所有星星
	OLED_ShowString(100, 0, (uint8_t*)"   ", 16);
	OLED_ShowString(100, 2, (uint8_t*)"   ", 16);
	OLED_ShowString(100, 4, (uint8_t*)"   ", 16);
	OLED_ShowString(100, 6, (uint8_t*)"   ", 16);
	static uint8_t array[4] = "*";
	switch(oled_state->sel_num)
	{
		case sel_kp:
			OLED_ShowString(100, 0, array, 16);
			break;
		case sel_ki:
			OLED_ShowString(100, 2, array, 16);
			break;
		case sel_var:
			OLED_ShowString(100, 4, array, 16);
			break;
	}
}



void display_setting_info(V_Ctrl_TypeDef* v_ctrl)
{
	static char kp[8];
	static char ki[8];
	static char vtar[8];
	
	sprintf(kp, "%.2f", v_ctrl->kp);
	sprintf(ki, "%.2f", v_ctrl->ki);
	sprintf(vtar, "%.1f", v_ctrl->Vtar);
	OLED_ShowString(50, 0, (uint8_t *)kp, 16);
	OLED_ShowString(50, 2, (uint8_t *)ki, 16);
	OLED_ShowString(50, 4, (uint8_t *)vtar, 16);
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
