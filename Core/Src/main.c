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
#include <math.h>
#include <string.h>
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
volatile uint8_t dma_flag = 0;
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
	
	
	V_Ctrl_Init(&v_ctrl);
	
	pwm_start();
	
	//用来中断对w0积分
	HAL_TIM_Base_Start_IT(&htim5);
	
	//用来中断对w0积分
	HAL_TIM_Base_Start_IT(&htim8);
	
	//开启ADC采样
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)v_ctrl.v_buf, V_BUFFER_SIZE);
	
	Tx_Buf_TypeDef tx_buf = {0};
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(dma_flag == 1)
    {
      dma_flag = 0;

			calculate_aver(&v_ctrl);
			calculate_rms(&v_ctrl);
			
			v_pid_update(&v_ctrl);
			 
			tx_buf.data[0] = v_ctrl.Vrms;
			tx_buf.data[1] = v_ctrl.sin_k;
			
			tx_buf.tail[0] = 0x00;
			tx_buf.tail[1] = 0x00;
			tx_buf.tail[2] = 0x80;
			tx_buf.tail[3] = 0x7f;
			
			HAL_UART_Transmit(&huart1, (uint8_t *)&tx_buf, sizeof(Tx_Buf_TypeDef), HAL_MAX_DELAY);

			
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
