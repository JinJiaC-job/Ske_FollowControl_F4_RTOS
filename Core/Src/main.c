/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lkmoto.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "ids830can.h"
#include "Sensors_reading.h"
#include "fourier_series_traj_exciting.h"
#include "esp8266.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define EXCITE_TRAJ_EXPERI
//#define wifi_test
// #define compensation_experi
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
#ifdef wifi_test
	//WIFIģ���ʼ��
	HAL_UART_Receive_IT(&huart3, &temp_rx, 1);
	HAL_TIM_Base_Start_IT(&htim4);
  
	if(esp8266_server_config())
	{
		printf("WIFIģ�����������ʧ�ܣ�\r\n");
	}
  
	HAL_Delay(10000);
	
	while(1){
	HAL_Delay(1000);
	esp8266_send_cmd("AT+CIPSEND=0,20","OK",200);  //����ָ�����ȵ�����
	u3_printf("This is a message\r\n");
	}
	
//	USART3_RX_STA=0;
//	if(USART3_RX_STA&0X8000)//���յ��ڴ���Ӧ����
//	{
//		printf("%s", USART3_RX_BUF);
//		USART3_RX_STA=0;
//	}
#endif
	
	
#ifdef EXCITE_TRAJ_EXPERI
	CAN_Filter_Init();
	traj_exciting_init();
//���е��motor1~motor5
 	for(int i=2;i<=6;i++)
 	{
 		motor_run(i);
 	}
#endif

#ifdef compensation_experi
	CAN_Filter_Init();
//���е��motor1~motor5
 	for(int i=2;i<=6;i++)
 	{
 		motor_run(i);
 	}
	
	//�������ʼλ��	
  // ske_base_position();
	HAL_Delay(1000);
#endif

#ifdef SET_ZERO_POSITION
//д�뵱ǰλ�õ�ROM��Ϊ���(���д���Ӱ��оƬ������������Ƶ��ʹ��)
 	HAL_Delay(1000);
 	for(int i=1;i<=6;i++)
 	{
 		write_current_position_to_rom(i);
 		HAL_Delay(500);
 	}
	printf("zero point set successfully!");
#endif

#ifdef EXCITE_TRAJ_EXPERI
//�������ʼλ��	
  ske_base_position();

//�ȶ�ȡһ�θ��ؽ�����	
	for(int i=1; i<=6; i++)
	{
		if(i == 1)
		{
			LinearActuator_read_position(i);
			LinearActuator_read_CurrentandSpeed(i);
			pressure_SensorReading();
		}
		else
		{
			read_status2(i);
			read_angle(i);
		}
	}
	
//������ʱ���жϣ�ÿ��0.05s����һ�ο�������
	HAL_TIM_Base_Start_IT(&htim2);
  printf("Exciting traj experiment start!!!\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (motor_control_k < 400)
	while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// compensation_singleJoint_GF(5);
  
	 pressure_SensorReading();
//	 printf("LinearActuator_Pressure0 = %.3fV \r\n", ADC_Pressure_Value[0]);
//	 HAL_Delay(100);
#ifdef EXCITE_TRAJ_EXPERI	
//		read_all_joint_message();
//		for(int i=1; i<=6; i++)
//		{
//			if(i == 1)
//			{
//				LinearActuator_read_position(i);
//				LinearActuator_read_CurrentandSpeed(i);
//				pressure_SensorReading();
//			}
//			else
//				read_status2(i);
//			  read_angle(i);
//		}

//		if(motor_control_k > 400)
//		{
//			printf("Exciting traj experiment end!!!\r\n");
//			break;
//		}
#endif
  }
	
#ifdef EXCITE_TRAJ_EXPERI
	printf("Exciting traj experiment end!!!\r\n");
	HAL_Delay(500);
	for(int i=1; i<=6; i++)
	{
		if(i == 1)
			LinearActuator_startRun_maxspeed_position(i, 0, 30);
		
		else if(i == 4)
			angle_close_loop_with_speed(i, 0, 30);
		else if(i == 5)
			angle_close_loop_with_speed(i, 0, 30);
		else if(i == 6)
			angle_close_loop_with_speed(i, 0, 30);
		
		else
			angle_close_loop_with_speed(i, 0, 30);
	}
	printf("Zero position!!!\n");
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
