/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATUS_SYSTEM_READY 0x01
#define STATUS_DC_ON 0x10
#define STATUS_QUIT_DC_ON 0x08
#define STATUS_INVERTER_ON 0x40
#define STATUS_QUIT_INVERTER_ON 0x20
#define STATUS_ERROR 0x02

#define CONTROL_DC_ON 0x02
#define CONTROL_ENABLE 0x05
#define CONTROL_INVERTER_ON 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//Defines for code control
#define HARDCODED_CYCLING_SETPOINT_VALUES
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan3;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan3;
#ifdef HARDCODED_CYCLING_SETPOINT_VALUES
int rpmsetpoint[10] = { 1000, 7000, 2000, 10000, 3000, 5000, 500, 8000, 4200,
		9000 };
int rpmIndex = 0;
#endif

enum State {
	RESET_ERROR = 0,
	WAIT_SYSTEM_READY,
	SET_DC_ON,
	WAIT_DC_ON,
	WAIT_QUIT_DC_ON,
	RESET_POINTS,
	SET_ENABLE_AND_INV_ON,
	WAIT_INVERTER_ON,
	WAIT_QUIT_INVERTER_ON,
	SET_POINTS,
	INV_ERROR
} ;

uint8_t msg1[8];
uint8_t msg2[8];
uint8_t inv1_av1_msg[8];
uint8_t inv1_av2_msg[8];
uint8_t inv2_av1_msg[8];
uint8_t inv2_av2_msg[8];

int rpm1setpoint = 0;
int setpoint1inc = 5;
int currentsetpoint1 = 0;

int rpm2setpoint = 0;
int setpoint2inc = 5;
int currentsetpoint2 = 0;
uint32_t TxMailbox;
int inv1Message = 0;
int inv2Message = 0;
uint16_t id1 = 0x300;
uint16_t id2 = 0x400;
enum State state = RESET_ERROR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void sendCANMessage(uint8_t data[], uint16_t id);
void CANInit();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan3) {
	CAN_RxHeaderTypeDef *rxheader = 0;
	uint8_t aData[8];
	HAL_CAN_GetRxMessage(hcan3, CAN_RX_FIFO0, rxheader, aData);
		switch (rxheader->StdId) {
		case 0x283:
			inv1Message= 1;
			for(int i = 0; i < 8; i++)
				inv1_av1_msg[i] = aData[i];
			break;
		case 0x285:
			inv1Message= 1;
			for(int i = 0; i < 8; i++)
				inv1_av2_msg[i] = aData[i];
			break;
		case 0x284:
			inv2Message= 1;
			for(int i = 0; i < 8; i++)
				inv2_av1_msg[i] = aData[i];
			break;
		case 0x286:
			inv2Message= 1;
			for(int i = 0; i < 8; i++)
				inv2_av2_msg[i] = aData[i];
			break;
		}
}


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
  MX_CAN3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	CANInit();
	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		switch (state) {
		case RESET_ERROR:
			// 0
			msg1[0] = 0;
			msg1[1] = 0x08;
			msg1[2] = 0;
			msg1[3] = 0;
			msg1[4] = 0;
			msg1[5] = 0;
			msg1[6] = 0;
			msg1[7] = 0;

			msg2[0] = 0;
			msg2[1] = 0x08;
			msg2[2] = 0;
			msg2[3] = 0;
			msg2[4] = 0;
			msg2[5] = 0;
			msg2[6] = 0;
			msg2[7] = 0;

			currentsetpoint1 = 0;
			rpm1setpoint = 0;

			currentsetpoint2 = 0;
			rpm2setpoint = 0;

			if ((inv1Message == 1 && (inv1_av1_msg[1] & 0x02) == 0)
			&& (inv2Message == 1 && (inv2_av1_msg[1] & 0x02) == 0))
				state = WAIT_SYSTEM_READY;
			break;
		case WAIT_SYSTEM_READY:
			// 1
			if ((inv1_av1_msg[1] & STATUS_SYSTEM_READY)
				&& (inv2_av1_msg[1] & STATUS_SYSTEM_READY))
				state = SET_DC_ON;
			break;
		case SET_DC_ON:
			// 2
			msg1[1] = CONTROL_DC_ON;
			msg2[1] = CONTROL_DC_ON;
			state = WAIT_DC_ON;
			break;
		case WAIT_DC_ON:
			// 3
			if ((inv1_av1_msg[1] & STATUS_DC_ON) && (inv2_av1_msg[1] & STATUS_DC_ON))
				state = WAIT_QUIT_DC_ON;
			else
				state = SET_DC_ON;
			break;
		case WAIT_QUIT_DC_ON:
			// 4
			if ((inv1_av1_msg[1] & STATUS_QUIT_DC_ON) && (inv2_av1_msg[1] & STATUS_QUIT_DC_ON))
				state = RESET_POINTS;
			else
				state = WAIT_DC_ON;
			break;
		case RESET_POINTS:
			// 5
			state = SET_ENABLE_AND_INV_ON;
			break;
		case SET_ENABLE_AND_INV_ON:
			// 6
			msg1[1] |= CONTROL_ENABLE | CONTROL_INVERTER_ON;
			msg2[1] |= CONTROL_ENABLE | CONTROL_INVERTER_ON;
			state = WAIT_INVERTER_ON;
			break;
		case WAIT_INVERTER_ON:
			// 7
			if ((inv1_av1_msg[1] & STATUS_INVERTER_ON) && (inv2_av1_msg[1] & STATUS_INVERTER_ON))
				state = WAIT_QUIT_INVERTER_ON;
			else
				state = SET_ENABLE_AND_INV_ON;
			break;
		case WAIT_QUIT_INVERTER_ON:
			// 8
			if ((inv1_av1_msg[1] & STATUS_QUIT_INVERTER_ON) && (inv2_av1_msg[1] & STATUS_QUIT_INVERTER_ON))
				state = SET_POINTS;
			else
				state = WAIT_INVERTER_ON;
			break;
		case SET_POINTS:
			// 9

			break;
		case INV_ERROR:
			// 10
			state = RESET_ERROR;
			break;
		default:
			break;
		}

		if ((inv1_av1_msg[1] & STATUS_ERROR && state != RESET_ERROR
				&& state != SET_POINTS) ||
				(inv2_av1_msg[1] & STATUS_ERROR && state != RESET_ERROR
						&& state != SET_POINTS)) {
			state = INV_ERROR;
		}

	}
    /* USER CODE END WHILE */
}
    /* USER CODE BEGIN 3 */
void sendCANMessage(uint8_t data[], uint16_t id)
{
	CAN_TxHeaderTypeDef pHeader;
		pHeader.DLC = 8;
		pHeader.RTR = CAN_RTR_DATA;
		pHeader.IDE = CAN_ID_STD;
		pHeader.StdId = id;
		HAL_CAN_AddTxMessage(&hcan3, &pHeader, data, &TxMailbox);

}
void CANInit()
{
	CAN_FilterTypeDef CanFilter;
	CanFilter.FilterIdHigh = 0x0000;
	CanFilter.FilterIdLow = 0;
	CanFilter.FilterMaskIdHigh = 0x0000;
	CanFilter.FilterMaskIdLow = 0;
	CanFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CanFilter.FilterBank = 13;
	CanFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	CanFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	CanFilter.FilterActivation = CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(&hcan3, &CanFilter);
	HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan3);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(state == SET_POINTS) {
#ifdef HARDCODED_CYCLING_SETPOINT_VALUES
			rpm1setpoint = rpmsetpoint[rpmIndex];
#endif

			if (currentsetpoint1 < rpm1setpoint) {
				currentsetpoint1 += 5;
			} else if (currentsetpoint1 > rpm1setpoint) {
				currentsetpoint1 -= 5;
			} else if (currentsetpoint1 == rpm1setpoint) {
				currentsetpoint1 = rpm1setpoint;
#ifdef HARDCODED_CYCLING_SETPOINT_VALUES
				rpmIndex = (rpmIndex + 1) % 10;
#endif
			}

			currentsetpoint1 = 20;

			msg1[2] = currentsetpoint1 & 0xFF;
			msg1[3] = (currentsetpoint1 & 0xFF00) >> 8;

			msg1[4] = 1000 & 0xFF;
			msg1[5] = (1000 & 0xFF00) >> 8;

			msg1[6] = -1000 & 0xFF;
			msg1[7] = (-1000 & 0xFF00) >> 8;

			currentsetpoint2 = 300;

			msg2[2] = currentsetpoint2 & 0xFF;
			msg2[3] = (currentsetpoint2 & 0xFF00) >> 8;

			msg2[4] = 1000 & 0xFF;
			msg2[5] = (1000 & 0xFF00) >> 8;

			msg2[6] = -1000 & 0xFF;
			msg2[7] = (-1000 & 0xFF00) >> 8;
	}
	sendCANMessage(msg1, id1);
	sendCANMessage(msg2, id2);
}
  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 6;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
while (1) {

	}
}
  /* USER CODE END Error_Handler_Debug */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
