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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bldc/bldc.h"
#include "controllers/pid.h"
#include "encoders/incremental_encoder/encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HAL_IMPORTANT(command) \
    if ((command) != HAL_OK) { \
        blink_notify(5);       \
        Error_Handler();       \
    }
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint16_t main_encoder_CPR = 12;
const float MAX_CURRENT = 22;
const float STALL_CURRENT = 13;

DriveInfo drive = {
    .is_on = false,
    // Physical consts
    .gear_ratio = 1,
    .ppairs = 7,
    .torque_const = 0.24,  // Kt (calculated), Nm / A == V / (rad/s)
    .speed_const = 4.14,   // Kv (measured), (rad/s) / V
    .max_current = MAX_CURRENT,
    .stall_current = STALL_CURRENT
};
DriverControl controller = {
    // Control params
    .predict_change = false,
    .detect_stall = true,
    .mode = SIX_STEP_CONTROL,
    .encoder_filtering = 1.0f,
    .speed_filtering = 0.3f,
    .sampling_interval = 0.05f,
    // Stalling detection
    .stall_timeout = 3,
    .stall_tolerance = 0.2,
    // Targets
    .velocity_target = 1.0,
    .current_limit = MAX_CURRENT,
    .user_current_limit = MAX_CURRENT,
    // Regulation
    .speed_mult = 1.0f,
    .I_mult = 0.5f,
    .PWM_mult = 400.0f,
    .max_PWM_per_s = 2000,
    // Timer period
    .T = 0.0001,
    .velocity_regulator =
            {.p_gain = 0.8, .i_gain = 0.3, .d_gain = 0.0, .integral_error_lim = 0.5, .tolerance = 0.02},
    .current_regulator = {
            .p_gain = 1.0,
            .i_gain = 0.0,
            .d_gain = 0.0,
            .integral_error_lim = 0.0,
            .tolerance = 0.001}
};
InverterState inverter = {
        .I_A_offset = 0,
        .I_B_offset = 0,
        .I_C_offset = 0,
};
#define ADC_buf_size 4
uint32_t ADC1_buf[ADC_buf_size] = {0};

IncrementalEncoder encoder_config;
DriveInfo drive;
DriverControl controller;
InverterState inverter;

GPIO_TypeDef* NotifyLED_GPIOx = LED2_GPIO_Port;
uint16_t NotifyLED_PIN = LED2_Pin;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool motor_get_state() {
    return HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
}

void motor_set_speed(float speed) {
    controller.velocity_target = speed;
}

void motor_set_current_lim(float limit) {
    controller.user_current_limit = limit;
}

float motor_get_speed() {
    return drive.shaft_velocity;
}

float motor_get_current_lim() {
    return controller.current_limit;
}

void start_motor() {
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
        HAL_Delay(10);
    }
    quit_stall(&drive, &controller);
    drive.is_on = true;
}

void stop_motor() {
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
    drive.is_on = false;
}
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  make_incr_encoder_reserved(
      &encoder_config,
      main_encoder_CPR,
      false,
      ENC2_5_GPIO_Port,
      ENC2_4_GPIO_Port,
      ENC2_3_GPIO_Port,
      ENC2_5_Pin,
      ENC2_4_Pin,
      ENC2_3_Pin
  );
  calc_encoder_step(&encoder_config);
  drive.pulses_per_pair = (uint16_t)((float)main_encoder_CPR / (float)drive.ppairs);
  stop_motor();

  // ADC clock settings result in ~1% error
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);  // apply factory calibration
  HAL_ADC_Start_DMA(&hadc1, ADC1_buf, ADC_buf_size);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // A-phase
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // B-phase
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // C-phase

  HAL_IMPORTANT(HAL_TIM_Base_Start_IT(&htim3))  // hbeat
  HAL_IMPORTANT(HAL_TIM_Base_Start_IT(&htim7))  // microseconds
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);  // motor control

  for (int i = 0; i < 64; i++) {
      inverter.I_A_offset += (3.3f * (float)ADC1_buf[1] / (16.0f * 4096.0f));
      inverter.I_B_offset += (3.3f * (float)ADC1_buf[2] / (16.0f * 4096.0f));
      inverter.I_C_offset += (3.3f * (float)ADC1_buf[3] / (16.0f * 4096.0f));
      HAL_Delay(1);
  }
  inverter.I_A_offset /= 64.0f;
  inverter.I_B_offset /= 64.0f;
  inverter.I_C_offset /= 64.0f;

  start_motor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENC2_3_Pin || GPIO_Pin == ENC2_4_Pin || GPIO_Pin == ENC2_5_Pin) {
        handle_encoder_channel(&encoder_config, GPIO_Pin);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM1)
        return;

    GEncoder* generic_encoder = (GEncoder*)(&encoder_config);
    motor_control(
        &controller,
        &drive,
        &inverter,
        ADC1_buf,
        generic_encoder,
        generic_encoder,
        &htim1
    );
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
