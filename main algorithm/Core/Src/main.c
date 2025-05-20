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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "arm_math.h"
#include "stdio.h"
#include "config.h"
#include "vad.h"
#include "beamforming.h"
#include <stdarg.h>
#include "string.h"
#include "gccphat.h"
#include "myprintf.h"
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
I2S_HandleTypeDef hi2s4;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi4_rx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

arm_rfft_fast_instance_f32 fft_audio_instance;

MicFFT mic_fft[NUM_MICS];

volatile int16_t sample_i2s;
volatile uint8_t button_flag, start_stop_recording;
uint8_t vad_ready =0;
uint8_t calibrating = 0;

volatile uint8_t half_mic1=0, full_mic1 = 0;
volatile uint8_t half_mic2 = 0, full_mic2 = 0;
volatile uint8_t half_mic3 = 0, full_mic3 = 0;
volatile uint8_t half_mic4 = 0, full_mic4 = 0;
volatile uint8_t half_all, full_all;


float beamforming_time_us = 0.0f;
float total_loop_time_us = 0.0f;
float cpu_usage_percent = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2S1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//extern void myprintf(const char *fmt, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void myprintf(const char *fmt, ...){
//	static char buffer[256];
//	va_list args;
//	va_start(args, fmt);
//	vsnprintf(buffer, sizeof(buffer),fmt,args);
//	va_end(args);
//
//	int len = strlen(buffer);
//	HAL_UART_Transmit(&huart2, (uint8_t*)buffer,len,-1);
//}

void check_buffer_integrity(int16_t* buffer, const char* label) {
    myprintf("[%s] Checking buffer...\n", label);

    int max_val = INT16_MIN;
    int min_val = INT16_MAX;
    uint32_t zero_count = 0;
    uint32_t sample_count = BUFFER_SIZE;

    for (uint32_t i = 0; i < sample_count; i++) {
        if (buffer[i] > max_val) max_val = buffer[i];
        if (buffer[i] < min_val) min_val = buffer[i];
        if (buffer[i] == 0) zero_count++;
    }

    myprintf("[%s] Min: %d, Max: %d, Zeros: %lu/%lu (%.2f%%)\n",
        label, min_val, max_val, zero_count, sample_count, (zero_count * 100.0f) / sample_count);
}
void calibrate_noise_all_mics() {
    static int first_call = 1;
    if (first_call) {
        vad_reset_noise_estimation();
        first_call = 0;
    }

    for (int mic = 0; mic < NUM_MICS; mic++) {
        float32_t frame[FFT_LENGTH];
        int16_t* samples = mic_fft[mic].raw_buffer;

        for (int i = 0; i < FFT_LENGTH; i++) {
            frame[i] = (float32_t)samples[i * 2];
        }

        prepare_frame(frame, frame);
        vad_update_noise_estimation(frame);
    }
}
void delay (uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

#define stepperrev 4096
/*use like delay */
void stepper_set_rpm(int rpm){ //set rpm -> max 13, min 1,,, went to 14 rev/min
	delay(60000000/stepperrev/rpm);
}
void stepper_half_drive (int step)
{
  switch (step){
         case 0:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

	  case 1:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

          case 2:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

	  case 3:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin,  GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin,GPIO_PIN_RESET);   // IN4
		  break;

	  case 4:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin,   GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

	  case 5:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin, GPIO_PIN_SET);   // IN4
		  break;

	  case 6:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin,GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin, GPIO_PIN_SET);   // IN4
		  break;

	  case 7:
		  HAL_GPIO_WritePin(GPIOB, in1_Pin, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(GPIOB, in2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOC, in3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, in4_Pin,GPIO_PIN_SET);   // IN4
		  break;

	}
}

void stepper_step_angle(float angle, int direction, int rpm){
	float anglesequence = 0.703125; //360 = 512 sequences
	int numberofsequences = (int)(angle/anglesequence);
	for (int seq = 0; seq < numberofsequences;seq++){
		if (direction==0) //for clockwise
		{
			for (int step = 7; step >=0; step--){
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}
		}
		else if(direction ==1) //for anti clockwise
		{
			for (int step =0; step<8; step++){
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}
		}
	}


}
float current_motor_angle = 0.0f;

void motor_face_angle(float target_angle_deg, int rpm) {
    float delta = target_angle_deg - current_motor_angle;

    // Normalize to [-180, 180]
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    int dir = delta > 0 ? 1 : 0;
    float rotate_amount = fabsf(delta);

    stepper_step_angle(rotate_amount, dir, rpm);
    current_motor_angle = target_angle_deg; // Update tracker
}
void motor_return_to_90(int rpm) {
    float target_angle = 10.0f;
    float delta = target_angle - current_motor_angle;

    // Normalize to [-180, 180]
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    int dir = (delta > 0) ? 1 : 0;
    float rotate_amount = fabsf(delta);

    stepper_step_angle(rotate_amount, dir, rpm);
    current_motor_angle = 90.0f; // Update current angle tracker
}

// Initialize motor to 90°
void init_motor() {
    current_motor_angle = 0.0f; // Start from 0°
    motor_return_to_90(10); // Move to 90° initially at 10 RPM
}


//Function to measure calculating time
// Initialize DWT Timer
void DWT_Init(void) {
    // Enable the DWT Counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;    // Enable Trace Control
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;               // Enable DWT Cycle Counter
    DWT->CYCCNT = 0;                                   // Reset Cycle Counter
}


// Function to measure time in microseconds
uint32_t DWT_GetMicroseconds(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000); // Convert to microseconds
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  MX_I2S2_Init();
  MX_I2S4_Init();
  MX_USART6_UART_Init();
  MX_I2S1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  myprintf("hello");

  //DWT_Init();

  HAL_TIM_Base_Start(&htim1);

  mic_fft[0].raw_buffer = data_mic1;
  	mic_fft[1].raw_buffer = data_mic2;
  	mic_fft[2].raw_buffer = data_mic3;
  	mic_fft[3].raw_buffer = data_mic4;

  HAL_I2S_DMAStop(&hi2s1);
  HAL_I2S_DMAStop(&hi2s2);
  HAL_I2S_DMAStop(&hi2s3);
  HAL_I2S_DMAStop(&hi2s4);
  	  HAL_Delay(500);

  	arm_rfft_fast_init_f32(&fft_audio_instance, FFT_LENGTH);
   	preprocessing_init();
//   	gccphat_init();
   	init_motor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (button_flag){
		  if (start_stop_recording){
			  HAL_I2S_DMAStop(&hi2s1);
			  HAL_I2S_DMAStop(&hi2s2);
			  HAL_I2S_DMAStop(&hi2s3);
			  HAL_I2S_DMAStop(&hi2s4);
			  start_stop_recording = 0;
			  half_all = 0;
			  full_all = 0;

			  myprintf("Stop recording\n");
			  calibrating = 0;
		  }
		  else{
			  start_stop_recording =1;
			  myprintf("Start recording\n");
			  HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*)data_mic1, sizeof(data_mic1)/2);
			  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)data_mic2, sizeof(data_mic2)/2);
			  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*)data_mic3, sizeof(data_mic3)/2);
			  HAL_I2S_Receive_DMA(&hi2s4, (uint16_t*)data_mic4, sizeof(data_mic4)/2);
			  calibrating = 1;
			  vad_init();
			  vad_ready =0;
		  }
		  button_flag = 0;

	  }
	  if (calibrating && half_all) {
	  	      calibrate_noise_all_mics();
	  	      if (vad_is_noise_estimation_done()) {
	  	          calibrating = 0;
	  	          vad_ready = 1;
	  	          myprintf("Calibrating done\n");


	  	          // Print background energy right after calibration
	  	          float bg_energy = vad_get_noise_energy();
	  	          myprintf("Background noise energy estimate: %.2f\n", bg_energy);

	  	      }
	  	      half_all = 0;
	  }


	  if (start_stop_recording && half_all && vad_ready) {

	  		  VADFrameInfo infos[NUM_MICS];

	  	      for (int mic = 0; mic < NUM_MICS; mic++) {
	  	          VADFrameInfo info = vad_process_mic_fft(&mic_fft[mic], &fft_audio_instance, 0);
	  	                  infos[mic] = info;
	  	                  //myprintf("[%s] Energy=%.2f ZCR=%.2f Freq=%.1fHz Voice=%d\n",
	  	                           //mic_labels[mic], info.energy, info.zcr, info.dominant_freq, info.is_voice);
	   	      } // for(mic)


		  if (vad_all_mics_agree(infos)) {
			  myprintf("Voice detected\n");
			  //beamforming
			  //DWT->CYCCNT = 0;
			  //uint32_t beamforming_start = DWT->CYCCNT;
			  int angle = 0;
			  float energy = 0.0f;
			  delay_and_sum_beamforming(mic_fft,infos, &angle, &energy);

			  //uint32_t beamforming_end = DWT->CYCCNT;
			  //beamforming_time_us = (float)(beamforming_end - beamforming_start) / (SystemCoreClock / 1000000.0f);
			  //uint32_t total_loop_end = DWT->CYCCNT;
			  //total_loop_time_us = (float)total_loop_end / (SystemCoreClock / 1000000.0f);
			  //cpu_usage_percent = (beamforming_time_us / total_loop_time_us) * 100.0f;

			  //myprintf("|Beamforming Time: %-16.2f |Total Loop Time: %-15.2f |CPU Usage: %-10.2f%% |\n",
			  //         beamforming_time_us, total_loop_time_us, cpu_usage_percent);


			  motor_face_angle((float)angle, 15);

			  myprintf("angle = %d degrees, energy = %.2f\n", angle, energy);


		  } // if (vad_all_mics_agree(infos))

		  	  half_all = 0;
	  } // if (start_stop_recording && half_all && vad_ready)

	  if (start_stop_recording && full_all && vad_ready) {
//  		  check_buffer_integrity(data_mic1, "Mic1");
//
//  		  check_buffer_integrity(data_mic2, "Mic2");
//
//  		  check_buffer_integrity(data_mic3, "Mic3");
//
//	  		  check_buffer_integrity(data_mic4, "Mic4");
	  		  VADFrameInfo infos[NUM_MICS];
	  		  for (int mic = 0; mic < NUM_MICS; mic++) {
	  		  	   VADFrameInfo info = vad_process_mic_fft(&mic_fft[mic], &fft_audio_instance, 0);

	  		  	   infos[mic] = info;

//
	  		  	                //myprintf("[%s] Energy=%.2f ZCR=%.2f Freq=%.1fHz Voice=%d\n",
	  		  	                         //mic_labels[mic], info.energy, info.zcr, info.dominant_freq, info.is_voice);
	  		   	      } // for(mic)
	  		  	  if (vad_all_mics_agree(infos)) {
	  		  	      myprintf("Voice detected\n");
	  		  		  //BEAMFORMING:
	  		  	  int angle = 0;
	  		  	  float energy = 0.0f;

	  		  	//uint32_t start_time = DWT_GetMicroseconds();

	  		  	  delay_and_sum_beamforming(mic_fft,infos, &angle, &energy);

	  		  	//uint32_t end_time = DWT_GetMicroseconds();
	  		  	//uint32_t elapsed_time = end_time - start_time;

	  		  	//myprintf("Beamforming Time: %lu us, Angle = %d°, Energy = %.2f\n", elapsed_time, angle, energy);
	  		  	  motor_face_angle((float)angle, 15);
//
	  		  	  myprintf("angle = %d degrees, energy = %.2f\n", angle, energy);

	  		  	  } // if (vad_all_mics_agree(infos))

	  		  	      full_all = 0;
	  	  } // if (start_stop_recording && full_all && vad_ready)



  } // while (1)
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
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 336;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 7;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief I2S4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S4_Init(void)
{

  /* USER CODE BEGIN I2S4_Init 0 */

  /* USER CODE END I2S4_Init 0 */

  /* USER CODE BEGIN I2S4_Init 1 */

  /* USER CODE END I2S4_Init 1 */
  hi2s4.Instance = SPI4;
  hi2s4.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s4.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s4.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s4.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s4.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s4.Init.CPOL = I2S_CPOL_LOW;
  hi2s4.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s4.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S4_Init 2 */

  /* USER CODE END I2S4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, in1_Pin|in2_Pin|in4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : in3_Pin */
  GPIO_InitStruct.Pin = in3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(in3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : in1_Pin in2_Pin in4_Pin */
  GPIO_InitStruct.Pin = in1_Pin|in2_Pin|in4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len){
	int DataIdx;
	for (DataIdx = 0; DataIdx  < len; DataIdx++){
		ITM_SendChar(*ptr++);

	}
	return len;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
    if (hi2s->Instance == SPI1) {         // Mic 1 (I2S3)
    	half_mic1 = 1;
    } else if (hi2s->Instance == SPI2) {  // Mic 2 (I2S2)
    	half_mic2 = 1;
    } else if (hi2s->Instance == SPI3) {  // Mic 3 (I2S4)
    	half_mic3 = 1;
    } else if (hi2s->Instance == SPI4) {  // Mic 4 (I2S5)
    	half_mic4 = 1;
    }

    // When all mics have completed half-buffer
    if (half_mic1 && half_mic2 && half_mic3 && half_mic4) {
    	half_all = 1;
    }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
    if (hi2s->Instance == SPI1) {         // Mic 1 (I2S3)
    	full_mic1 = 1;
    } else if (hi2s->Instance == SPI2) {  // Mic 2 (I2S2)
    	full_mic2 = 1;
    } else if (hi2s->Instance == SPI3) {  // Mic 3 (I2S4)
    	full_mic3 = 1;
    } else if (hi2s->Instance == SPI4) {  // Mic 4 (I2S5)
    	full_mic4 = 1;
    }

    // When all mics have completed full-buffer
    if (full_mic1 && full_mic2 && full_mic3 && full_mic4) {
    	full_all = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == button_Pin){
		button_flag = 1;
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
