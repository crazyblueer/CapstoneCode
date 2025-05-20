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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAV_WRITE_SAMPLE_COUNT 2048

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

SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FATFS FatFs;
 FIL fil;
 FRESULT fres;
int16_t data_i2s[WAV_WRITE_SAMPLE_COUNT];
int16_t data_i2s_mic2[WAV_WRITE_SAMPLE_COUNT];
int16_t data_i2s_mic3[WAV_WRITE_SAMPLE_COUNT];
int16_t data_i2s_mic4[WAV_WRITE_SAMPLE_COUNT];

volatile int16_t sample_i2s;
volatile uint8_t button_flag, start_stop_recording;
static uint8_t wav_file_header[44]={0x52, 0x49, 0x46, 0x46,
0xa4, 0xa9, 0x03, 0x00,
0x57 ,0x41, 0x56, 0x45,
		0x66, 0x6d,	0x74, 0x20,
    0x10, 0x00, 0x00, 0x00,
		0x01, 0x00,
    0x04, 0x00, //4 channels
		0x00, 0x7d, 0x00, 0x00,
		0x00, 0xE4, 0x03, 0x00,
		0x08, 0x00,
    0x10, 0x00,
		0x64, 0x61, 0x74, 0x61,
		0x80, 0xa9, 0x03, 0x00};
static uint32_t wav_file_size;
static uint8_t first_time =0;
volatile uint8_t half_i2s, full_i2s;

volatile uint8_t mic1_half_ready = 0, mic1_full_ready = 0;
volatile uint8_t mic2_half_ready = 0, mic2_full_ready = 0;
volatile uint8_t mic3_half_ready = 0, mic3_full_ready = 0;
volatile uint8_t mic4_half_ready = 0, mic4_full_ready = 0;

volatile int frame_counter = 0;
const int frames_to_skip = 10;  // skip first 5 half/full frames

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S1_Init(void);
static void MX_I2S4_Init(void);
static void MX_SPI5_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...){
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer,len,-1);
}

void start_recording(uint32_t frequency){
	 char file_name[15];
	static uint8_t file_counter = 10;
//	int file_number_digits = file_counter;
	uint32_t byte_rate = frequency*2*4;
	wav_file_header[24] = (uint8_t)frequency;
		wav_file_header[25] = (uint8_t)(frequency >> 8);
		wav_file_header[26] = (uint8_t)(frequency >> 16);
		wav_file_header[27] = (uint8_t)(frequency >> 24);
		wav_file_header[28] = (uint8_t)byte_rate;
		wav_file_header[29] = (uint8_t)(byte_rate >> 8);
		wav_file_header[30] = (uint8_t)(byte_rate >> 16);
		wav_file_header[31] = (uint8_t)(byte_rate >> 24);
		// defining a wave file name
//			file_name[4] = file_number_digits%10 + 48;
//			file_number_digits /= 10;
//			file_name[3] = file_number_digits%10 + 48;
//			file_number_digits /= 10;
//			file_name[2] = file_number_digits%10 + 48;
			snprintf(file_name, sizeof(file_name), "tree_%03d.wav", file_counter++);


			// creating a file
			fres = f_open(&fil ,file_name, FA_WRITE|FA_CREATE_ALWAYS);
			if(fres != FR_OK)
			{
				myprintf("Error creating file (%s): %d\n", file_name, fres);
				return;
			}
			else
			{
				myprintf("Successfully opened file: %s\n", file_name);
			}
			wav_file_size = 0;


}


//void write2wave_file(uint8_t *mic1, uint8_t *mic2, uint8_t *mic3, uint16_t stereo_sample_count_bytes) {
//    uint32_t temp_number;
//    myprintf("Writing...\n");
//
//    if (first_time == 0) {
//        fres = f_write(&fil, (void *)wav_file_header, sizeof(wav_file_header), (UINT *)&temp_number);
//        if (fres != FR_OK) {
//            myprintf("Header write error: %d\n", fres);
//            f_close(&fil);
//            return;
//        }
//        first_time = 1;
//    }
//
//    // stereo_sample_count_bytes = number of bytes per mic (e.g., 2048 = 512 stereo samples = 512 LEFT samples)
//
//    const int num_stereo_frames = stereo_sample_count_bytes / 4;  // each stereo sample = 4 bytes
//    uint8_t interleaved[3 * num_stereo_frames * 2];  // 3 mics * 2 bytes (LEFT only)
//
//    for (int i = 0, j = 0; i < stereo_sample_count_bytes; i += 4) {
//        // Extract LEFT sample from each mic and interleave
//        interleaved[j++] = mic1[i];       // mic1 LEFT LSB
//        interleaved[j++] = mic1[i + 1];   // mic1 LEFT MSB
//
//        interleaved[j++] = mic2[i];       // mic2 LEFT LSB
//        interleaved[j++] = mic2[i + 1];   // mic2 LEFT MSB
//
//        interleaved[j++] = mic3[i];       // mic3 LEFT LSB
//        interleaved[j++] = mic3[i + 1];   // mic3 LEFT MSB
//    }
//
//    // Write interleaved data
//    UINT bytes_written;
//    fres = f_write(&fil, interleaved, sizeof(interleaved), &bytes_written);
//
//    if (fres != FR_OK) {
//        myprintf("Write error: %d\n", fres);
//        f_close(&fil);
//        return;
//    }
//
//    wav_file_size += bytes_written;
//
//    static int write_count = 0;
//    if (++write_count % 10 == 0) {
//        f_sync(&fil);
//    }
//}




void write2wave_file(uint8_t *mic1, uint8_t *mic2, uint8_t *mic3, uint8_t *mic4, uint16_t stereo_sample_count_bytes) {
    uint32_t temp_number;
    myprintf("Writing...\n");

    if (first_time == 0) {
        fres = f_write(&fil, (void *)wav_file_header, sizeof(wav_file_header), (UINT *)&temp_number);
        if (fres != FR_OK) {
            myprintf("Header write error: %d\n", fres);
            f_close(&fil);
            return;
        }
        first_time = 1;
    }

    float gain = 3.0f;  // Adjust this value as needed

    const int num_stereo_frames = stereo_sample_count_bytes / 4;  // each stereo frame = 4 bytes (L+R)
    uint8_t interleaved[4 * num_stereo_frames * 2];  // 3 mics * 2 bytes per (amplified) LEFT sample

    for (int i = 0, j = 0; i < stereo_sample_count_bytes; i += 4) {
        uint8_t *mics[] = {mic1, mic2, mic3, mic4};
        for (int m = 0; m < 4; m++) {
            // Reconstruct 16-bit sample (little endian)
            int16_t sample = (int16_t)(mics[m][i] | (mics[m][i + 1] << 8));

            // Amplify
            int32_t amplified = (int32_t)(sample * gain);

            // Saturate to int16_t
            if (amplified > 32767) amplified = 32767;
            if (amplified < -32768) amplified = -32768;

            // Store back as little endian
            interleaved[j++] = amplified & 0xFF;
            interleaved[j++] = (amplified >> 8) & 0xFF;
        }
    }

    // Write interleaved data
    UINT bytes_written;
    fres = f_write(&fil, interleaved, sizeof(interleaved), &bytes_written);

    if (fres != FR_OK) {
        myprintf("Write error: %d\n", fres);
        f_close(&fil);
        return;
    }

    wav_file_size += bytes_written;

    static int write_count = 0;
    if (++write_count % 10 == 0) {
        f_sync(&fil);
    }
}



void stop_recording(){

	uint16_t temp_number;
		// updating data size sector
		wav_file_size -= 8;
		wav_file_header[4] = (uint8_t)wav_file_size;
		wav_file_header[5] = (uint8_t)(wav_file_size >> 8);
		wav_file_header[6] = (uint8_t)(wav_file_size >> 16);
		wav_file_header[7] = (uint8_t)(wav_file_size >> 24);
		wav_file_size -= 36;
		wav_file_header[40] = (uint8_t)wav_file_size;
		wav_file_header[41] = (uint8_t)(wav_file_size >> 8);
		wav_file_header[42] = (uint8_t)(wav_file_size >> 16);
		wav_file_header[43] = (uint8_t)(wav_file_size >> 24);

		// moving to the beginning of the file to update the file format
		f_lseek(&fil, 0);
		f_write(&fil,(void *)wav_file_header, sizeof(wav_file_header),(UINT*)&temp_number);
		if(fres != 0)
		{
			myprintf("error in updating the first sector: %d \n", fres);
			while(1);
		}
		f_close(&fil);
		first_time = 0;
		myprintf("closed the file \n");

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
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_I2S1_Init();
  MX_I2S4_Init();
  MX_SPI5_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */
  myprintf("\r\n~ SD card demo ~\r\n\r\n");
  HAL_Delay(1000);



  fres = f_mount(&FatFs, "",1);
  if (fres != FR_OK){
	  myprintf("f_mount error (%i)\r\n", fres);
	  while(1);
  }

  DWORD free_clusters, free_sectors, total_sectors;
  FATFS* getFreeFs;
  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK){
	  myprintf("f_getfree error (%i)\r\n",fres);
	  while(1);
  }

  total_sectors =(getFreeFs->n_fatent-2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;
  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors/2, free_sectors/2);

  //open text file
//  fres = f_open(&fil, "text.txt", FA_READ);
//  if (fres != FR_OK){
//	  myprintf("f_open error (%i)\r\n");
//	  while(1);
//  }
//  myprintf("I was able to open 'text.txt' for reading!\r\n");
//
//  BYTE readBuf[30];
//
//  TCHAR* rres = f_gets((TCHAR*)readBuf,30, &fil);
//  if (rres != 0){
//	  myprintf("read string from 'text.txt' contents:%s\r\n", readBuf);
//
//  }
//  else{
//	  myprintf("f_gets error (%i)\r\n",fres);
//  }

//  f_close(&fil);


  //write a file "write.txt"
//  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
//  if(fres == FR_OK){
//	  myprintf("I was able to open 'write.tx' for writing\r\n");
//  }
//  else{
//	  myprintf("f_open error (%i)\r\n", fres);
//  }
//
//  //copy in a string
//  strncpy((char*)readBuf, "a new file is made!",19);
//  UINT bytesWrote;
//  fres = f_write(&fil, readBuf, 19, &bytesWrote);
//  if(fres==FR_OK){
//	  myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
//  }
//  else{
//	  myprintf("f_write error (%i)\r\n");
//  }
//  f_close(&fil);
//  f_mount(NULL,"",0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 if(button_flag){
		 if(start_stop_recording){
			 HAL_I2S_DMAStop(&hi2s1);
			 HAL_I2S_DMAStop(&hi2s2);
			 HAL_I2S_DMAStop(&hi2s3);
			 HAL_I2S_DMAStop(&hi2s4);
			 start_stop_recording = 0;
			 stop_recording();
			 myprintf("stop recording \n");
		 }
		 else{
			 start_stop_recording = 1;
			 start_recording(I2S_AUDIOFREQ_32K);
			 myprintf("start recording \n");
			 HAL_Delay(500);
			  HAL_I2S_Receive_DMA(&hi2s1, (uint16_t *)data_i2s, sizeof(data_i2s)/2);
			  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)data_i2s_mic2, sizeof(data_i2s_mic2)/2);
			  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)data_i2s_mic3, sizeof(data_i2s_mic3)/2);
			  HAL_I2S_Receive_DMA(&hi2s4, (uint16_t *)data_i2s_mic4, sizeof(data_i2s_mic4)/2);

		 }
		 button_flag= 0;

	 }
	 if (start_stop_recording == 1 && (half_i2s == 1 || full_i2s == 1)) {
//	     if (frame_counter < frames_to_skip) {
//	         // Skip initial noisy frames
//	         frame_counter++;
//	         half_i2s = full_i2s = 0;
//	         continue;
//	     }

	     if (half_i2s == 1) {
	         write2wave_file(
	             (uint8_t*)data_i2s,
	             (uint8_t*)data_i2s_mic2,
	             (uint8_t*)data_i2s_mic3,
	             (uint8_t*)data_i2s_mic4,
	             WAV_WRITE_SAMPLE_COUNT
	         );
	         half_i2s = 0;
	     } else if (full_i2s == 1) {
	         write2wave_file(
	             ((uint8_t*)data_i2s) + WAV_WRITE_SAMPLE_COUNT,
	             ((uint8_t*)data_i2s_mic2) + WAV_WRITE_SAMPLE_COUNT,
	             ((uint8_t*)data_i2s_mic3) + WAV_WRITE_SAMPLE_COUNT,
	             ((uint8_t*)data_i2s_mic4) + WAV_WRITE_SAMPLE_COUNT,
	             WAV_WRITE_SAMPLE_COUNT
	         );
	         full_i2s = 0;
	     }
	 }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 14;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
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
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len){
	int DataIdx;
	for (DataIdx =0; DataIdx < len; DataIdx++){
		ITM_SendChar(*ptr++);
	}
	return len;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI1) {
        mic1_full_ready = 1;
    } else if (hi2s->Instance == SPI2) {
        mic2_full_ready = 1;
    } else if (hi2s->Instance == SPI3) {
        mic3_full_ready = 1;
    }
    else if (hi2s->Instance == SPI4) {
           mic4_full_ready = 1;
       }

    if (mic1_full_ready && mic2_full_ready && mic3_full_ready && mic4_full_ready) {
        full_i2s = 1;
        mic1_full_ready = mic2_full_ready = mic3_full_ready  = mic4_full_ready= 0;
    }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI1) {
        mic1_half_ready = 1;
    } else if (hi2s->Instance == SPI2) {
        mic2_half_ready = 1;
    } else if (hi2s->Instance == SPI3) {
        mic3_half_ready = 1;
    }
    else if (hi2s->Instance == SPI4) {
               mic4_half_ready = 1;
           }

    if (mic1_half_ready && mic2_half_ready && mic3_half_ready && mic4_half_ready) {
        half_i2s = 1;
        mic1_half_ready = mic2_half_ready = mic3_half_ready=mic4_half_ready = 0;
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
