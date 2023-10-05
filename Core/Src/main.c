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
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "i2s.h"
#include "pdm2pcm.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "MY_CS43L22.h"
#include "ili9341.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Possible screens */
typedef enum {
	MENU_SCREEN,
	FFT_SCREEN,
	SPECTROGRAM_SCREEN,
	NUMBER_OF_SCREENS
} screen;

/* Possible options in menu screen */
typedef enum {
	VOLUME_MENU_OPTION,
	SAVE_FILE_MENU_OPTION,
	FILE_NUMBER_MENU_OPTION,
	FREQ_RANGE_MENU_OPTION,
	MAG_RANGE_MENU_OPTION,
	FFT_MENU_OPTION,
	SPECTROGRAM_MENU_OPTION,
	NUMBER_OF_MENU_OPTIONS
} menu_option;

/* Structure of wave file */
typedef struct {
	uint32_t chunk_ID;       	/* Byte 0 */
	uint32_t file_size;      	/* Byte 4 */
	uint32_t file_format;    	/* Byte 8 */
	uint32_t sub_chunk1_ID;   	/* Byte 12 */
	uint32_t sub_chunk1_size;	/* Byte 16 */
	uint16_t audio_format;   	/* Byte 20 */
	uint16_t nbr_channels;   	/* Byte 22 */
	uint32_t sample_rate;    	/* Byte 24 */
	uint32_t byte_rate;      	/* Byte 28 */
	uint16_t block_align;    	/* Byte 32 */
	uint16_t bit_per_sample;  	/* Byte 34 */
	uint32_t sub_chunk2_ID;   	/* Byte 36 */
	uint32_t sub_Chunk2_size; 	/* Byte 40 */
} WAV;

/* Union containing values of options */
typedef union {
	struct values_struct {
		uint8_t current_volume;
		uint8_t save_file;
		uint8_t current_file_number;
		uint8_t current_freq_range;
		uint8_t current_mag_range;
	} values_struct;
	uint8_t values_array[5];
} option_values;

/* Union containing maximal values of options */
typedef union max_option_values {
	struct max_values_struct {
		uint8_t max_current_volume;
		uint8_t max_save_file;
		uint8_t	max_current_file_number;
		uint8_t max_current_freq_range;
		uint8_t max_current_mag_range;
	} max_values_struct;
	uint8_t max_values_array[5];
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_FS 				16000 						/* Sampling frequency */
#define RX_BUFFER_SIZE 			128 						/* Length of audio receive buffer */
#define TX_BUFFER_SIZE 			2048						/* Length of audio transmit buffer */
#define MONO_MID_BUFFER_SIZE 	16 							/* Length of mono audio buffer after decimation */
#define STEREO_MID_BUFFER_SIZE	MONO_MID_BUFFER_SIZE * 2	/* Length of stereo audio buffer after decimation */
#define FFT_BUFFER_SIZE 		TX_BUFFER_SIZE / 2 			/* Length of FFT buffer */
#define FFT_MAG_BUFFER_SIZE 	FFT_BUFFER_SIZE / 2 		/* Length of buffer containing magnitude of FFT*/
#define REC_FILE_NAME 			"rec"						/* Name of saved wave file */
#define REC_FILE_EXT			".wav"						/* Extension of wave file */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Bitmap of arrow image */
const uint16_t arrow_img[7][9] = {
		{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000},
		{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000},
		{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0x0000},
		{0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
		{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0x0000},
		{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000},
		{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000}
};

/* Possible options values */
const uint16_t freq_ranges[] = {500, 1000, 2000, 4000, 8000};
const uint8_t mag_ranges[] = {60, 80, 120, 240};

/* Possible arrow positions */
const uint8_t arrow_positions[] = {20, 50, 80, 110, 140, 190, 220};

/* Current option values */
option_values current_option_values = {
		80,
		FALSE,
		0,
		4,
		3
};

/* Maximal possible option values */
const uint8_t max_option_values[] = {
		100,
		1,
		99,
		sizeof(freq_ranges) / sizeof(freq_ranges[0]) - 1,
		sizeof(mag_ranges) / sizeof(mag_ranges[0]) - 1
};

screen current_screen = MENU_SCREEN; 				/* Current screen */
menu_option current_option = VOLUME_MENU_OPTION;	/* Current option */

bool display_menu_flag = TRUE;						/* Flag indicating if screen should return to menu */
bool reset_counter_flag = FALSE;					/* Flag indicating if encoder's counter variable should be reseted */
bool flag_audio_data_ready;							/* Flag indicating if data is ready */
bool flag_audio_file_save;							/* Flag indicating if file should be saved */

int16_t rx_buff[RX_BUFFER_SIZE]; 					/* Receive buffer */
int16_t tx_buff[TX_BUFFER_SIZE]; 					/* Transmit buffer */
int16_t stereo_mid_buff[STEREO_MID_BUFFER_SIZE];	/* Stereo buffer after decimation */
int16_t  fft_prev_db_buff[ILI9341_WIDTH];			/* Previously calculated FFT buffer */
uint16_t callback_counter;							/* Counter of callback */
uint32_t audio_buff_offset;							/* Offset of audio buffer */

WAV		wave_format; 								/* Structure containing bytes of recorded WAV file */
FIL    	wave_file; 									/* Structure containing file object */
FATFS  	usb_fatfs; 									/* Structure containing file system object */

uint8_t  file_num = 0;								/* Number of file currently saved */
uint8_t  wave_header[44];							/* Header of wave file */
uint32_t bytes_recorded;							/* Number of bytes written at one time */
uint32_t total_bytes;								/* Total bytes written */
char     usbh_path[4];								/* Logical drive number to be mounted/unmounted */
char 	 file_name[10];								/* Wave file name */

arm_rfft_fast_instance_f32 fft_audio_instance;		/* Instance FFT structure */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

void pdm_to_pcm(int16_t *rx_buff, int16_t *stereo_mid_buff);
void display_menu(uint8_t current_option_values[]);
void change_option(menu_option *current_option, bool reset_counter_flag);
void change_value_of_option(menu_option current_option, uint8_t *current_option_values, const uint8_t max_option_values[], bool reset_counter_flag);
void display_option(menu_option current_option, int8_t pos);
void draw_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void create_file_name(uint8_t file_number);
void start_recording(void);
void end_recording(void);
void fft_start(void);
void fft_compute(int16_t *tx_buff, float32_t *fft_mag_buff);
void fft_display(void);
void spectrogram_start(void);
void spectrogram_display(uint8_t line_counter);
void display_freq_range(uint16_t freq_range);
void display_mag_range(uint16_t mag_range);
void calc_scale(uint8_t freq_range, uint16_t *i, uint16_t *j);
void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint16_t *R, uint16_t *G, uint16_t *B);
uint32_t wave_process_header_init(uint8_t *p_header, WAV *p_wave_format);
uint32_t wave_process_enc_init(uint32_t freq, uint8_t *p_header);
uint32_t wave_process_header_update(uint8_t *p_header, uint32_t total_bytes);

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_I2S2_Init();
  MX_CRC_Init();
  MX_FATFS_Init();
  MX_I2S3_Init();
  MX_TIM1_Init();
  MX_PDM2PCM_Init();
  /* USER CODE BEGIN 2 */

  /* DAC initialize */
  CS43_Init(hi2c1, MODE_I2S);
  CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
  CS43_Stop();

  /* Display driver initialize */
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);

  /* Encoder's timer initialize */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  /* DMA initialize */
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)rx_buff, RX_BUFFER_SIZE);
  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)tx_buff, TX_BUFFER_SIZE);

  /* USB power supply */
  MX_DriverVbusFS(0);

  /* FFT initialize */
  arm_rfft_fast_init_f32(&fft_audio_instance, FFT_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Load menu screen only at the start and when FFT or spectrogram is closed */
	  if (current_screen == MENU_SCREEN && display_menu_flag) {
		  display_menu(current_option_values.values_array);
		  display_menu_flag = FALSE;
	  }

	  /* If encoder is rotated move arrow and change current option */
	  change_option(&current_option, reset_counter_flag);

	  /* Counter of "change_option" should be reseted only when program
	     returns from changing options values or executing tasks */
	  reset_counter_flag = FALSE;


	  /* If encoder button is pressed */
	  if (!HAL_GPIO_ReadPin(ENC_Input_GPIO_Port, ENC_Input_Pin)) {
		  /* Wait until encoder's button is released */
		  while (!HAL_GPIO_ReadPin(ENC_Input_GPIO_Port, ENC_Input_Pin))
			  ;

		  /* If user wants to change option values */
		  if (current_option < FFT_MENU_OPTION) {
			  draw_frame(120, arrow_positions[current_option] - 8, 190 , arrow_positions[current_option] + 13, ILI9341_WHITE);
		  }

		  /* Encoder's counter value should be reseted */
		  reset_counter_flag = TRUE;

		  /* While encoder's button is not pressed again and menu flag is not set */
		  while (!display_menu_flag && HAL_GPIO_ReadPin(ENC_Input_GPIO_Port, ENC_Input_Pin)) {
			  /* If user wants to change option values */
			  if (current_option < FFT_MENU_OPTION) {
				  change_value_of_option(current_option, current_option_values.values_array, max_option_values, reset_counter_flag);
				  reset_counter_flag = FALSE;
			  } else if (current_option == FFT_MENU_OPTION) {
				  /* If user wants to execute FFT */
				  fft_start();
				  display_menu_flag = TRUE;
			  } else if (current_option == SPECTROGRAM_MENU_OPTION) {
				  /* If user wants to execute spectrogram */
		  		  spectrogram_start();
				  display_menu_flag = TRUE;
			  }
		  }

		  if (current_option < FFT_MENU_OPTION) {
			  /* Draw black border to remove white one */
			  draw_frame(120, arrow_positions[current_option] - 8, 190 , arrow_positions[current_option] + 13, ILI9341_BLACK);
		  } else {
			  /* Change values to return to menu screen */
			  current_option = VOLUME_MENU_OPTION;
			  current_screen = MENU_SCREEN;
		  }

		  /* Counter of "change_option" should be reseted when returning to menu screen */
		  reset_counter_flag = TRUE;

		  /* Wait until encoder's button is released */
		  while (!HAL_GPIO_ReadPin(ENC_Input_GPIO_Port, ENC_Input_Pin))
			  ;
	  }
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Manage the DMA half transfer complete interrupt from receiving channel
  * @param  hi2s: I2S handler
  * @retval None
  */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	/* Only when the interrupt is triggered by I2S connected to the microphone */
	if (hi2s->Instance == SPI2) {
		/* Convert PDM sound to PCM sound */
		pdm_to_pcm(rx_buff, stereo_mid_buff);

		/* Copy converted data to transmit buffer */
		memcpy(&tx_buff[callback_counter * (STEREO_MID_BUFFER_SIZE)], stereo_mid_buff, STEREO_MID_BUFFER_SIZE * 2);

		/* If there is enough data set flag and offset */
		if (callback_counter == (TX_BUFFER_SIZE / (STEREO_MID_BUFFER_SIZE * 2)) - 1) {
			flag_audio_data_ready = TRUE;
			audio_buff_offset = 0;
			callback_counter++;
		} else if (callback_counter == (TX_BUFFER_SIZE / (STEREO_MID_BUFFER_SIZE)) - 1) {
			flag_audio_data_ready = TRUE;
			audio_buff_offset = TX_BUFFER_SIZE / 2;
			callback_counter = 0;
		} else {
			callback_counter++;
		}
	}
}

/**
  * @brief  Manage the DMA full transfer complete interrupt from receiving channel
  * @param  hi2s: I2S handler
  * @retval None
  */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	/* Only when the interrupt is triggered by I2S connected to the microphone */
	if (hi2s->Instance == SPI2) {
		/* Convert PDM sound to PCM sound */
		pdm_to_pcm(&rx_buff[RX_BUFFER_SIZE / 2], stereo_mid_buff);

		/* Copy converted data to transmit buffer */
		memcpy(&tx_buff[callback_counter * (STEREO_MID_BUFFER_SIZE)], stereo_mid_buff, STEREO_MID_BUFFER_SIZE * 2);

		/* If there is enough data set flag and offset */
		if (callback_counter == (TX_BUFFER_SIZE / (STEREO_MID_BUFFER_SIZE * 2)) - 1) {
			flag_audio_data_ready = TRUE;
			audio_buff_offset = 0;
			callback_counter++;
		} else if (callback_counter == (TX_BUFFER_SIZE / (STEREO_MID_BUFFER_SIZE)) - 1) {
			flag_audio_data_ready = TRUE;
			audio_buff_offset = TX_BUFFER_SIZE / 2;
			callback_counter = 0;
		} else {
			callback_counter++;
		}
	}
}

/**
  * @brief       Converts audio format from PDM to PCM
  * @param[in]   rx_buff: Received PDM data buffer
  * @param[out]  stereo_mid_buff: Converted PCM data buffer
  * @retval      None
  */
void pdm_to_pcm(int16_t* rx_buff, int16_t* stereo_mid_buff) {
	/* One channel (mono) buffer */
	uint16_t mono_mid_buff[MONO_MID_BUFFER_SIZE];

	/* PDM to PCM filter */
	PDM_Filter((int8_t*)&rx_buff[0], (int16_t*)&mono_mid_buff[0], &PDM1_filter_handler);

	/* Duplicate samples to create 2 channel (stereo) sound */
	for (uint8_t i = 0; i < MONO_MID_BUFFER_SIZE; i++) {
		stereo_mid_buff[i * 2] = mono_mid_buff[i];
		stereo_mid_buff[i * 2 + 1] = mono_mid_buff[i];
	}
}

/**
  * @brief  Display current values of options
  * @param  current_option_values: Array from which the values are displayed
  * @retval None
  */
void display_menu(uint8_t current_option_values[]) {
	/* Buffers to store string option values */
	char s_volume[3];
	char s_freq_range[4];
	char s_mag_range[3];
	char s_file_number[4];

	/* Display volume value */
	itoa((int)current_option_values[VOLUME_MENU_OPTION], s_volume, 10);
	ILI9341_WriteString(30, arrow_positions[VOLUME_MENU_OPTION], "VOLUME", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(130, arrow_positions[VOLUME_MENU_OPTION], s_volume, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(145, arrow_positions[VOLUME_MENU_OPTION], " %", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

	/* Display whether file should be saved */
	ILI9341_WriteString(30, arrow_positions[SAVE_FILE_MENU_OPTION], "SAVE FILE", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	if (current_option_values[SAVE_FILE_MENU_OPTION]) {
		ILI9341_WriteString(130, arrow_positions[SAVE_FILE_MENU_OPTION], "YES", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
	} else {
		ILI9341_WriteString(130, arrow_positions[SAVE_FILE_MENU_OPTION], "NO", Font_7x10, ILI9341_RED, ILI9341_BLACK);
	}

	/* Display current file number */
	sprintf(s_file_number, "%02d", current_option_values[FILE_NUMBER_MENU_OPTION]);
	ILI9341_WriteString(30, arrow_positions[FILE_NUMBER_MENU_OPTION], "FILE NUMBER", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(130, arrow_positions[FILE_NUMBER_MENU_OPTION], s_file_number, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

	/* Display current frequency range */
	itoa(freq_ranges[current_option_values[FREQ_RANGE_MENU_OPTION]], s_freq_range, 10);
	ILI9341_WriteString(30, arrow_positions[FREQ_RANGE_MENU_OPTION], "FREQ RANGE", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(130, arrow_positions[FREQ_RANGE_MENU_OPTION], s_freq_range, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(160, arrow_positions[FREQ_RANGE_MENU_OPTION], " Hz", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

	/* Display current magnitude range */
	itoa(mag_ranges[current_option_values[MAG_RANGE_MENU_OPTION]], s_mag_range, 10);
	ILI9341_WriteString(30, arrow_positions[MAG_RANGE_MENU_OPTION], "MAG RANGE", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(130, arrow_positions[MAG_RANGE_MENU_OPTION], s_mag_range, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(152, arrow_positions[MAG_RANGE_MENU_OPTION], " dB", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

	/* Display FFT and spectrogram words */
	ILI9341_WriteString(30, arrow_positions[FFT_MENU_OPTION], "FFT", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(30, arrow_positions[SPECTROGRAM_MENU_OPTION], "SPECTROGRAM", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

	/* Display arrow */
	ILI9341_DrawImage(10, arrow_positions[VOLUME_MENU_OPTION], 9, 7, arrow_img);
}

/**
  * @brief  Change current option and display arrow pointing to it
  * @param  current_option: Pointer to the option that will be changed
  * @param  reset_counter_flag: Flag indicating whether the counter should be reseted
  * @retval None
  */
void change_option(menu_option *current_option, bool reset_counter_flag) {
	/* Value of last enoder's timer tick */
	static uint16_t last_timer_counter = 0;

	/* "change_option" and "change_value_of_option" use the same encoder's timer, so
	   after one function is called the other one's counter should be set to timer's value */
	if (reset_counter_flag) {
		last_timer_counter = htim1.Instance->CNT;
	}

	/* Difference between current timer's value and previous one */
	int16_t timer_dif = htim1.Instance->CNT - last_timer_counter;

	/* One encoder impulse is equal 4 ticks of timer */
	/* If encoder is moved arrow position is changed */
	if (timer_dif >= 4 || timer_dif <= -4) {
		ILI9341_FillRectangle(10, arrow_positions[*current_option], 9, 7, ILI9341_BLACK);
		timer_dif /= 4;
		int8_t pos = *current_option - (int8_t)timer_dif;

		if (pos >= NUMBER_OF_MENU_OPTIONS) {
			pos = VOLUME_MENU_OPTION;
		} else if (pos < VOLUME_MENU_OPTION) {
			pos = SPECTROGRAM_MENU_OPTION;
		}

		*current_option = pos;

		ILI9341_DrawImage(10, arrow_positions[pos], 9, 7, arrow_img);
		last_timer_counter = htim1.Instance->CNT;
	}
}

/**
  * @brief  Change current value of option
  * @param  current_option: Current chosen option
  * @param  current_option_values: Pointer to array of current option values
  * @param  max_option_values: Array of maximal values of options
  * @param  reset_counter_flag: Flag indicating if encoder's counter variable should be reseted
  * @retval None
  */
void change_value_of_option(menu_option current_option, uint8_t *current_option_values, const uint8_t max_option_values[], bool reset_counter_flag) {
	/* Value of last enoder's timer tick */
	static uint16_t last_timer_counter = 0;

	/* "change_option" and "change_value_of_option" use the same encoder's timer, so
	   after one function is called the other one's counter should be set to timer's value */
	if (reset_counter_flag) {
		last_timer_counter = htim1.Instance->CNT;
	}

	/* Difference between current timer's value and previous one */
	int16_t timer_dif = htim1.Instance->CNT - last_timer_counter;

	/* One encoder impulse is equal 4 ticks of timer */
	/* If encoder is moved option value is changed */
	if (timer_dif >= 4 || timer_dif <= -4) {
		timer_dif /= 4;
		int8_t pos = current_option_values[current_option] - (int8_t)timer_dif;

		if (pos > max_option_values[current_option]) {
			pos = 0;
		} else if (pos < 0) {
			pos = max_option_values[current_option];
		}

		current_option_values[current_option] = pos;

		last_timer_counter = htim1.Instance->CNT;

		display_option(current_option, pos);
	}
}

/**
  * @brief  Display current value of option
  * @param  current_option: Current chosen option
  * @param  pos: Position (value) of option
  * @retval None
  */
void display_option(menu_option current_option, int8_t pos) {
	/* Depending on current option and position new value of option is displayed */
	switch (current_option) {
		case VOLUME_MENU_OPTION:
			char s_volume[3];
			itoa(pos, s_volume, 10);
			ILI9341_FillRectangle(130, arrow_positions[VOLUME_MENU_OPTION], 21, 10, ILI9341_BLACK);
			ILI9341_WriteString(130, arrow_positions[VOLUME_MENU_OPTION], s_volume, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
			break;
		case SAVE_FILE_MENU_OPTION:
			ILI9341_FillRectangle(130, arrow_positions[SAVE_FILE_MENU_OPTION], 21, 10, ILI9341_BLACK);
			if (pos == TRUE) {
				ILI9341_WriteString(130, arrow_positions[SAVE_FILE_MENU_OPTION], "YES", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
			} else if (pos == FALSE) {
				ILI9341_WriteString(130, arrow_positions[SAVE_FILE_MENU_OPTION], "NO", Font_7x10, ILI9341_RED, ILI9341_BLACK);
			} else {
				ILI9341_WriteString(130, arrow_positions[SAVE_FILE_MENU_OPTION], "ERROR", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
			}
			break;
		case FILE_NUMBER_MENU_OPTION:
			char s_file_number[4];
			sprintf(s_file_number, "%02d", pos);
			ILI9341_FillRectangle(130, arrow_positions[FILE_NUMBER_MENU_OPTION], 21, 10, ILI9341_BLACK);
			ILI9341_WriteString(130, arrow_positions[FILE_NUMBER_MENU_OPTION], s_file_number, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
			break;
		case FREQ_RANGE_MENU_OPTION:
			char s_freq_range[4];
			itoa(freq_ranges[pos], s_freq_range, 10);
			ILI9341_FillRectangle(130, arrow_positions[FREQ_RANGE_MENU_OPTION], 28, 10, ILI9341_BLACK);
			ILI9341_WriteString(130, arrow_positions[FREQ_RANGE_MENU_OPTION], s_freq_range, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
			break;
		case MAG_RANGE_MENU_OPTION:
			char s_mag_range[3];
			itoa(mag_ranges[pos], s_mag_range, 10);
			ILI9341_FillRectangle(130, arrow_positions[MAG_RANGE_MENU_OPTION], 21, 10, ILI9341_BLACK);
			ILI9341_WriteString(130, arrow_positions[MAG_RANGE_MENU_OPTION], s_mag_range, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
			break;
		default:
			break;
	}
}

/**
  * @brief  Display frame
  * @param  x: x coordinate
  * @param  y: y coordinate
  * @param  w: width of frame
  * @param  h: height of frame
  * @param  color: color of frame
  * @retval None
  */
void draw_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	/* Draw frame */
	/* Used when the option is selected */
	for (uint16_t i = x; i <= w; i++) {
		for (uint16_t j = y; j <= h; j++) {
			if ((i == x || i == w) || (j == y || j == h)) {
				ILI9341_DrawPixel(i, j, color);
			}
		}
	}
}

/**
  * @brief  Creates file name
  * @retval None
  */
void create_file_name(uint8_t file_number) {
	sprintf(file_name, "%s%02d%s", REC_FILE_NAME, file_number, REC_FILE_EXT);

	/* Increment file number */
	current_option_values.values_struct.current_file_number++;
}

/**
  * @brief  Start recording audio to wave file
  * @retval None
  */
void start_recording(void) {
	create_file_name(current_option_values.values_struct.current_file_number);

	/* Mount USB stick */
	f_mount(&usb_fatfs, (TCHAR const*)usbh_path, 0);

	/* Remove file if it already exists */
	f_unlink(file_name);

	/* Open file */
	f_open(&wave_file, file_name, FA_CREATE_ALWAYS | FA_WRITE);

	/* Create header of wave file */
	wave_process_enc_init(AUDIO_FS, wave_header);
	f_write(&wave_file, wave_header, 44, (void*)&bytes_recorded);
	total_bytes = bytes_recorded;
}

/**
  * @brief  End recording audio to wave file
  * @retval None
  */
void end_recording(void) {
	/* Move read/write pointer to the beginning of file */
	f_lseek(&wave_file, 0);

	/* Update header of wave file */
	wave_process_header_update(wave_header, (uint32_t)&wave_format);
	f_write(&wave_file, wave_header, 44, (void*)&bytes_recorded);

	/* Close file */
	f_close (&wave_file);

	/* Unmount USB stick */
	f_mount(NULL, 0, 1);
}

/**
  * @brief  Start FFT
  * @retval None
  */
void fft_start(void) {
	current_screen = FFT_SCREEN;

	/* Clear screen */
	ILI9341_FillScreen(ILI9341_BLACK);

	/* Display frequency and magnitude range */
	display_freq_range(freq_ranges[current_option_values.values_struct.current_freq_range]);
	display_mag_range(mag_ranges[current_option_values.values_struct.current_mag_range]);

	if (current_option_values.values_struct.save_file) {
		start_recording();
	}

	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD4_Pin);

	/* Start DAC */
	CS43_SetVolume(current_option_values.values_struct.current_volume);
	CS43_Start();

	/* While encoder's button is not pressed again */
	while (HAL_GPIO_ReadPin(ENC_Input_GPIO_Port, ENC_Input_Pin)) {
		/* If there is enough sound data */
		if (flag_audio_data_ready) {
			fft_display();
			if (current_option_values.values_struct.save_file) {
				f_write(&wave_file, (int8_t*)(tx_buff + audio_buff_offset), TX_BUFFER_SIZE, (void*)&bytes_recorded);
				total_bytes += bytes_recorded;
			}
			flag_audio_data_ready = FALSE;
		}
	}

	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD4_Pin);

	if (current_option_values.values_struct.save_file) {
		end_recording();
	}

	/* Stop DAC */
	CS43_Stop();

	/* Clear screen */
	ILI9341_FillScreen(ILI9341_BLACK);
}

/**
  * @brief       Compute FFT on transmit buffer
  * @param[in]   tx_buff: Transmit buffer
  * @param[out]  fft_mag_buff: Output buffer of FFT magnitude
  * @retval      None
  */
void fft_compute(int16_t* tx_buff, float32_t* fft_mag_buff) {
	/* Sound buffer */
	float32_t float_tx_buff[FFT_BUFFER_SIZE];
	/* FFT buffer (real and imaginary part) */
	float32_t fft_out_buff[FFT_BUFFER_SIZE];

	float32_t hann_multiplier = 0;

	/* Convert sound from int32_t to float32_t */
	/* Hann window is applied to reduce spectral leakage */
	for (uint16_t i = 0; i < FFT_BUFFER_SIZE; i++) {
		hann_multiplier = 0.5 * (1 - cosf(2 * PI * i / (FFT_BUFFER_SIZE - 1)));
		float_tx_buff[i] = hann_multiplier * (float32_t)tx_buff[i];
	}

	/* Compute FFT */
	arm_rfft_fast_f32(&fft_audio_instance, float_tx_buff, fft_out_buff, 0);

	/* Calculate magnitude from real and imaginary part of FFT */
	arm_cmplx_mag_f32(fft_out_buff, fft_mag_buff, FFT_MAG_BUFFER_SIZE);
}

/**
  * @brief  Display FFT
  * @retval None
  */
void fft_display(void) {
	/* FFT magnitude buffers */
	float32_t fft_mag_buff[FFT_MAG_BUFFER_SIZE];
	int32_t fft_db_buff[FFT_MAG_BUFFER_SIZE];

	/* Compute FFT */
	fft_compute((int16_t*)(tx_buff + audio_buff_offset), fft_mag_buff);

	/* Loop values */
	uint16_t i = 0, j = 0;

	/* Calculate loop values */
	calc_scale(current_option_values.values_struct.current_freq_range, &i, &j);

	/* Scale displayed FFT depending on frequency and magnitude range */
	for (uint16_t len = 0; len < i; len++) {
		for (uint16_t current_val = 0; current_val < j; current_val++) {
			uint32_t index = len * j + current_val;

			fft_db_buff[index] = (int32_t)(20 * log10f(fft_mag_buff[len]));

			if (fft_prev_db_buff[index] != fft_db_buff[index]) {
				ILI9341_DrawPixel(index, 240 - (fft_prev_db_buff[index] * 240 / mag_ranges[current_option_values.values_struct.current_mag_range]), ILI9341_BLACK);
				ILI9341_DrawPixel(index, 240 - (fft_db_buff[index] * 240 / mag_ranges[current_option_values.values_struct.current_mag_range]), ILI9341_WHITE);
			}
			fft_prev_db_buff[index] = fft_db_buff[index];
		}
	}
}

/**
  * @brief  Start spectrogram
  * @retval None
  */
void spectrogram_start(void) {
	current_screen = SPECTROGRAM_SCREEN;

	/* Clear screen */
	ILI9341_FillScreen(ILI9341_BLACK);

	/* does not work here ???*/
	/* display_freq_range(); */

	char s_freq_range[4];
	itoa(freq_ranges[current_option_values.values_struct.current_freq_range], s_freq_range, 10);
	ILI9341_WriteString(270, arrow_positions[VOLUME_MENU_OPTION], "FREQ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(270, arrow_positions[VOLUME_MENU_OPTION] + 12, "RANGE", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(270, arrow_positions[VOLUME_MENU_OPTION] + 24, s_freq_range, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(298, arrow_positions[VOLUME_MENU_OPTION] + 24, " Hz", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

	if (current_option_values.values_struct.save_file) {
		start_recording();
	}

	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD4_Pin);

	/* Current horizontal line that should be displayed */
	uint8_t line_counter = 239;

	/* Start DAC */
	CS43_SetVolume(current_option_values.values_struct.current_volume);
	CS43_Start();

	while (HAL_GPIO_ReadPin(ENC_Input_GPIO_Port, ENC_Input_Pin)) {
		if (flag_audio_data_ready) {
			spectrogram_display(line_counter);
			line_counter--;

			if (line_counter > 239) {
				line_counter = 239;
			}
			if (current_option_values.values_struct.save_file) {
				f_write(&wave_file, (int8_t*)(tx_buff + audio_buff_offset), TX_BUFFER_SIZE, (void*)&bytes_recorded);
				total_bytes += bytes_recorded;
			}

			flag_audio_data_ready = FALSE;
		}
	}
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD4_Pin);

	if (current_option_values.values_struct.save_file) {
		end_recording();
	}

	/* Stop DAC */
	CS43_Stop();

	/* Clear Screen */
	ILI9341_FillScreen(ILI9341_BLACK);
}

/**
  * @brief  Display spectrogram
  * @param  line_counter: Spectrogram's line that should be displayed
  * @retval None
  */
void spectrogram_display(uint8_t line_counter) {
	/* FFT magnitude buffers */
	float32_t fft_mag_buff[FFT_MAG_BUFFER_SIZE];
	int32_t fft_db_buff[FFT_MAG_BUFFER_SIZE];

	/* Compute FFT */
	fft_compute((int16_t*)(tx_buff + audio_buff_offset), fft_mag_buff);

	/* RGB color values */
	uint16_t r, g, b;

	/* Loop values */
	uint16_t i = 0, j = 0;

	/* Calculate loop values */
	calc_scale(current_option_values.values_struct.current_freq_range, &i, &j);

	/* Scale displayed FFT depending on frequency and magnitude range */
	for (uint16_t len = 0; len < i; len++) {
		for (uint16_t current_val = 0; current_val < j; current_val++) {
			uint32_t index = len * j + current_val;
			fft_db_buff[index] = (int32_t)(20 * log10f(fft_mag_buff[len]));

			/* Draw black line */
			if (line_counter > 0) {
				ILI9341_DrawPixel(index, line_counter - 1, ILI9341_BLACK);
			}

			/* Draw line of spectrogram */
			if (index > 0 && fft_db_buff[index] == fft_db_buff[index - 1]) {
				ILI9341_DrawPixel(index, line_counter, ILI9341_COLOR565(r, g, b));
			} else {
				uint16_t h;
				h = 240 - 2 * (uint16_t)fft_db_buff[index];
				hsv_to_rgb(h, 100, 100, &r, &g, &b);
				ILI9341_DrawPixel(index, line_counter, ILI9341_COLOR565(r, g, b));
			}
		}
	}
}

/**
  * @brief  Display range of frequency
  * @param  freq_range: Range of frequency
  * @retval None
  */
void display_freq_range(uint16_t freq_range) {
	char s_freq_range[4];
	itoa(freq_range, s_freq_range, 10);
	ILI9341_WriteString(270, arrow_positions[0], "FREQ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(270, arrow_positions[0] + 12, "RANGE", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(270, arrow_positions[0] + 24, s_freq_range, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(298, arrow_positions[0] + 24, " Hz", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
}
/**
  * @brief  Display range of magnitude
  * @param  mag_range: Range of magnitude
  * @retval None
  */
void display_mag_range(uint16_t mag_range) {
	char s_mag_range[4];
	itoa(mag_range, s_mag_range, 10);
	ILI9341_WriteString(270, arrow_positions[2], "MAG", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(270, arrow_positions[2] + 12, "RANGE", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(270, arrow_positions[2] + 24, s_mag_range, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
	ILI9341_WriteString(291, arrow_positions[2] + 24, " dB", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
}

/**
  * @brief       Calculate loop values used in scaling FFT and spectrogram
  * @param[in]   freq_range: Current frequency range
  * @param[out]  i: First loop value
  * @param[out]  j: Second loop value
  * @retval      None
  */
void calc_scale(uint8_t freq_range, uint16_t *i, uint16_t *j) {
	/* Depending on frequency range different loop values are used */
	switch (freq_range) {
		case 0:
			*i = FFT_MAG_BUFFER_SIZE / 32;
			*j = 16;
			break;
		case 1:
			*i = FFT_MAG_BUFFER_SIZE / 16;
			*j = 8;
			break;
		case 2:
			*i = FFT_MAG_BUFFER_SIZE / 8;
			*j = 4;
			break;
		case 3:
			*i = FFT_MAG_BUFFER_SIZE / 4;
			*j = 2;
			break;
		case 4:
			*i = FFT_MAG_BUFFER_SIZE / 2;
			*j = 1;
			break;
		default:
			break;
	}
}

/**
  * @brief       Convert colors in HSV to RGB
  * @param[in]   h: Hue of color
  * @param[in]   s: Saturation of color
  * @param[in]   v: Value of color
  * @param[out]  R: Red color value
  * @param[out]  G: Green color value
  * @param[out]  B: Blue color value
  * @retval      None
  */
void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint16_t *R, uint16_t *G, uint16_t *B) {
	uint16_t c, m, x;

	/* Chroma */
    c = s * v / 10;
    /* An intermediate value used for computing the RGB model */
	x = c * (100 - abs(((h * 100 / 60) % 200) - 100)) / 100;
	/* RGB component with the smallest value */
	m = (v * 10) - c;

    switch((h / 60)) {
    	case 0:
            *R = c + m;
            *G = x + m;
            *B = m;
            break;
        case 1:
            *R = x + m;
            *G = c + m;
            *B = m;
            break;
        case 2:
        	*R = m;
            *G = c + m;
            *B = x + m;
            break;
        case 3:
        	*R = m;
            *G = x + m;
            *B = c + m;
            break;
        case 4:
            *R = x + m;
            *G = m;
            *B = c + m;
            break;
        case 5:
            *R = c + m;
            *G = m;
            *B = x + m;
            break;
    }
}

/**
  * @brief  Initialize the wave header file
  * @param  p_header: Header buffer to be filled
  * @param  p_wave_format: Pointer to the wave structure to be filled
  * @retval 0 if passed, !0 if failed
  */
uint32_t wave_process_header_init(uint8_t *p_header, WAV *p_wave_format) {
  /* Write chunkID, must be 'RIFF' */
  p_header[0] = 'R';
  p_header[1] = 'I';
  p_header[2] = 'F';
  p_header[3] = 'F';

  /* Write the file length */
  /* The sampling time: this value will be be written back at the end of the
     recording operation.  Example: 661500 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  p_header[4] = 0x00;
  p_header[5] = 0x4C;
  p_header[6] = 0x1D;
  p_header[7] = 0x00;

  /* Write the file format, must be 'wave' */
  p_header[8]  = 'W';
  p_header[9]  = 'A';
  p_header[10] = 'V';
  p_header[11] = 'E';

  /* Write the format chunk, must be'fmt ' */
  p_header[12]  = 'f';
  p_header[13]  = 'm';
  p_header[14]  = 't';
  p_header[15]  = ' ';

  /* Write the length of the 'fmt' data, must be 0x10 */
  p_header[16]  = 0x10;
  p_header[17]  = 0x00;
  p_header[18]  = 0x00;
  p_header[19]  = 0x00;

  /* Write the audio format, must be 0x01 (PCM) */
  p_header[20]  = 0x01;
  p_header[21]  = 0x00;

  /* Write the number of channels, ie. 0x01 (Mono) */
  p_header[22]  = p_wave_format->nbr_channels;
  p_header[23]  = 0x00;

  /* Write the Sample Rate in Hz */
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00 */
  p_header[24]  = (uint8_t)((p_wave_format->sample_rate & 0xFF));
  p_header[25]  = (uint8_t)((p_wave_format->sample_rate >> 8) & 0xFF);
  p_header[26]  = (uint8_t)((p_wave_format->sample_rate >> 16) & 0xFF);
  p_header[27]  = (uint8_t)((p_wave_format->sample_rate >> 24) & 0xFF);

  /* Write the Byte Rate */
  p_header[28]  = (uint8_t)((p_wave_format->byte_rate & 0xFF));
  p_header[29]  = (uint8_t)((p_wave_format->byte_rate >> 8) & 0xFF);
  p_header[30]  = (uint8_t)((p_wave_format->byte_rate >> 16) & 0xFF);
  p_header[31]  = (uint8_t)((p_wave_format->byte_rate >> 24) & 0xFF);

  /* Write the block alignment */
  p_header[32]  = p_wave_format->block_align;
  p_header[33]  = 0x00;

  /* Write the number of bits per sample */
  p_header[34]  = p_wave_format->bit_per_sample;
  p_header[35]  = 0x00;

  /* Write the Data chunk, must be 'data' */
  p_header[36]  = 'd';
  p_header[37]  = 'a';
  p_header[38]  = 't';
  p_header[39]  = 'a';

  /* Write the number of sample data */
  /* This variable will be written back at the end of the recording operation */
  p_header[40]  = 0x00;
  p_header[41]  = 0x4C;
  p_header[42]  = 0x1D;
  p_header[43]  = 0x00;

  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief       Encoder of wave file initialization
  * @param[in]   freq: Sampling frequency
  * @param[out]  p_header: Pointer to the wave file header to be written
  * @retval      0 if success, !0 else
  */
uint32_t wave_process_enc_init(uint32_t freq, uint8_t* p_header) {
  /* Initialize the encoder structure */
  wave_format.sample_rate = freq;         /* Audio sampling frequency */
  wave_format.nbr_channels = 2;           /* Number of channels: 1:Mono or 2:Stereo */
  wave_format.bit_per_sample = 16;        /* Number of bits per sample (16, 24 or 32) */
  wave_format.file_size = 0x001D4C00;     /* Total length of useful audio data (payload) */
  wave_format.sub_chunk1_size = 44;       /* The file header chunk size */
  wave_format.byte_rate = (wave_format.sample_rate * \
                        (wave_format.bit_per_sample/8) * \
                         wave_format.nbr_channels);      /* Number of bytes per second  (sample rate * block align)  */
  wave_format.block_align = wave_format.nbr_channels * \
                         (wave_format.bit_per_sample/8); /* channels * bits/sample / 8 */

  /* Parse the wav file header and extract required information */
  if(wave_process_header_init(p_header, &wave_format))
  {
    return 1;
  }
  return 0;
}

/**
  * @brief       Initialize the wave header file
  * @param[out]  p_header: Header Buffer to be filled
  * @param[in]   total_bytes: Total bytes of wave file
  * @retval      0 if passed
  */
uint32_t wave_process_header_update(uint8_t* p_header, uint32_t total_bytes) {
  /* Write the file length */
  /* The sampling time: this value will be be written back at the end of the
     recording operation.  Example: 661500 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  p_header[4] = (uint8_t)(total_bytes);
  p_header[5] = (uint8_t)(total_bytes >> 8);
  p_header[6] = (uint8_t)(total_bytes >> 16);
  p_header[7] = (uint8_t)(total_bytes >> 24);
  /* Write the number of sample data */
  /* This variable will be written back at the end of the recording operation */
  total_bytes -= 44;
  p_header[40] = (uint8_t)(total_bytes);
  p_header[41] = (uint8_t)(total_bytes >> 8);
  p_header[42] = (uint8_t)(total_bytes >> 16);
  p_header[43] = (uint8_t)(total_bytes >> 24);
  /* Return 0 if all operations are OK */
  return 0;
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
