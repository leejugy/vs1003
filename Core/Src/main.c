/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mp3sample.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define XCS_PIN  GPIO_PIN_4
#define XCS_PORT GPIOA

#define RESET_PIN GPIO_PIN_2
#define RESET_PORT GPIOA

#define XDCS_PIN  GPIO_PIN_1
#define XDCS_PORT GPIOA

#define DREQ_PIN  GPIO_PIN_3
#define DREQ_PORT GPIOA

#define VS1003_R 0x03
#define VS1003_W 0x02


#define ADD_MODE 				0x00
#define ADD_CLOCKF 			0x03
#define ADD_DECODE_TIME 0x04
#define ADD_AUDATA 			0x05
#define ADD_VOL			    0x0b

#define _32kbps_mono_sampling_rate (16000)*2+1
#define _32kbps_stereo_sampling_rate (12000)*2+1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_send_buffer[32];
uint8_t uart_rx_val;

uint16_t current_vol = 0x0F0F;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart_send(uint8_t uart_num,char *fmt,...);
void VS1003_WriteReg(uint8_t add, uint16_t val);
uint16_t VS1003_ReadReg(uint8_t add);
void VS1003_init();
void SDI_Send(uint8_t *MP3_Data,size_t MP3_Len);

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance){
		HAL_UART_Receive_DMA(&huart1, &uart_rx_val, 1);
	}
  UNUSED(huart);
}

void uart_send(uint8_t uart_num,char *fmt,...){
	va_list arg;
	va_start(arg,fmt);
	vsnprintf((char *)uart_send_buffer,32,fmt,arg);

	if(uart_num==1){
		HAL_UART_Transmit(&huart1, uart_send_buffer, 32, 10);
	}
	va_end(arg);
	memset(uart_send_buffer,0,32);
}



void VS1003_init(){
	uint16_t _16_data=0;

  HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(XCS_PORT, XCS_PIN, GPIO_PIN_SET);
  uint8_t retry = 0;

  while(1){
  	_16_data = VS1003_ReadReg(ADD_MODE);
  	VS1003_WriteReg(ADD_MODE,0x0804);
  	if(_16_data==0x800 || retry++>=100){
  		uart_send(1,"ADD_MODE : 0x%04X\n",_16_data);
  		break;
  	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = VS1003_ReadReg(ADD_CLOCKF);
    VS1003_WriteReg(ADD_CLOCKF,0x9BE8);
    if(_16_data==0x9BE8 || retry++>=100){
    	uart_send(1,"ADD_CLOCKF : 0x%04X\n",_16_data);
    	break;
   	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = VS1003_ReadReg(ADD_AUDATA);
    VS1003_WriteReg(ADD_AUDATA,_32kbps_stereo_sampling_rate);
    if(_16_data==_32kbps_stereo_sampling_rate || retry++>=100){
    	uart_send(1,"ADD_AUDATA : 0x%04X\n",_16_data);
    	break;
   	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = VS1003_ReadReg(ADD_VOL);
    VS1003_WriteReg(ADD_VOL,current_vol);
    if(_16_data==current_vol || retry++>=100){
    	uart_send(1,"ADD_VOL : 0x%04X\n",_16_data);
    	break;
   	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = VS1003_ReadReg(ADD_DECODE_TIME);
    VS1003_WriteReg(ADD_DECODE_TIME,0x0000);
    if(_16_data==0x0000 || retry++>=100){
    	uart_send(1,"ADD_DECODE_TIME : 0x%04X\n",_16_data);
    	break;
   	}
  }
  uint8_t tx_val = 0xff;



  HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1,&tx_val,1,10);
  HAL_SPI_Transmit(&hspi1,&tx_val,1,10);
  HAL_SPI_Transmit(&hspi1,&tx_val,1,10);
  HAL_SPI_Transmit(&hspi1,&tx_val,1,10);

  HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, GPIO_PIN_SET);

  HAL_Delay(20);
}

void SDI_Send(uint8_t *MP3_Data,size_t MP3_Len){
	HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, GPIO_PIN_RESET);
	for(uint8_t i=0;i<MP3_Len;i++){
		HAL_SPI_Transmit(&hspi1,MP3_Data++,1,10);
	}
	HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, GPIO_PIN_SET);
}

void control_mp3(uint32_t *data){
	if(uart_rx_val == 'r'){
		uart_send(1,"restart!\n");
	  *data=0;
	  uart_rx_val = 0;
	}
	else if(uart_rx_val == '-' && current_vol<(0xFDFD)){
		uart_send(1,"minus vol\n");
	  current_vol +=0x0101;
	  VS1003_WriteReg(ADD_VOL,current_vol);
	  uart_rx_val=0;
	}
	else if(uart_rx_val == '+' && (current_vol>0x0101)){
		uart_send(1,"plus vol\n");
	  current_vol -=0x0101;
	  VS1003_WriteReg(ADD_VOL,current_vol);
	  uart_rx_val=0;
	}
}

void VS1003_WriteReg(uint8_t add, uint16_t val){
	uint8_t tx_val;

	while(HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0);

	HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(XCS_PORT, XCS_PIN, GPIO_PIN_RESET);

	tx_val = VS1003_W;
	HAL_SPI_Transmit(&hspi1,&tx_val,1,10);
	tx_val = add;
	HAL_SPI_Transmit(&hspi1,&tx_val,1,10);
	tx_val = val>>8;
	HAL_SPI_Transmit(&hspi1,&tx_val,1,10);
	tx_val = val;
	HAL_SPI_Transmit(&hspi1,&tx_val,1,10);

	HAL_GPIO_WritePin(XCS_PORT, XCS_PIN, GPIO_PIN_SET);

}


uint16_t VS1003_ReadReg(uint8_t add){
	uint16_t ret;
	uint8_t tx_val;
	uint8_t rx_val;

	while(HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0);

	HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(XCS_PORT, XCS_PIN, GPIO_PIN_RESET);

	tx_val = VS1003_R;
	HAL_SPI_Transmit(&hspi1,&tx_val,1,10);
	tx_val = add;
	HAL_SPI_Transmit(&hspi1,&tx_val,1,10);

	HAL_SPI_TransmitReceive(&hspi1,&tx_val,&rx_val,1,10);
	ret = rx_val<<8;
	HAL_SPI_TransmitReceive(&hspi1,&tx_val,&rx_val,1,10);
	ret |= rx_val;

	HAL_GPIO_WritePin(XCS_PORT, XCS_PIN, GPIO_PIN_SET);

	return ret;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1, &uart_rx_val, 1);
  VS1003_init();
  uint32_t data = 0;
  bool end_flag=true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  	if(HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN)==GPIO_PIN_SET)
  		{
  			if(data+32<sizeof(MP3_DATA)){
  				SDI_Send(&MP3_DATA[data],32);
  				data+=32;
  			}
  			else if(end_flag){
  				uart_send(1,"end!\n");
  				end_flag=false;
  			}
  			control_mp3(&data);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
