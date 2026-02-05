/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SHT3X.h"
#include "DHT22.h"
#include "I2CLCD.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    EVENT_NONE = 0,
    EVENT_GAS_READ,
	EVENT_TEMP_READ,
	EVENT_LCD,
	EVENT_ISR,
    EVENT_UART_CONTINUE,
} Event_t;
DHT22_HandleTypeDef dht;
DHT22_Data_t dht_data;
#define EVENT_QUEUE_SIZE 15
volatile Event_t event_queue[EVENT_QUEUE_SIZE];
volatile int8_t qhead = 0, qtail = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t rx_buffer[50];
volatile uint8_t rx_index = 0;
volatile uint8_t rx_byte;
volatile uint16_t period_ADC = 1000;
volatile uint16_t period_temp = 3000;
volatile uint16_t period_LCD = 4000;
volatile uint16_t timer_ADC = 0;
volatile uint16_t timer_temp = 0;
volatile uint16_t timer_LCD = 0;
char uart_tx_buf[32];
uint32_t ADC_val = 0;
float t, h;
char lcd_line1[16];
char lcd_line2[16];
uint8_t status;
uint8_t flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Push_Event(Event_t event) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq(); // Khóa ngắt tạm thời

    int8_t next_tail = (qtail + 1) % EVENT_QUEUE_SIZE;
    if (next_tail != qhead) {
        event_queue[qtail] = event;
        qtail = next_tail;
    }

    __set_PRIMASK(primask); // Khôi phục trạng thái ngắt
}

Event_t Pop_Event(void) {
    if (qhead == qtail) return EVENT_NONE;
    Event_t event = event_queue[qhead];
    qhead = (qhead + 1) % EVENT_QUEUE_SIZE;
    return event;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Kiểm tra ký tự kết thúc lệnh
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Kết thúc chuỗi an toàn
                Push_Event(EVENT_ISR); // Đẩy task xử lý lệnh vào hàng đợi
            }
        }
        else {
            if (rx_byte >= 32 && rx_byte <= 126) {
                if (rx_index < (sizeof(rx_buffer) - 1)) {
                    rx_buffer[rx_index++] = rx_byte;
                }
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
    }
}

void Check_UART_Error(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(huart); // Xóa cờ lỗi Overrun

        // Kích hoạt lại việc nhận ngắt vì khi có lỗi ORE, HAL sẽ dừng nhận dữ liệu
        HAL_UART_Receive_IT(huart, (uint8_t*)&rx_byte, 1);

        // Debug: Nháy LED hoặc gửi chuỗi báo lỗi để biết hệ thống vừa bị tràn
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}
void Task_ADC(void){
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){
        ADC_val = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
}
void Task_UART(void) {
    uint8_t len = strlen(uart_tx_buf);
    if (len > 0) {
        if (HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buf, len, 100) == HAL_OK) {
            uart_tx_buf[0] = '\0';
            memset(uart_tx_buf, 0, sizeof(uart_tx_buf));
        }
    }
}
void Task_Read_Sensor(void) {
    if (flag == 0) {
        status = SHT31_ReadData(&t, &h);
    } else {
    	int res = DHT22_Read(&dht, &dht_data);
        if(res==0){
        	status = 1;
        	t = dht_data.Temperature;
        }
        else{
        	int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf), "%d", res);
        	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buf, len, 100);
        }
    }
}
void Task_LCD(void) {
    char buf[16];

    //lcd_clear();

    lcd_put_cur(0, 0);
    if(status) {
        snprintf(buf, sizeof(buf), "Temp: %.1f C", t);
    } else {
        snprintf(buf, sizeof(buf), "Temp: Error");
    }
    lcd_send_string(buf);

    lcd_put_cur(1, 0);
    snprintf(buf, sizeof(buf), "Gas: %lu", ADC_val);
    lcd_send_string(buf);
}
void Task_ISR(void) {
    if (strstr((char*)rx_buffer, "SET_PERIOD") != NULL) {
        char target[10] = {0};
        int val = 0;
        if (sscanf((char*)rx_buffer, "SET_PERIOD:%9[^:]:%d", target, &val) == 2) {
        	if (strcmp(target, "ADC") == 0) {
        		period_ADC = (uint16_t)val;
                timer_ADC = 0;
                HAL_UART_Transmit(&huart1, (uint8_t*)"ADC OK\r\n", 8, 10);
            } else if (strcmp(target, "DHT") == 0 || strcmp(target, "SENSOR") == 0) {
                period_temp = (uint16_t)val;
                timer_temp = 0;
                HAL_UART_Transmit(&huart1, (uint8_t*)"TEMP OK\r\n", 9, 10);
            }
        }
    }
    else if (strstr((char*)rx_buffer, "MODE:SHT") != NULL) {
        flag = 0;
        HAL_UART_Transmit(&huart1, (uint8_t*)"MODE SHT\r\n", 10, 10);
    }
    else if (strstr((char*)rx_buffer, "MODE:DHT") != NULL) {
        flag = 1;
        HAL_UART_Transmit(&huart1, (uint8_t*)"MODE DHT\r\n", 10, 10);
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rx_index = 0;
    memset((uint8_t*)rx_buffer, 0, sizeof(rx_buffer));
    __set_PRIMASK(primask);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        timer_ADC++;
        timer_temp++;
        timer_LCD++;

        if (timer_ADC >= period_ADC) {
                    // Kiểm tra xem hàng đợi có còn chỗ không trước khi đẩy
                    int8_t next_tail = (qtail + 1) % EVENT_QUEUE_SIZE;
                    if (next_tail != qhead) {
                        Push_Event(EVENT_GAS_READ);
                    }
                    timer_ADC = 0;
                }
        if (timer_temp >= period_temp) {
            Push_Event(EVENT_TEMP_READ);
            timer_temp = 0;
        }
        if (timer_LCD >= period_LCD) {
            Push_Event(EVENT_LCD);
            timer_LCD = 0;
        }
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  SHT31_Init();
  lcd_init ();
  HAL_TIM_Base_Start_IT(&htim2); // Ngắt 1ms
  HAL_TIM_Base_Start(&htim1);
  dht.htim = &htim1;
  dht.GPIOx = GPIOA;
  dht.GPIO_Pin = GPIO_PIN_2;
  DHT22_Init(&dht);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
  HAL_DBGMCU_EnableDBGSleepMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
      {
	  Check_UART_Error(&huart1);
	  Event_t current_e = Pop_Event();

	  if (current_e != EVENT_NONE) {
		  switch(current_e) {
		  case EVENT_GAS_READ:
			  if (uart_tx_buf[0] == '\0') {
				  Task_ADC();
				  snprintf(uart_tx_buf, sizeof(uart_tx_buf), "ADC: %lu\r\n", ADC_val);
				  Task_UART();
			  } else {
				  Push_Event(EVENT_GAS_READ);
			  }
			  break;

		  case EVENT_TEMP_READ:
			  if (uart_tx_buf[0] == '\0') {
				  Task_Read_Sensor();
				  const char* name = (flag == 0) ? "SHT31" : "DHT22";
				  snprintf(uart_tx_buf, sizeof(uart_tx_buf), "%s: %.1f C\r\n", name, t);
				  Task_UART();
			  } else {
				  Push_Event(EVENT_TEMP_READ);
			  }
			  break;

		  case EVENT_LCD:
			  Task_LCD();
			  break;

		  case EVENT_ISR:
			  Task_ISR();
			  break;

		  default: break;
		  }
	  } else {
		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
