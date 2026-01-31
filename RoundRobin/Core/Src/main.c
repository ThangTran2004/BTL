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
#include "cmsis_os.h"
#include "I2CLCD.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "SHT3X.h"
#include "DHT22.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SOURCE_ADC,
    SOURCE_SHT_TEMP,
	SOURCE_DHT_TEMP,
} DataSource_t;
typedef struct {
    DataSource_t source;
    float value;
} Message_t;
DHT22_HandleTypeDef myDHT22={
		.GPIOx=GPIOA,
		.GPIO_Pin=GPIO_PIN_2,
		.htim=&htim1
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Thread IDs
osThreadId_t TaskSHTHandle;
osThreadId_t TaskLCDHandle;
osThreadId_t TaskUARTHandle;
osThreadId_t TaskADCHandle;
osThreadId_t TaskDHTHandle;
osThreadId_t TaskCommandHandle;
osMessageQueueId_t uartQueueHandle;
osMessageQueueId_t lcdQueueHandle;
osMutexId_t I2CMutexHandle;
osMutexId_t DataMutexHandle;
osMutexId_t UARTMutexHandle;
osEventFlagsId_t uartEventHandle;

volatile uint32_t period_SHT  = 3000;
volatile uint32_t period_DHT  = 3000;
volatile uint32_t period_ADC  = 1000;
volatile uint32_t period_LCD  = 4000;
uint32_t ADC_Val;
uint8_t rx_cmd_buf[32];
uint8_t rx_cmd_idx = 0;
uint8_t rx_byte;
uint8_t mode=0;
float last_temp = 0.0f;
uint32_t last_adc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void StartTaskSHT(void *argument);
void StartTaskLCD(void *argument);
void StartTaskUART(void *argument);
void StartTaskADC(void *argument);
void StartTaskDHT(void *argument);
void StartTaskCommand(void *argument);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  DHT22_Init(&myDHT22);
  HAL_TIM_Base_Start(myDHT22.htim);
  lcd_init();
  SHT31_Init();
  // Tạo các đối tượng RTOS
  I2CMutexHandle = osMutexNew(NULL);
  DataMutexHandle= osMutexNew(NULL);
  UARTMutexHandle= osMutexNew(NULL);
  uartEventHandle = osEventFlagsNew(NULL);
  uartQueueHandle = osMessageQueueNew(20, sizeof(Message_t), NULL);
  osKernelInitialize();

  // Định nghĩa thuộc tính chung (Cùng Priority để chạy Round Robin)
  const osThreadAttr_t rr_attributes = {
      .stack_size = 128 * 4,
      .priority = osPriorityNormal
  };
  const osThreadAttr_t uart_attributes = {
      .stack_size = 256 * 4,
      .priority = osPriorityNormal // Cao hơn để ưu tiên in dữ liệu
  };
  const osThreadAttr_t command_attributes = {
    .name = "TaskCommand",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  TaskADCHandle = osThreadNew(StartTaskADC, NULL, &rr_attributes);
  TaskDHTHandle = osThreadNew(StartTaskDHT, NULL, &rr_attributes);
  TaskSHTHandle  = osThreadNew(StartTaskSHT,  NULL, &rr_attributes);
  TaskUARTHandle = osThreadNew(StartTaskUART, NULL, &uart_attributes);
  TaskLCDHandle  = osThreadNew(StartTaskLCD,  NULL, &uart_attributes);
  TaskCommandHandle = osThreadNew(StartTaskCommand, NULL, &command_attributes);
  osThreadSuspend(TaskDHTHandle);
  if (TaskCommandHandle == NULL) {
      // Nếu nhảy vào đây, chắc chắn là do hết Heap RAM
      HAL_UART_Transmit(&huart1, (uint8_t*)"ISR Task Creation Failed!\r\n", 27, 100);
  }
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  /* USER CODE END 2 */

  //MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
// --- TASK : DHT (Chạy xen kẽ mỗi 3s) ---
void StartTaskSHT(void *argument) {
    float t, h;
    Message_t msg;
    uint32_t tick;
    for(;;) {
    	if(mode==1){
    		osThreadResume(TaskDHTHandle);
    		osThreadSuspend(TaskSHTHandle);
    	}
    	tick = osKernelGetTickCount();
    	if (osMutexAcquire(I2CMutexHandle, 10) == osOK) {
    	            SHT31_StartMeasure();
    	            osMutexRelease(I2CMutexHandle);
    	        }
    	osDelay(20);
    	if (osMutexAcquire(I2CMutexHandle, 10) == osOK){
    		if(SHT31_GetResult(&t, &h)==1){
    			if(osMutexAcquire(DataMutexHandle,10)== osOK){
    				last_temp=t;
    				osMutexRelease(DataMutexHandle);
    			}
    			msg.source = SOURCE_SHT_TEMP;
    			msg.value = t;
    			osMessageQueuePut(uartQueueHandle, &msg, 0, 10);
    		}
    		osMutexRelease(I2CMutexHandle);
    	}
        osDelayUntil(tick+period_SHT);
    }
}

// --- TASK : LCD (Bảo vệ bởi Mutex) ---
void StartTaskLCD(void *argument) {
    char str_buf[16];
    float t_display;
    uint32_t adc_display;
    uint32_t tick = osKernelGetTickCount();
    osDelay(100);
    for(;;) {
    	tick+=period_LCD;
        // 1. Lấy dữ liệu mới nhất từ kho lưu trữ
        osMutexAcquire(DataMutexHandle, osWaitForever);
        t_display = last_temp;
        adc_display = last_adc;
        osMutexRelease(DataMutexHandle);

        // 2. Hiển thị lên LCD
        if (osMutexAcquire(I2CMutexHandle, osWaitForever) == osOK) {
            lcd_clear();

            // Dòng 1: Nhiệt độ
            lcd_put_cur(0, 0);
            sprintf(str_buf, "Temp: %.1f C", t_display);
            lcd_send_string(str_buf);

            // Dòng 2: ADC
            lcd_put_cur(1, 0);
            sprintf(str_buf, "Gas: %lu", adc_display);
            lcd_send_string(str_buf);

            osMutexRelease(I2CMutexHandle);
        }
        osDelayUntil(tick);
    }
}

// --- TASK : UART ---
void StartTaskUART(void *argument) {
    Message_t received_msg;
    char str[64];
    for(;;) {
        if (osMessageQueueGet(uartQueueHandle, &received_msg, NULL, osWaitForever) == osOK) {

            int len = 0;
            if (received_msg.source == SOURCE_ADC) {
                len = sprintf(str, "[ADC] Gas: %d\r\n", (int)received_msg.value);
            } else if (received_msg.source == SOURCE_SHT_TEMP) {
                len = sprintf(str, "[SHT] Temp: %.2f C\r\n", received_msg.value);
            } else if (received_msg.source == SOURCE_DHT_TEMP) {
                len = sprintf(str, "[DHT] Temp: %.2f C\r\n", received_msg.value);
            }

            // Bảo vệ Mutex UART khi in
            if (osMutexAcquire(UARTMutexHandle, osWaitForever) == osOK) {
                HAL_UART_Transmit(&huart1, (uint8_t*)str, len, 100);
                osMutexRelease(UARTMutexHandle);
            }
        }
    }
}
// --- TASK : ADC ---
void StartTaskADC(void *argument) {
    Message_t msg_adc;
    uint32_t local_adc_val;
    HAL_ADCEx_Calibration_Start(&hadc1);
    uint32_t tick = osKernelGetTickCount();
    for(;;) {
        // Sử dụng osDelayUntil để giữ nhịp chính xác 2 giây
        tick += period_ADC;

        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                local_adc_val = HAL_ADC_GetValue(&hadc1);
                if(osMutexAcquire(DataMutexHandle,10)==osOK){
                	last_adc = local_adc_val;
                	osMutexRelease(DataMutexHandle);
                }
                msg_adc.source = SOURCE_ADC;
                msg_adc.value = (float)local_adc_val;
                osMessageQueuePut(uartQueueHandle, &msg_adc, 0, 10);
            }
            else{
            	msg_adc.source = SOURCE_ADC;
            	msg_adc.value = -999.0; // Dùng một giá trị đặc biệt để báo lỗi
            	osMessageQueuePut(uartQueueHandle, &msg_adc, 0, 10);
            }
        }
        HAL_ADC_Stop(&hadc1);
        osDelayUntil(tick);
    }
}
// --- Task DHT ---
void StartTaskDHT(void *argument) {
    DHT22_Data_t dht_data;
    Message_t msg;
    uint32_t tick ;

    for(;;) {
    	if(mode==0){
    		osThreadResume(TaskSHTHandle);
    		osThreadSuspend(TaskDHTHandle);
    	}
        tick = osKernelGetTickCount();

        if (DHT22_Read(&myDHT22, &dht_data) == 0) {
            osMutexAcquire(DataMutexHandle, 10);
            last_temp = dht_data.Temperature;
            osMutexRelease(DataMutexHandle);

            msg.source = SOURCE_DHT_TEMP;
            msg.value = dht_data.Temperature;
            osMessageQueuePut(uartQueueHandle, &msg, 0, 10);
        }

        osDelayUntil(tick+period_DHT);
    }
}
// --- Task Command xử lý lệnh từ UART ---
void StartTaskCommand(void *argument) {
    int val;
    char target_name[20]; // Dùng để lưu tên task vừa thay đổi để in DONE

    for(;;) {
        // Chờ tín hiệu từ ngắt UART (Semaphore/Notification)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        strcpy(target_name, "UNKNOWN"); // Mặc định

        // 1. Xử lý đổi Mode
        if (strcmp((char*)rx_cmd_buf, "MODE:SHT") == 0) {
            mode = 0;
            strcpy(target_name, "MODE SHT");
            osEventFlagsSet(uartEventHandle, 0x08);
        }
        else if (strcmp((char*)rx_cmd_buf, "MODE:DHT") == 0) {
            mode = 1;
            strcpy(target_name, "MODE DHT");
            osEventFlagsSet(uartEventHandle, 0x08);
        }
        // 2. Xử lý thay đổi chu kỳ (Ví dụ lệnh ADC:2000)
        else if (sscanf((char*)rx_cmd_buf, "ADC:%d", &val) == 1) {
            period_ADC = (uint32_t)val;
            strcpy(target_name, "ADC");
        }
        else if (sscanf((char*)rx_cmd_buf, "SHT:%d", &val) == 1) {
            period_SHT = (uint32_t)val;
            strcpy(target_name, "SHT");
        }
        else if (sscanf((char*)rx_cmd_buf, "LCD:%d", &val) == 1) {
            period_LCD = (uint32_t)val;
            strcpy(target_name, "LCD");
        }

        // 3. Phản hồi qua UART (Có bảo vệ Mutex)
        if (osMutexAcquire(UARTMutexHandle, 100) == osOK) {
            char msg[40];
            int len = sprintf(msg, ">> %s OK\n", target_name);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
            osMutexRelease(UARTMutexHandle);
        }

        // 4. Reset buffer an toàn (Dùng Critical Section để tránh ngắt chen vào)
        taskENTER_CRITICAL();
        memset(rx_cmd_buf, 0, sizeof(rx_cmd_buf));
        rx_cmd_idx = 0;
        taskEXIT_CRITICAL();
    }
}
// --- CALLBACK NGẮT UART ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Nếu nhận ký tự kết thúc
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_cmd_idx > 0) {
                rx_cmd_buf[rx_cmd_idx] = '\0'; // Kết thúc chuỗi
                vTaskNotifyGiveFromISR((TaskHandle_t)TaskCommandHandle, NULL);
            }
            rx_cmd_idx = 0;
        } else {
            if (rx_cmd_idx < sizeof(rx_cmd_buf) - 1) {
                rx_cmd_buf[rx_cmd_idx++] = rx_byte;
            }
        }
        // Tiếp tục nhận byte tiếp theo
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
