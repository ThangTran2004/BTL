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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT22.h"
#include "I2CLCD.h"
#include "SHT3X.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "queue.h"
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
typedef struct {
    const char *name;
    osThreadId_t handle;
    volatile uint32_t *period; // Trỏ tới biến period tương ứng
} RMS_Task_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_NUM 3
RMS_Task_t rmsTasks[TASK_NUM];
#define MIN_PERIOD_SENSOR  2000
#define MIN_PERIOD_ADC     100
#define MIN_PERIOD_LCD     500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
osThreadId_t TaskSHTHandle;
osThreadId_t TaskLCDHandle;
osThreadId_t TaskUARTHandle;
osThreadId_t TaskADCHandle;
osThreadId_t TaskDHTHandle;
osThreadId_t TaskCommandHandle;
osMessageQueueId_t uartQueueHandle;
osMessageQueueId_t mailboxTempHandle;
osMessageQueueId_t mailboxADCHandle;
osMutexId_t I2CMutexHandle;
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
const osMessageQueueAttr_t mailbox_attributes = { .name = "Mailbox" };
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
void RMS_UpdatePriority(void) {
    // 1. Sắp xếp danh sách Task theo chu kỳ (Bubble Sort)
    // Task có period nhỏ nhất sẽ đứng đầu mảng
    for (int i = 0; i < TASK_NUM - 1; i++) {
        for (int j = i + 1; j < TASK_NUM; j++) {
            if (*rmsTasks[j].period < *rmsTasks[i].period) {
                RMS_Task_t tmp = rmsTasks[i];
                rmsTasks[i] = rmsTasks[j];
                rmsTasks[j] = tmp;
            }
        }
    }

    // 2. Gán Priority mới dựa trên thứ tự đã sắp xếp
    osPriority_t basePri = osPriorityBelowNormal;
    for (int i = 0; i < TASK_NUM; i++) {
        // Task ở cuối mảng (period lớn nhất) nhận basePri
        // Task ở đầu mảng (period nhỏ nhất) nhận basePri + (TASK_NUM - 1)
        osPriority_t newPri = (osPriority_t)(basePri + (TASK_NUM - 1 - i));
        osThreadSetPriority(rmsTasks[i].handle, newPri);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SHT31_Init();
  lcd_init();
  HAL_TIM_Base_Start(myDHT22.htim);
  DHT22_Init(&myDHT22);
  I2CMutexHandle = osMutexNew(NULL);
  UARTMutexHandle= osMutexNew(NULL);
  uartEventHandle = osEventFlagsNew(NULL);
  uartQueueHandle = osMessageQueueNew(20, sizeof(Message_t), NULL);
  mailboxTempHandle = osMessageQueueNew(1, sizeof(float), &mailbox_attributes);
  mailboxADCHandle  = osMessageQueueNew(1, sizeof(uint32_t), &mailbox_attributes);
  const osThreadAttr_t adc_attributes = {
              .name = "TaskADC",
              .stack_size = 128 * 4,
              .priority = (osPriority_t) osPriorityAboveNormal,
            };
  const osThreadAttr_t temp_attributes = {
            .name = "TaskReadSensor",
            .stack_size = 128 * 4,
            .priority = (osPriority_t) osPriorityNormal,
          };
  const osThreadAttr_t uart_attributes = {
          .name = "TaskUART",
          .stack_size = 256 * 4,
          .priority = (osPriority_t) osPriorityLow,
        };
  const osThreadAttr_t lcd_attributes = {
        .name = "TaskLCD",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityBelowNormal,
      };
  const osThreadAttr_t command_attributes = {
      .name = "TaskCommand",
      .stack_size = 256 * 4,
      .priority = (osPriority_t) osPriorityHigh,
    };
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  //MX_FREERTOS_Init();
  TaskADCHandle = osThreadNew(StartTaskADC, NULL, &adc_attributes);
  TaskDHTHandle = osThreadNew(StartTaskDHT, NULL, &temp_attributes);
  TaskSHTHandle = osThreadNew(StartTaskSHT, NULL, &temp_attributes);
  TaskUARTHandle = osThreadNew(StartTaskUART, NULL, &uart_attributes);
  TaskLCDHandle = osThreadNew(StartTaskLCD, NULL, &lcd_attributes);
  TaskCommandHandle = osThreadNew(StartTaskCommand, NULL, &command_attributes);
  osThreadSuspend(TaskDHTHandle);
  if (TaskCommandHandle == NULL) {
  // Nếu nhảy vào đây, chắc chắn là do hết Heap RAM
  HAL_UART_Transmit(&huart1, (uint8_t*)"ISR Task Creation Failed!\r\n", 27, 100);
  }
  rmsTasks[0] = (RMS_Task_t){"ADC", TaskADCHandle, &period_ADC};
  rmsTasks[1] = (RMS_Task_t){"SHT", TaskSHTHandle, &period_SHT};
  rmsTasks[2] = (RMS_Task_t){"LCD", TaskLCDHandle, &period_LCD};
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
// --- TASK : SHT ---
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
    			xQueueOverwrite((QueueHandle_t)mailboxTempHandle, &t);
    			msg.source = SOURCE_SHT_TEMP;
    			msg.value = t;
    			osMessageQueuePut(uartQueueHandle, &msg, 0, 10);
    		}
    		osMutexRelease(I2CMutexHandle);
    	}
        osDelayUntil(tick+period_SHT);
    }
}

// --- TASK : LCD ---
void StartTaskLCD(void *argument) {
    char str_buf[16];
    float current_t = 0;
    uint32_t current_adc = 0;
    uint32_t tick = osKernelGetTickCount();
    osDelay(100);
    for(;;) {
    	tick+=period_LCD;
        // 1. Lấy dữ liệu mới nhất từ kho lưu trữ
        xQueuePeek((QueueHandle_t)mailboxTempHandle, &current_t, 0);
        xQueuePeek((QueueHandle_t)mailboxADCHandle, &current_adc, 0);

        // 2. Hiển thị lên LCD
        if (osMutexAcquire(I2CMutexHandle, osWaitForever) == osOK) {
            lcd_clear();

            // Dòng 1: Nhiệt độ
            lcd_put_cur(0, 0);
            sprintf(str_buf, "Temp: %.1f C", current_t);
            lcd_send_string(str_buf);

            // Dòng 2: ADC
            lcd_put_cur(1, 0);
            sprintf(str_buf, "Gas: %lu", current_adc);
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
        tick += period_ADC;

        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                local_adc_val = HAL_ADC_GetValue(&hadc1);
                xQueueOverwrite((QueueHandle_t)mailboxADCHandle, &local_adc_val);
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
    int status;
    for(;;) {
    	if(mode==0){
    		osThreadResume(TaskSHTHandle);
    		osThreadSuspend(TaskDHTHandle);
    	}
        tick = osKernelGetTickCount();
        status = DHT22_Read(&myDHT22, &dht_data);
        if (status== 0) {
            xQueueOverwrite((QueueHandle_t)mailboxTempHandle, &dht_data.Temperature);
            msg.source = SOURCE_DHT_TEMP;
            msg.value = dht_data.Temperature;
            osMessageQueuePut(uartQueueHandle, &msg, 0, 10);
        }
        else{
        	char str[64];
        	int len = 0;
        	len = sprintf(str, "%d\r\n", (int)status);
        	if (osMutexAcquire(UARTMutexHandle, osWaitForever) == osOK) {
        	                HAL_UART_Transmit(&huart1, (uint8_t*)str, len, 100);
        	                osMutexRelease(UARTMutexHandle);
        }
        }
        osDelayUntil(tick+period_DHT);
    }
}
// --- Task Command xử lý lệnh từ UART ---
void StartTaskCommand(void *argument) {
    int val;
    char target_name[20];
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        bool updated = false;
        bool invalid = false; // Biến báo lỗi nếu nhập số quá nhỏ
        strcpy(target_name, "UNKNOWN");

        if (strcmp((char*)rx_cmd_buf, "MODE:SHT") == 0) {
            mode = 0;
            rmsTasks[1].handle = TaskSHTHandle;
            rmsTasks[1].period = &period_SHT;
            rmsTasks[1].name = "SHT";
            strcpy(target_name, "MODE SHT");
            updated = true;
        }
        else if (strcmp((char*)rx_cmd_buf, "MODE:DHT") == 0) {
            mode = 1;
            rmsTasks[1].handle = TaskDHTHandle;
            rmsTasks[1].period = &period_DHT;
            rmsTasks[1].name = "DHT";
            strcpy(target_name, "MODE DHT");
            updated = true;
        }
        else if (sscanf((char*)rx_cmd_buf, "ADC:%d", &val) == 1) {
            if (val >= MIN_PERIOD_ADC) {
                period_ADC = (uint32_t)val;
                strcpy(target_name, "ADC");
                updated = true;
            } else { invalid = true; }
        }
        else if (sscanf((char*)rx_cmd_buf, "SHT:%d", &val) == 1) {
            if (val >= MIN_PERIOD_SENSOR) {
                period_SHT = (uint32_t)val;
                strcpy(target_name, "SHT");
                updated = true;
            } else { invalid = true; }
        }
        else if (sscanf((char*)rx_cmd_buf, "DHT:%d", &val) == 1) {
            if (val >= MIN_PERIOD_SENSOR) {
                period_DHT = (uint32_t)val;
                strcpy(target_name, "DHT");
                updated = true;
            } else { invalid = true; }
        }
        else if (sscanf((char*)rx_cmd_buf, "LCD:%d", &val) == 1) {
            if (val >= MIN_PERIOD_LCD) {
                period_LCD = (uint32_t)val;
                strcpy(target_name, "LCD");
                updated = true;
            } else { invalid = true; }
        }

        // 3. Phản hồi UART
        if (osMutexAcquire(UARTMutexHandle, 100) == osOK) {
            char msg[64];
            if (invalid) {
                sprintf(msg, ">> ERROR: Period too small!\r\n");
            } else if (updated) {
                RMS_UpdatePriority();
                sprintf(msg, ">> %s Updated, RMS OK\r\n", target_name);
            } else {
                sprintf(msg, ">> %s OK\r\n", target_name);
            }
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            osMutexRelease(UARTMutexHandle);
        }
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
