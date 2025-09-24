/* Includes */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mpu6050.h"
#include "EKF.h"
#include "INS.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Private defines */
#define DEG_TO_RAD 0.01745329251994329576923690768489f
#define RAD_TO_DEG 57.295779513082320876798154814105f

// Define the frequency of the sensor reading task in Hz
#define SENSOR_TASK_FREQUENCY_HZ 100
// Calculate the fixed sample time (dt) in seconds
#define FIXED_DT_S (1.0f / SENSOR_TASK_FREQUENCY_HZ)
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
// MPU6050, EKF, and INS structures
MPU6050_Data_t MPU6050_Data;
EKF ekf;
INS_t ins;


/* FreeRTOS handles */
TaskHandle_t sensorTaskHandle;
TaskHandle_t processingTaskHandle;
TaskHandle_t reportingTaskHandle;

// Queues for inter-task communication
// This queue sends raw sensor data from sensorTask to processingTask
QueueHandle_t sensorDataQueue;
// This queue sends filtered results from processingTask to reportingTask
QueueHandle_t processedDataQueue;

/* Data structures for queues */
// Struct to hold raw sensor data for the queue
typedef struct {
    float Ax_g, Ay_g, Az_g;
    float Gx_dps, Gy_dps, Gz_dps;
    float dt_s;
} SensorQueueData_t;

// Struct to hold processed data for the queue
typedef struct {
    float roll_rad, pitch_rad;
    float pos_x_m, pos_y_m;
} ProcessedQueueData_t;



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);


// Task function prototypes
void sensorTask(void *pvParameters);
void processingTask(void *pvParameters);
void reportingTask(void *pvParameters);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retarget printf to UART
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  printf("System Initializing...\r\n");
  /* USER CODE BEGIN 2 */
  // Initialize MPU6050
  if (MPU6050_Init(&hi2c1) != HAL_OK) {
      printf("MPU6050 Initialization Failed\r\n");
      while (1);
  }
  printf("MPU6050 Initialized.\r\n");

  MPU6050_Calibrate(&hi2c1, &MPU6050_Data);

  // -- ADD THIS BLOCK for better EKF initialization --
  printf("Calculating initial attitude...\r\n");
  float initial_roll = 0.0f;
  float initial_pitch = 0.0f;
  // Take a few readings to get a stable starting value
  for(int i=0; i<50; i++) {
      MPU6050_Read_All(&hi2c1, &MPU6050_Data);
      initial_roll += atan2f(MPU6050_Data.Ay, MPU6050_Data.Az);
      initial_pitch += atan2f(-MPU6050_Data.Ax, sqrtf(MPU6050_Data.Ay * MPU6050_Data.Ay + MPU6050_Data.Az * MPU6050_Data.Az));
      HAL_Delay(5);
  }
  initial_roll /= 50.0f;
  initial_pitch /= 50.0f;
  printf("Initial Roll: %.2f, Pitch: %.2f\r\n", initial_roll * RAD_TO_DEG, initial_pitch * RAD_TO_DEG);
  // --------------------------------------------------

  // Initialize EKF
    EKF_Init(&ekf, 1.0f, 0.001f, 0.8f, initial_roll, initial_pitch);
    printf("EKF Initialized.\r\n");

    // Initialize INS
    INS_Init(&ins);
    printf("INS Initialized.\r\n");

    // Create queues
    sensorDataQueue = xQueueCreate(1, sizeof(SensorQueueData_t)); // Queue depth of 1 is enough
    processedDataQueue = xQueueCreate(1, sizeof(ProcessedQueueData_t));

    // Create tasks
    xTaskCreate(sensorTask, "SensorTask", 256, NULL, 3, &sensorTaskHandle);
    xTaskCreate(processingTask, "ProcessingTask", 1536, NULL, 2, &processingTaskHandle);
    xTaskCreate(reportingTask, "ReportingTask", 256, NULL, 1, &reportingTaskHandle);

    printf("Tasks created. Starting scheduler.\r\n");

    // Start the scheduler
    vTaskStartScheduler();





  while (1)
  {

  }

}

void sensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_tick = 0;

    for(;;) {
        // Use vTaskDelay instead of vTaskDelayUntil to allow for processing time
        vTaskDelay(pdMS_TO_TICKS(1000 / SENSOR_TASK_FREQUENCY_HZ));

        // --- DYNAMIC dt CALCULATION ---
        uint32_t current_tick = xTaskGetTickCount();
        float dt = (last_tick == 0) ? (1.0f / SENSOR_TASK_FREQUENCY_HZ) : (float)(current_tick - last_tick) / configTICK_RATE_HZ;
        last_tick = current_tick;
        // -----------------------------

        MPU6050_Read_All(&hi2c1, &MPU6050_Data);

        SensorQueueData_t dataToSend;
        dataToSend.Ax_g = MPU6050_Data.Ax;
        dataToSend.Ay_g = MPU6050_Data.Ay;
        dataToSend.Az_g = MPU6050_Data.Az;
        dataToSend.Gx_dps = MPU6050_Data.Gx;
        dataToSend.Gy_dps = MPU6050_Data.Gy;
        dataToSend.Gz_dps = MPU6050_Data.Gz;
        dataToSend.dt_s = dt; // Add dt to the message

        xQueueOverwrite(sensorDataQueue, &dataToSend);
    }
}


/**
 * @brief Task to process sensor data using EKF and INS.
 * @param pvParameters Not used.
 */
void processingTask(void *pvParameters) {
    SensorQueueData_t receivedData;

    for(;;) {
        // Wait indefinitely until new sensor data is available in the queue.
    	   if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
    	            // 1. EKF Predict step (using the MEASURED dt)
    	            EKF_Predict(&ekf,
    	                        receivedData.Gx_dps * DEG_TO_RAD,
    	                        receivedData.Gy_dps * DEG_TO_RAD,
    	                        receivedData.Gz_dps * DEG_TO_RAD,
    	                        receivedData.dt_s); // <-- USE DYNAMIC dt

    	            // 2. EKF Update step
    	            EKF_Update(&ekf,
    	                       receivedData.Ax_g * G_MPS2,
    	                       receivedData.Ay_g * G_MPS2,
    	                       receivedData.Az_g * G_MPS2);

    	            // 3. INS Update step with ZUPT logic
    	            if (fabsf(ekf.phi_r) < (20.0f * DEG_TO_RAD) && fabsf(ekf.theta_r) < (20.0f * DEG_TO_RAD)) {
    	                INS_Update(&ins,
    	                           ekf.phi_r, ekf.theta_r,
    	                           receivedData.Ax_g, receivedData.Ay_g, receivedData.Az_g,
    	                           receivedData.dt_s); // <-- USE DYNAMIC dt
    	            } else {
    	                ins.velocity_x_mps = 0.0f;
    	                ins.velocity_y_mps = 0.0f;
    	            }

            // 4. Send processed data to the reporting task
            ProcessedQueueData_t dataToSend;
            dataToSend.roll_rad = ekf.phi_r;
            dataToSend.pitch_rad = ekf.theta_r;
            dataToSend.pos_x_m = ins.position_x_m;
            dataToSend.pos_y_m = ins.position_y_m;

            xQueueOverwrite(processedDataQueue, &dataToSend);
        }
    }
}

/**
 * @brief Task to report final data over UART.
 * @param pvParameters Not used.
 */
void reportingTask(void *pvParameters) {
    ProcessedQueueData_t receivedData;

    for(;;) {
        // Wait for processed data to become available
        if (xQueueReceive(processedDataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
            // Print results for debugging/plotting
            // Convert radians to degrees for easier reading
            printf("Roll: %.2f, Pitch: %.2f, X_pos: %.2f, Y_pos: %.2f\r\n",
                   receivedData.roll_rad * RAD_TO_DEG,
                   receivedData.pitch_rad * RAD_TO_DEG,
                   receivedData.pos_x_m,
                   receivedData.pos_y_m);

            // Add a delay to this task to prevent flooding the UART port.
            // This won't affect the sensor/processing tasks due to its low priority.
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
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
