/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 电机位置闭环控制主程序
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief PID控制器结构体定义
 */
typedef struct
{
    float Kp;                // 比例系数
    float Ki;                // 积分系数
    float Kd;                // 微分系数
    float error;             // 当前误差
    float last_error;        // 上一次误差
    float integral;          // 积分项
    float output;            // 输出值
    float max_output;        // 最大输出限幅
    float max_integral;      // 积分限幅
} PID_HandleTypeDef;

/**
 * @brief 电机状态信息结构体定义
 */
typedef struct
{
    int16_t speed_rpm;       // 电机转速(rpm)
    int16_t real_current;    // 实际电流值
    int16_t angle;           // 单次角度值(0-8191)
    uint8_t temp;            // 电机温度(℃)
    int16_t last_angle;      // 上一次角度值
    int32_t total_angle;     // 累计绝对角度值
    int32_t loop_count;      // 角度循环计数(跨零计数)
} Motor_HandleTypeDef;

/**
 * @brief CAN通信配置结构体定义
 */
typedef struct
{
    uint32_t tx_id;          // 发送ID
    uint32_t rx_id;          // 接收ID
    CAN_TxHeaderTypeDef tx_header;  // CAN发送头
    CAN_RxHeaderTypeDef rx_header;  // CAN接收头
    uint8_t tx_data[8];      // 发送数据缓冲区
    uint8_t rx_data[8];      // 接收数据缓冲区
    uint32_t tx_mailbox;     // 发送邮箱
} CAN_Motor_HandleTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 电机控制参数宏定义
#define MOTOR_MAX_CURRENT    16384    // 电机最大电流值
#define MOTOR_ANGLE_MAX      8192     // 单圈角度最大值(12位编码器)
#define MOTOR_ANGLE_HALF     4096     // 单圈角度半值(跨零判断阈值)
#define PID_CONTROL_PERIOD   1        // PID控制周期(ms)

// CAN通信参数宏定义
#define CAN_MOTOR_TX_ID      0x200    // 电机控制指令发送ID
#define CAN_MOTOR_RX_ID      0x203    // 电机状态反馈接收ID

// PID参数默认配置
#define PID_DEFAULT_KP       3.5f     // 默认比例系数
#define PID_DEFAULT_KI       0.0f     // 默认积分系数
#define PID_DEFAULT_KD       11.0f    // 默认微分系数
#define PID_MAX_OUTPUT       16384.0f  // PID最大输出
#define PID_MAX_INTEGRAL     2000.0f  // PID最大积分值
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LIMIT_VALUE(val, max, min)  ((val) > (max) ? (max) : ((val) < (min) ? (min) : (val)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// 全局句柄定义
static CAN_Motor_HandleTypeDef hcan_motor;  // 电机CAN通信句柄
static Motor_HandleTypeDef hmotor;          // 电机状态句柄
static PID_HandleTypeDef hpid_pos;          // 位置PID句柄

static int32_t target_angle = 0;            // 目标角度值
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// CAN通信相关函数
static void CAN_Motor_Config(void);
static void CAN_Motor_SendCurrent(int16_t current);

// PID控制相关函数
static void PID_Init(PID_HandleTypeDef *hpid);
static float PID_Calculate(PID_HandleTypeDef *hpid, float target, float feedback);

// 电机控制相关函数
static void Motor_Init(Motor_HandleTypeDef *hmotor);
static void Motor_UpdateAngle(Motor_HandleTypeDef *hmotor, int16_t curr_angle);
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
  HAL_Init();                              // 初始化HAL库
  SystemClock_Config();                    // 配置系统时钟
  MX_GPIO_Init();                          // 初始化GPIO
  MX_CAN1_Init();                          // 初始化CAN1

  /* USER CODE BEGIN 2 */
  // 初始化电机状态
  Motor_Init(&hmotor);
  
  // 初始化PID控制器
  PID_Init(&hpid_pos);
  
  // 配置CAN滤波器并启动CAN
  CAN_Motor_Config();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 延时等待电机反馈数据
  HAL_Delay(100);
  
  // 初始化目标角度为当前电机角度
  if (hmotor.last_angle != -1)
  {
    target_angle = hmotor.total_angle;
  }
  else
  {
    target_angle = 0;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // PID位置闭环计算
    float pid_output = PID_Calculate(&hpid_pos, (float)target_angle, (float)hmotor.total_angle);
    
    // 发送电机控制电流
    CAN_Motor_SendCurrent((int16_t)pid_output);
    
    // 控制周期延时
    HAL_Delay(PID_CONTROL_PERIOD);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  初始化电机状态结构体
  * @param  hmotor: 电机状态句柄指针
  * @retval None
  */
static void Motor_Init(Motor_HandleTypeDef *hmotor)
{
  if (hmotor == NULL) return;
  
  memset(hmotor, 0, sizeof(Motor_HandleTypeDef));
  hmotor->last_angle = -1;  // 初始化为无效值，表示未收到首次角度
  hmotor->loop_count = 0;
  hmotor->total_angle = 0;
}

/**
  * @brief  更新电机累计角度值(处理跨零)
  * @param  hmotor: 电机状态句柄指针
  * @param  curr_angle: 当前采样角度值
  * @retval None
  */
static void Motor_UpdateAngle(Motor_HandleTypeDef *hmotor, int16_t curr_angle)
{
  if (hmotor == NULL) return;

  hmotor->angle = curr_angle;

  // 首次采样不计算差值
  if (hmotor->last_angle == -1)
  {
    hmotor->last_angle = curr_angle;
    return;
  }

  // 计算角度差值，处理跨零情况
  int16_t angle_diff = curr_angle - hmotor->last_angle;
  if (angle_diff < -MOTOR_ANGLE_HALF)
  {
    hmotor->loop_count++;  // 正方向跨零
  }
  else if (angle_diff > MOTOR_ANGLE_HALF)
  {
    hmotor->loop_count--;  // 负方向跨零
  }

  // 更新累计角度和历史角度
  hmotor->total_angle = (int32_t)hmotor->loop_count * MOTOR_ANGLE_MAX + curr_angle;
  hmotor->last_angle = curr_angle;
}

/**
  * @brief  配置CAN滤波器(电机反馈数据)
  * @retval None
  */
static void CAN_Motor_Config(void)
{
  CAN_FilterTypeDef can_filter_conf = {0};

  // 初始化CAN电机通信句柄
  hcan_motor.tx_id = CAN_MOTOR_TX_ID;
  hcan_motor.rx_id = CAN_MOTOR_RX_ID;
  
  // 配置CAN滤波器
  can_filter_conf.FilterActivation = ENABLE;
  can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_conf.FilterIdHigh = 0x0000;
  can_filter_conf.FilterIdLow = 0x0000;
  can_filter_conf.FilterMaskIdHigh = 0x0000;
  can_filter_conf.FilterMaskIdLow = 0x0000;
  can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter_conf.FilterBank = 0;
  can_filter_conf.SlaveStartFilterBank = 14;  // 区分主从滤波器组(针对双CAN)

  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_conf) != HAL_OK)
  {
    Error_Handler();
  }

  // 初始化CAN发送头
  hcan_motor.tx_header.StdId = hcan_motor.tx_id;
  hcan_motor.tx_header.IDE = CAN_ID_STD;
  hcan_motor.tx_header.RTR = CAN_RTR_DATA;
  hcan_motor.tx_header.DLC = 8;
  hcan_motor.tx_header.TransmitGlobalTime = DISABLE;
}

/**
  * @brief  发送电机控制电流
  * @param  current: 目标电流值
  * @retval None
  */
static void CAN_Motor_SendCurrent(int16_t current)
{
  // 电流限幅
  current = LIMIT_VALUE(current, MOTOR_MAX_CURRENT, -MOTOR_MAX_CURRENT);

  // 填充发送数据
  memset(hcan_motor.tx_data, 0, 8);
  hcan_motor.tx_data[4] = (uint8_t)(current >> 8);  // 电流高8位
  hcan_motor.tx_data[5] = (uint8_t)(current & 0xFF);// 电流低8位

  // 发送CAN报文
  if (HAL_CAN_AddTxMessage(&hcan1, &hcan_motor.tx_header, hcan_motor.tx_data, &hcan_motor.tx_mailbox) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  初始化PID控制器参数
  * @param  hpid: PID控制器句柄指针
  * @retval None
  */
static void PID_Init(PID_HandleTypeDef *hpid)
{
  if (hpid == NULL) return;

  // 初始化PID参数
  hpid->Kp = PID_DEFAULT_KP;
  hpid->Ki = PID_DEFAULT_KI;
  hpid->Kd = PID_DEFAULT_KD;
  hpid->error = 0.0f;
  hpid->last_error = 0.0f;
  hpid->integral = 0.0f;
  hpid->output = 0.0f;
  hpid->max_output = PID_MAX_OUTPUT;
  hpid->max_integral = PID_MAX_INTEGRAL;
}

/**
  * @brief  PID控制器计算
  * @param  hpid: PID控制器句柄指针
  * @param  target: 目标值
  * @param  feedback: 反馈值
  * @retval PID输出值
  */
static float PID_Calculate(PID_HandleTypeDef *hpid, float target, float feedback)
{
  if (hpid == NULL) return 0.0f;

  // 计算当前误差
  hpid->error = target - feedback;

  // 积分项计算与限幅
  hpid->integral += hpid->error;
  hpid->integral = LIMIT_VALUE(hpid->integral, hpid->max_integral, -hpid->max_integral);

  // PID核心计算
  hpid->output = hpid->Kp * hpid->error +                // 比例项
                 hpid->Ki * hpid->integral +             // 积分项
                 hpid->Kd * (hpid->error - hpid->last_error);  // 微分项

  // 输出限幅
  hpid->output = LIMIT_VALUE(hpid->output, hpid->max_output, -hpid->max_output);

  // 保存本次误差
  hpid->last_error = hpid->error;

  return hpid->output;
}

/**
  * @brief  CAN接收FIFO0中断回调函数
  * @param  hcan: CAN句柄指针
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1) return;

  // 读取接收数据
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan_motor.rx_header, hcan_motor.rx_data) == HAL_OK)
  {
    // 判断是否为电机反馈ID
    if (hcan_motor.rx_header.StdId == hcan_motor.rx_id)
    {
      // 解析电机反馈数据
      int16_t curr_angle = (hcan_motor.rx_data[0] << 8) | hcan_motor.rx_data[1];
      hmotor.speed_rpm = (hcan_motor.rx_data[2] << 8) | hcan_motor.rx_data[3];
      hmotor.real_current = (hcan_motor.rx_data[4] << 8) | hcan_motor.rx_data[5];
      hmotor.temp = hcan_motor.rx_data[6];

      // 更新电机累计角度
      Motor_UpdateAngle(&hmotor, curr_angle);
    }
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
  __disable_irq();
  while (1)
  {
    // 可添加错误指示（如LED闪烁）
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(500);
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
  printf("Assert failed: File %s, Line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

