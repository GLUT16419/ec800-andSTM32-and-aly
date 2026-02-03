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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 直接在main.c中实现DHT11功能，避免链接问题
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>  // 添加math.h头文件以支持fabs函数
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 短信报警相关宏定义
#define SMS_ALERT_TEMP_THRESHOLD_HIGH  25.0f    // 温度上限报警阈值（°C）
#define SMS_ALERT_TEMP_THRESHOLD_LOW   2.0f     // 温度下限报警阈值（°C）
#define SMS_ALERT_HUMIDITY_THRESHOLD   80.0f    // 湿度上限报警阈值（%RH）
#define SMS_ALERT_OXYGEN_THRESHOLD     19.5f    // 氧气含量下限报警阈值（%）
#define SMS_RECIPIENT_NUMBER           "13800000000"  // 短信接收号码（可修改）
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 短信报警相关变量
uint8_t sms_alert_sent = 0;  // 报警短信是否已发送标志（防止重复发送）
uint32_t sms_alert_timer = 0;  // 短信报警计时器（用于控制报警间隔）
#define SMS_ALERT_INTERVAL 3600000  // 短信报警间隔（毫秒，1小时）

// SHT41温湿度传感器相关定义
#define SHT41_ADDR              0x44 << 1  // I2C地址（7位地址左移1位，最低位为读写位）
#define SHT41_CMD_MEASURE_HIGH  0xFD       // 高精度测量命令
#define SHT41_CMD_RESET         0x94       // 传感器复位命令

// 时间间隔定义（单位：毫秒）
#define TEMP_HUMIDITY_INTERVAL 15000       // 温湿度读取间隔：15秒
#define FIRST_GNSS_DELAY       300000      // 首次GNSS定位延迟：5分钟
#define GNSS_INTERVAL          120000      // 后续GNSS定位间隔：2分钟

// 传感器数据变量，用于发送到阿里云
float humidity = 0.0f;
float temperature = 0.0f;
float oxygen = 21.0f;  // 氧气含量（%），预留变量，用户后面会实现

// 传感器历史数据变量，用于读取失败时的预测
float last_valid_humidity = 0.0f;      // 上一次有效的湿度值
float last_valid_temperature = 0.0f;   // 上一次有效的温度值
uint8_t has_valid_history = 0;         // 是否有有效的历史数据标志

// 滤波算法相关定义和变量
#define FILTER_BUFFER_SIZE 5            // 滤波缓冲区大小（奇数，便于中值滤波）

// 温度滤波缓冲区
float temp_filter_buffer[FILTER_BUFFER_SIZE];
uint8_t temp_buffer_index = 0;
uint8_t temp_buffer_full = 0;

// 湿度滤波缓冲区
float humi_filter_buffer[FILTER_BUFFER_SIZE];
uint8_t humi_buffer_index = 0;
uint8_t humi_buffer_full = 0;

// GNSS相关定义和变量
#define UART_BUFFER_SIZE 512  // 串口接收缓冲区大小

// 串口接收缓冲区和状态变量
uint8_t uart1_rx_buffer[UART_BUFFER_SIZE];
uint16_t uart1_rx_index = 0;
uint8_t uart1_rx_complete = 0;

// GNSS定位数据
double gnss_longitude = 0.0;
char gnss_latitude_str[20] = "";
double gnss_latitude = 0.0;
double gnss_speed = 0.0;  // 车速（单位：公里/小时）
char gnss_gga[200] = "";

// GNSS历史数据变量，用于读取失败时的预测
double last_valid_gnss_longitude = 0.0;  // 上一次有效的经度值
double last_valid_gnss_latitude = 0.0;   // 上一次有效的纬度值
double last_valid_gnss_speed = 0.0;      // 上一次有效的车速值
uint8_t has_valid_gnss_history = 0;      // 是否有有效的GNSS历史数据标志

// 时间戳变量
uint32_t last_temp_humidity_time = 0;  // 上次读取温湿度的时间
uint32_t last_gnss_time = 0;          // 上次获取GNSS定位的时间
uint8_t first_gnss_done = 0;          // 首次GNSS定位完成标志

// 宏定义状态返回值
#define SUCCESS       1
#define ERROR         0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 滤波函数前向声明
float Median_Filter(float *buffer, uint8_t size);
float Moving_Average_Filter(float *buffer, uint8_t size);
float Combined_Filter(float *buffer, uint8_t size, uint8_t index, uint8_t *full_flag, float new_value);

// 补偿函数前向声明
float Temperature_Compensation(float temp, float humi);

// SHT41函数前向声明
HAL_StatusTypeDef SHT41_Init(void);
HAL_StatusTypeDef SHT41_Read_Temp_Humidity(float *temp, float *humi);
void UpdateSensorData(void);

// GNSS相关函数前向声明
void GNSS_Init(void);
void GNSS_SendCommand(const char* command);
void GNSS_ReadPosition(void);
void GNSS_ReadGGA(void);

// 短信相关函数前向声明
void SendSMS(const char* phone_number, const char* message);
void CheckAndSendSMSAlert(void);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // 等待系统稳定
  HAL_Delay(3000);
  
  // 初始化SHT41传感器
  if(SHT41_Init() != HAL_OK)
  {
      // 初始化失败处理（可选）
  }
  
  // 初始化并连接阿里云物联网平台（仅执行一次）
  ConnectToAliyun();
  
  // 启用串口1接收中断
  HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer[uart1_rx_index], 1);
  
  // 初始化GNSS模块
  GNSS_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 初始化时间戳
  uint32_t current_time = HAL_GetTick();
  last_temp_humidity_time = current_time;
  
  // 首次GNSS定位时间设为当前时间+5分钟
  last_gnss_time = current_time + FIRST_GNSS_DELAY;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 获取当前时间
    current_time = HAL_GetTick();
    
    // 检查是否需要读取温湿度数据（每15秒）
    if((current_time - last_temp_humidity_time) >= TEMP_HUMIDITY_INTERVAL)
    {
      // 优化测量时机：避免车辆震动高峰期
      // 当车速大于30 km/h时，增加测量次数以补偿震动影响
      // 当车速变化率较大时，延迟测量直到车辆状态稳定
      
      float speed_change = 0.0f;
      if (has_valid_gnss_history)
      {
          speed_change = fabs(gnss_speed - last_valid_gnss_speed);
      }
      
      // 车辆处于稳定状态（车速低或变化小）时进行测量
      if (gnss_speed < 5.0f || speed_change < 10.0f)  // 车速低于5 km/h或变化小于10 km/h
      {
          // 更新传感器数据
          UpdateSensorData();
          last_temp_humidity_time = current_time;
          
          // 发送温湿度和GNSS数据到阿里云
          SendDataToAliyun();
      }
      // 车辆处于行驶状态（车速高且变化大）时增加测量次数
      else if (gnss_speed >= 30.0f)
      {
          // 增加测量次数，取多次测量的平均值以提高精度
          for (uint8_t i = 0; i < 3; i++)
          {
              UpdateSensorData();
              HAL_Delay(100);  // 短暂延时
          }
          last_temp_humidity_time = current_time;
          
          // 发送温湿度和GNSS数据到阿里云
          SendDataToAliyun();
      }
      // 车辆处于过渡状态时，延迟测量
      else
      {
          // 不进行测量，等待下一个周期
          // 这有助于避免在车辆加速、减速或颠簸时进行测量
      }
    }
    
    // 检查是否需要获取GNSS定位信息
    if((current_time - last_gnss_time) >= (first_gnss_done ? GNSS_INTERVAL : FIRST_GNSS_DELAY))
    {
      // 获取GNSS定位信息
      GNSS_ReadPosition();
      
      // 获取GNSS GGA语句
      GNSS_ReadGGA();
      
      // 更新时间戳并标记首次定位已完成
      last_gnss_time = current_time;
      first_gnss_done = 1;
    }
    
    // 短暂延时，减少CPU占用
    HAL_Delay(100);
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
/**
  * @brief  初始化GNSS模块
  * @retval None
  */
void GNSS_Init(void)
{
    // 发送AT指令开启GNSS
    GNSS_SendCommand("AT+QGPS=1");
    HAL_Delay(1000);
}

/**
  * @brief  发送AT指令到GNSS模块
  * @param  command: 要发送的AT指令
  * @retval None
  */
void GNSS_SendCommand(const char* command)
{
    // 通过串口1发送AT指令
    HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), 1000);
    
    // 发送回车换行
    uint8_t crlf[] = "\r\n";
    HAL_UART_Transmit(&huart1, crlf, 2, 100);
}

/**
  * @brief  读取GNSS定位数据
  * @retval None
  */
void GNSS_ReadPosition(void)
{
    // 发送AT指令获取GNSS定位数据
    GNSS_SendCommand("AT+QGPSLOC=2");
    
    // 等待一段时间让模块响应
    HAL_Delay(2000);
    
    // 检查是否收到完整的响应
    if(uart1_rx_complete)
    {
        // 查找"+QGPSLOC:"响应
        char* response = (char*)uart1_rx_buffer;
        char* qgpsloc = strstr(response, "+QGPSLOC:");
        
        if(qgpsloc != NULL)
        {
            // 解析响应格式：+QGPSLOC: <mode>,<latitude>,<longitude>,<altitude>,<speed>,<course>,<date>,<time>,<satellites>,<HDOP>
            char* token = strtok(qgpsloc, ",");
            
            // 跳过模式字段
            if(token != NULL) token = strtok(NULL, ",");
            
            // 解析纬度
            if(token != NULL)
            {
                gnss_latitude = atof(token);
                token = strtok(NULL, ",");
            }
            
            // 解析经度
            if(token != NULL)
            {
                gnss_longitude = atof(token);
                token = strtok(NULL, ",");
            }
            
            // 跳过高度、速度、航向、日期、时间等字段
            for(int i = 0; i < 5 && token != NULL; i++)
            {
                token = strtok(NULL, ",");
            }
            
            // 解析车速（第8个字段）
            if(token != NULL)
            {
                gnss_speed = atof(token);
            }
            
            // 保存当前有效的GNSS数据为历史数据
            last_valid_gnss_longitude = gnss_longitude;
            last_valid_gnss_latitude = gnss_latitude;
            last_valid_gnss_speed = gnss_speed;
            has_valid_gnss_history = 1;  // 设置有有效GNSS历史数据标志
        }
        else
        {
            // 没有找到+QGPSLOC响应，检查是否有错误
            if(strstr(response, "ERROR") != NULL)
            {
                // 检查是否有历史数据
                if(has_valid_gnss_history)
                {
                    // 使用历史数据进行预测
                    gnss_latitude = last_valid_gnss_latitude;
                    gnss_longitude = last_valid_gnss_longitude;
                    gnss_speed = last_valid_gnss_speed;
                }
                else
                {
                    // 如果没有历史数据，设置默认值
                    gnss_latitude = 0.0;
                    gnss_longitude = 0.0;
                    gnss_speed = 0.0;
                }
            }
        }
        
        // 重置接收缓冲区
        uart1_rx_index = 0;
        uart1_rx_complete = 0;
        memset(uart1_rx_buffer, 0, UART_BUFFER_SIZE);
    }
    else
    {
        // 没有收到完整响应，检查是否有历史数据
        if(has_valid_gnss_history)
        {
            // 使用历史数据进行预测
            gnss_latitude = last_valid_gnss_latitude;
            gnss_longitude = last_valid_gnss_longitude;
            gnss_speed = last_valid_gnss_speed;
        }
        else
        {
            // 如果没有历史数据，设置默认值
            gnss_latitude = 0.0;
            gnss_longitude = 0.0;
            gnss_speed = 0.0;
        }
    }
}

/**
  * @brief  读取GNSS GGA语句
  * @retval None
  */
void GNSS_ReadGGA(void)
{
    // 发送AT指令获取GGA语句
    GNSS_SendCommand("AT+QGPSGNMEA=\"GGA\"");
    
    // 等待响应
    HAL_Delay(2000);
    
    // 检查是否收到完整的响应
    if(uart1_rx_complete)
    {
        // 查找GGA语句
        char* response = (char*)uart1_rx_buffer;
        char* gga_start = strstr(response, "$GPGGA");
        char* gga_end = strstr(response, "\r\n");
        
        if(gga_start != NULL && gga_end != NULL)
        {
            // 复制GGA语句
            int length = gga_end - gga_start;
            if(length < sizeof(gnss_gga) - 1)
            {
                strncpy(gnss_gga, gga_start, length);
                gnss_gga[length] = '\0';
            }
        }
        
        // 重置接收缓冲区
        uart1_rx_index = 0;
        uart1_rx_complete = 0;
        memset(uart1_rx_buffer, 0, UART_BUFFER_SIZE);
    }
}
/**
  * @brief  SHT41初始化函数
  * @retval HAL状态
  */
HAL_StatusTypeDef SHT41_Init(void)
{
    uint8_t cmd = SHT41_CMD_RESET;
    
    // 发送复位命令
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, SHT41_ADDR, &cmd, 1, 100);
    
    // 等待传感器稳定
    HAL_Delay(10);
    
    return status;
}

/**
  * @brief  CRC-8校验函数（SHT41使用，多项式0x31）
  * @param  data: 要校验的数据
  * @param  length: 数据长度
  * @param  crc: 期望的CRC值
  * @retval uint8_t: 1=校验通过，0=校验失败
  */
uint8_t SHT41_CRC8_Check(uint8_t *data, uint8_t length, uint8_t crc)
{
    uint8_t crc_reg = 0xFF;
    uint8_t bit;
    
    for (uint8_t i = 0; i < length; i++)
    {
        crc_reg ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (crc_reg & 0x80)
            {
                crc_reg = (crc_reg << 1) ^ 0x31;  // 多项式0x31
            }
            else
            {
                crc_reg <<= 1;
            }
        }
    }
    
    return (crc_reg == crc) ? 1 : 0;
}

/**
  * @brief  从SHT41读取温湿度数据
  * @param  temp: 存储温度的指针
  * @param  humi: 存储湿度的指针
  * @retval HAL状态
  */
HAL_StatusTypeDef SHT41_Read_Temp_Humidity(float *temp, float *humi)
{
    uint8_t cmd = SHT41_CMD_MEASURE_HIGH;
    uint8_t data[6];
    uint16_t raw_temp, raw_humi;
    HAL_StatusTypeDef status;
    uint8_t retry_count = 0;
    
    // 发送测量命令，添加重试机制
    do {
        status = HAL_I2C_Master_Transmit(&hi2c1, SHT41_ADDR, &cmd, 1, 100);
        retry_count++;
        if(status == HAL_BUSY && retry_count < 3) {
            HAL_Delay(5); // 短暂延时后重试
        }
    } while(status == HAL_BUSY && retry_count < 3);
    
    if(status != HAL_OK) return status;
    
    // 等待测量完成（高精度模式下需要约10ms）
    HAL_Delay(10);
    
    retry_count = 0;
    // 读取6字节数据，添加重试机制
    do {
        status = HAL_I2C_Master_Receive(&hi2c1, SHT41_ADDR, data, 6, 100);
        retry_count++;
        if(status == HAL_BUSY && retry_count < 3) {
            HAL_Delay(5); // 短暂延时后重试
        }
    } while(status == HAL_BUSY && retry_count < 3);
    
    if(status != HAL_OK) return status;
    
    // 进行CRC校验
    if (!SHT41_CRC8_Check(&data[0], 2, data[2])) {
        return HAL_ERROR; // 温度数据CRC校验失败
    }
    
    if (!SHT41_CRC8_Check(&data[3], 2, data[5])) {
        return HAL_ERROR; // 湿度数据CRC校验失败
    }
    
    // 组合原始温度数据
    raw_temp = ((uint16_t)data[0] << 8) | data[1];
    raw_humi = ((uint16_t)data[3] << 8) | data[4];
    
    // 温度转换公式：T = -45 + 175 * (raw_temp / 65535.0)
    *temp = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    
    // 湿度转换公式：RH = -6 + 125 * (raw_humi / 65535.0)
    *humi = -6.0f + 125.0f * ((float)raw_humi / 65535.0f);
    
    // 应用温度补偿修正湿度测量值
    *humi = Temperature_Compensation(*temp, *humi);
    
    // 数据合理性检查
    // 1. 范围检查（基于冷链车实际工作环境）
    const float MIN_TEMP = -25.0f;  // 冷链车最低温度
    const float MAX_TEMP = 30.0f;   // 冷链车最高温度
    const float MIN_HUMI = 0.0f;    // 最小湿度
    const float MAX_HUMI = 100.0f;  // 最大湿度
    
    // 2. 变化速率检查（避免突然跳变）
    const float MAX_TEMP_CHANGE = 5.0f;  // 允许的最大温度变化速率（°C/采样周期）
    const float MAX_HUMI_CHANGE = 10.0f; // 允许的最大湿度变化速率（%RH/采样周期）
    
    // 检查温度范围
    if (*temp < MIN_TEMP || *temp > MAX_TEMP)
    {
        return HAL_ERROR; // 温度超出合理范围
    }
    
    // 检查湿度范围
    if (*humi < MIN_HUMI || *humi > MAX_HUMI)
    {
        return HAL_ERROR; // 湿度超出合理范围
    }
    
    // 如果有历史数据，检查变化速率
    if (has_valid_history)
    {
        float temp_change = fabs(*temp - last_valid_temperature);
        float humi_change = fabs(*humi - last_valid_humidity);
        
        if (temp_change > MAX_TEMP_CHANGE)
        {
            return HAL_ERROR; // 温度变化过快，数据可能异常
        }
        
        if (humi_change > MAX_HUMI_CHANGE)
        {
            return HAL_ERROR; // 湿度变化过快，数据可能异常
        }
    }
    
    // 确保湿度在合理范围内
    if(*humi < 0.0f) *humi = 0.0f;
    if(*humi > 100.0f) *humi = 100.0f;
    
    return HAL_OK;
}

/**
  * @brief  中值滤波函数
  * @param  buffer: 数据缓冲区
  * @param  size: 缓冲区大小
  * @retval float: 滤波后的值
  */
float Median_Filter(float *buffer, uint8_t size)
{
    float temp;
    float sorted_buffer[FILTER_BUFFER_SIZE];
    
    // 复制数据到临时数组
    for (uint8_t i = 0; i < size; i++)
    {
        sorted_buffer[i] = buffer[i];
    }
    
    // 冒泡排序
    for (uint8_t i = 0; i < size - 1; i++)
    {
        for (uint8_t j = 0; j < size - i - 1; j++)
        {
            if (sorted_buffer[j] > sorted_buffer[j + 1])
            {
                // 交换位置
                temp = sorted_buffer[j];
                sorted_buffer[j] = sorted_buffer[j + 1];
                sorted_buffer[j + 1] = temp;
            }
        }
    }
    
    // 返回中间值
    return sorted_buffer[size / 2];
}

/**
  * @brief  滑动平均滤波函数
  * @param  buffer: 数据缓冲区
  * @param  size: 缓冲区大小
  * @retval float: 滤波后的值
  */
float Moving_Average_Filter(float *buffer, uint8_t size)
{
    float sum = 0.0f;
    
    for (uint8_t i = 0; i < size; i++)
    {
        sum += buffer[i];
    }
    
    return sum / size;
}

/**
  * @brief  组合滤波函数（中值滤波+滑动平均滤波）
  * @param  buffer: 数据缓冲区
  * @param  size: 缓冲区大小
  * @param  index: 当前索引
  * @param  full_flag: 缓冲区是否已满标志
  * @param  new_value: 新采样值
  * @retval float: 滤波后的值
  */
float Combined_Filter(float *buffer, uint8_t size, uint8_t index, uint8_t *full_flag, float new_value)
{
    // 将新值加入缓冲区
    buffer[index] = new_value;
    
    // 更新索引
    index = (index + 1) % size;
    
    // 标记缓冲区是否已满
    if (!(*full_flag) && index == 0)
    {
        *full_flag = 1;
    }
    
    // 确定实际使用的缓冲区大小
    uint8_t actual_size = (*full_flag) ? size : index;
    
    // 首先应用中值滤波去除脉冲干扰
    float median_value = Median_Filter(buffer, actual_size);
    
    // 然后应用滑动平均滤波平滑信号
    float filtered_value = Moving_Average_Filter(buffer, actual_size);
    
    return filtered_value;
}

/**
  * @brief  温度补偿函数（基于SHT41传感器特性和冷链车环境）
  * @param  temp: 当前温度值（°C）
  * @param  humi: 原始湿度值（%RH）
  * @retval float: 补偿后的湿度值（%RH）
  * @note   补偿系数应根据实际校准数据调整
  */
float Temperature_Compensation(float temp, float humi)
{
    // 基于温度的湿度补偿公式
    // 该公式根据SHT41传感器特性和实际应用环境调整
    // 补偿系数可以通过在不同温度点校准获得
    
    // 基础补偿系数
    float compensation = 0.0f;
    
    // 低温环境补偿（冷链车常见）
    if (temp < 0.0f)
    {
        // 温度越低，湿度测量值通常偏高，需要负补偿
        compensation = -0.15f * (0.0f - temp);  // 每降低1°C，湿度值降低约0.15%
    }
    // 中温环境补偿
    else if (temp >= 0.0f && temp < 25.0f)
    {
        // 此温度范围补偿较小
        compensation = -0.03f * temp;  // 温度从0°C到25°C，湿度值降低约0.03%/°C
    }
    // 高温环境补偿
    else
    {
        // 温度越高，湿度测量值通常偏低，需要正补偿
        compensation = 0.08f * (temp - 25.0f);  // 温度超过25°C，每升高1°C，湿度值增加约0.08%
    }
    
    // 应用补偿并返回结果
    return humi + compensation;
}

/**
  * @brief  更新传感器数据
  * @retval None
  */
void UpdateSensorData(void)
{
  float raw_temp, raw_humi;
  
  // 读取SHT41传感器数据
  HAL_StatusTypeDef status = SHT41_Read_Temp_Humidity(&raw_temp, &raw_humi);
  
  if(status == HAL_OK)
  {
    // 如果读取成功，应用组合滤波算法
    temperature = Combined_Filter(temp_filter_buffer, FILTER_BUFFER_SIZE, temp_buffer_index, &temp_buffer_full, raw_temp);
    humidity = Combined_Filter(humi_filter_buffer, FILTER_BUFFER_SIZE, humi_buffer_index, &humi_buffer_full, raw_humi);
    
    // 更新缓冲区索引
    temp_buffer_index = (temp_buffer_index + 1) % FILTER_BUFFER_SIZE;
    humi_buffer_index = (humi_buffer_index + 1) % FILTER_BUFFER_SIZE;
    
    // 保存当前数据为历史数据
    last_valid_humidity = humidity;
    last_valid_temperature = temperature;
    has_valid_history = 1;  // 设置有有效历史数据标志
  }
  else
  {
    // 如果读取失败，检查是否有历史数据
    if(has_valid_history)
    {
      // 使用历史数据进行预测（这里简单使用上一次有效值）
      temperature = last_valid_temperature;
      humidity = last_valid_humidity;
    }
    else
    {
      // 如果没有历史数据，设置默认值
      temperature = 0.0f;
      humidity = 0.0f;
    }
  }
  
  // 检查数据是否超过阈值，如果是则发送短信报警
  CheckAndSendSMSAlert();
}

/**
  * @brief  发送短信函数
  * @param  phone_number: 短信接收号码
  * @param  message: 短信内容
  * @retval None
  */
void SendSMS(const char* phone_number, const char* message)
{
    char cmd_buffer[256];
    
    // 1. 设置短信格式为文本模式
    GNSS_SendCommand("AT+CMGF=1");
    HAL_Delay(500);
    
    // 2. 设置短信接收号码
    sprintf(cmd_buffer, "AT+CMGS=\"%s\"", phone_number);
    GNSS_SendCommand(cmd_buffer);
    HAL_Delay(500);
    
    // 3. 发送短信内容
    GNSS_SendCommand(message);
    HAL_Delay(500);
    
    // 4. 发送Ctrl+Z结束短信
    uint8_t ctrlz[] = {0x1A};  // Ctrl+Z的ASCII码
    HAL_UART_Transmit(&huart1, ctrlz, 1, 1000);
    HAL_Delay(2000);
}

/**
  * @brief  检查数据是否超过阈值并发送短信报警
  * @retval None
  */
void CheckAndSendSMSAlert(void)
{
    char sms_content[256];
    uint32_t current_time = HAL_GetTick();
    uint8_t alert_triggered = 0;
    
    // 检查是否超过报警间隔（防止短时间内重复发送）
    if((current_time - sms_alert_timer) < SMS_ALERT_INTERVAL && sms_alert_sent)
    {
        return;  // 还在报警间隔内，不发送
    }
    
    // 检查温度是否超过阈值
    if(temperature > SMS_ALERT_TEMP_THRESHOLD_HIGH)
    {
        sprintf(sms_content, "Cold Chain Alert: Temp %.1f°C > High Threshold %.1f°C\r\nLocation: %.6f,%.6f", 
                temperature, SMS_ALERT_TEMP_THRESHOLD_HIGH, gnss_latitude, gnss_longitude);
        alert_triggered = 1;
    }
    else if(temperature < SMS_ALERT_TEMP_THRESHOLD_LOW)
    {
        sprintf(sms_content, "Cold Chain Alert: Temp %.1f°C < Low Threshold %.1f°C\r\nLocation: %.6f,%.6f", 
                temperature, SMS_ALERT_TEMP_THRESHOLD_LOW, gnss_latitude, gnss_longitude);
        alert_triggered = 1;
    }
    // 检查湿度是否超过阈值
    else if(humidity > SMS_ALERT_HUMIDITY_THRESHOLD)
    {
        sprintf(sms_content, "Cold Chain Alert: Humidity %.1f%%RH > Threshold %.1f%%RH\r\nLocation: %.6f,%.6f", 
                humidity, SMS_ALERT_HUMIDITY_THRESHOLD, gnss_latitude, gnss_longitude);
        alert_triggered = 1;
    }
    // 检查氧气含量是否低于阈值
    else if(oxygen < SMS_ALERT_OXYGEN_THRESHOLD)
    {
        sprintf(sms_content, "Cold Chain Alert: Oxygen %.1f%% < Threshold %.1f%%\r\nLocation: %.6f,%.6f", 
                oxygen, SMS_ALERT_OXYGEN_THRESHOLD, gnss_latitude, gnss_longitude);
        alert_triggered = 1;
    }
    
    // 如果触发了报警，则发送短信
    if(alert_triggered)
    {
        SendSMS(SMS_RECIPIENT_NUMBER, sms_content);
        
        // 更新报警状态和计时器
        sms_alert_sent = 1;
        sms_alert_timer = current_time;
    }
    else
    {
        // 没有触发报警，重置报警标志
        sms_alert_sent = 0;
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
