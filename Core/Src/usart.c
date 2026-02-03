/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
// 声明外部变量
extern float humidity;      // 湿度值（来自main.c）
extern float temperature;   // 温度值（来自main.c）
extern double gnss_longitude; // 经度（来自main.c GNSS模块）
extern double gnss_latitude;  // 纬度（来自main.c GNSS模块）
extern double gnss_speed;     // 车速（来自main.c GNSS模块）

// 数据上报全局变量定义
#include <string.h>
#include <stdio.h>

// 数据上报全局变量定义
uint8_t connect_flag = 0;          // 连接状态标志
const int message_id = 100;        // 消息ID，保持固定值

/**
  * @brief  发送AT指令到串口1
  * @param  command: 要发送的AT指令字符串
  * @retval None
  */
void SendATCommand(char* command)
{
    // 发送AT指令
    HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
    
    // 发送回车换行符
    uint8_t end[] = "\r\n";
    HAL_UART_Transmit(&huart1, end, 2, HAL_MAX_DELAY);
    
    // 等待一段时间让模块处理命令
    HAL_Delay(1000);
}

/**
  * @brief  发送连接阿里云的完整AT指令序列
  * @retval None
  */
/**
  * @brief  初始化并连接阿里云物联网平台
  * @retval None
  */
void ConnectToAliyun(void)
{
    if (connect_flag == 0) {
        // 1. 初始化配置
        SendATCommand("AT+QMTCFG=\"recv/mode\",0,0,1");
        HAL_Delay(500);
        
        // 2. 建立网络连接
        SendATCommand("AT+QMTOPEN=0,\"iot-example.mqtt.iothub.aliyuncs.com\",1883");
        HAL_Delay(2000);
        
        // 3. MQTT服务器认证
        SendATCommand("AT+QMTCONN=0,\"example-device|securemode=2,signmethod=hmacsha256,timestamp=1234567890|\",\"example&device\",\"example-key-1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef\"");
        HAL_Delay(3000);
        
        // 4. 订阅云端响应主题
        SendATCommand("AT+QMTSUB=0,1,\"/sys/example-device/example/thing/event/property/post_reply\",1");
        HAL_Delay(2000);
        
        connect_flag = 1; // 标记已连接
    }
}

/**
  * @brief  发送传感器数据到阿里云
  * @retval None
  */
void SendDataToAliyun(void)
{
    if (connect_flag == 1) {
        // 构建JSON数据，包含温湿度、GNSS定位数据和车速
        char json_data[512];
        sprintf(json_data, "{\"id\": \"%d\", \"version\": \"1.0\", \"params\": {\"mhumi\": %.1f, \"VehInsideTemp\": %.1f, \"GeoLocation\": {\"Longitude\": %.6f, \"Latitude\": %.6f, \"Altitude\": 50.5, \"CoordinateSystem\": 1}, \"VehSpeed\": %.2f}}, \"method\": \"thing.event.property.post\"}",
                message_id, humidity, temperature, gnss_longitude, gnss_latitude, gnss_speed);
        
        // 计算JSON数据长度
        int json_len = strlen(json_data);
        
        // 构建发布命令
        char pub_command[256];
        sprintf(pub_command, "AT+QMTPUBEX=0,2,1,0,\"/sys/example-device/example/thing/event/property/post\",%d", json_len);
        
        // 发送发布命令
        SendATCommand(pub_command);
        
        // 等待模块响应'>'提示符
        HAL_Delay(1000);
        
        // 发送JSON数据
        HAL_UART_Transmit(&huart1, (uint8_t*)json_data, json_len, HAL_MAX_DELAY);
        
        // 发送结束符
        uint8_t end[] = "\r\n";
        HAL_UART_Transmit(&huart1, end, 2, HAL_MAX_DELAY);
        
        // 等待响应
        HAL_Delay(2000);
    }
}

/**
  * @brief  发送连接阿里云的完整AT指令序列（保持兼容性）
  * @retval None
  */
void SendATCommandsToAliyun(void)
{
    ConnectToAliyun();
    SendDataToAliyun();
}
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
