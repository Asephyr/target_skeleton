/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "can.h"
#include "usart.h"

#include "ins_all.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
namespace
{
// 遥控器串口接收管理器：由 CommTaskInit() 创建，UART 回调通过它转发。
hw_comm::UartRxMgr *uart_rx_mgr_ptr = nullptr;

// 电机实例：由 CommTaskInit() 获取，CommTask / CAN 回调使用。
hw_motor::Motor *motor_left = nullptr;
hw_motor::Motor *motor_right = nullptr;
hw_motor::Motor *motor_rotate = nullptr;

// CAN 接收缓冲：供 HAL_CAN_RxFifo0MsgPendingCallback 使用。
CAN_RxHeaderTypeDef can1_rx_header;
uint8_t can1_rx_data[8];
}  // namespace
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  motor_left = CreateMotorLeft();
  motor_right = CreateMotorRight();
  motor_rotate = CreateMotorRotate();

  uart_rx_mgr_ptr = CreateRcRxMgr();
  if (uart_rx_mgr_ptr != nullptr) {
    uart_rx_mgr_ptr->addReceiver(CreateRemoteControl());
    uart_rx_mgr_ptr->startReceive();
  }

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1,
                                   CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
}

void CommTask(void) {
  struct Frame {
    uint32_t id = 0;
    uint8_t data[8] = {0};
  };

  hw_motor::Motor *motors[] = {motor_left, motor_right, motor_rotate};
  Frame frames[3] = {};
  size_t frame_count = 0;

  for (hw_motor::Motor *motor : motors) {
    if (motor == nullptr) {
      continue;
    }

    Frame *frame = nullptr;
    for (size_t i = 0; i < frame_count; ++i) {
      if (frames[i].id == motor->txId()) {
        frame = &frames[i];
        break;
      }
    }
    if (frame == nullptr) {
      frame = &frames[frame_count++];
      frame->id = motor->txId();
    }

    size_t len = 8;
    motor->encode(len, frame->data);
  }

  for (size_t i = 0; i < frame_count; ++i) {
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = frames[i].id;
    header.ExtId = 0;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    HAL_CAN_AddTxMessage(&hcan1, &header, frames[i].data, &mailbox);
  }
}
/* Private function definitions ----------------------------------------------*/
/* CAN1 接收回调框架已给出：取帧 + 重新使能中断，数据分发留给你。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1)
    {
        return;
    }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1_rx_header,
                             can1_rx_data) != HAL_OK)
    {
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        return;
    }

    hw_motor::Motor *motors[] = {motor_left, motor_right, motor_rotate};
    for (hw_motor::Motor *motor : motors) {
      if (motor != nullptr && motor->rxId() == can1_rx_header.StdId) {
        motor->decode(can1_rx_header.DLC, can1_rx_data,
                      can1_rx_header.StdId);
        break;
      }
    }

    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (uart_rx_mgr_ptr == nullptr)
    {
        return;
    }
    if (huart != &huart3)
    {
        return;
    }
    uart_rx_mgr_ptr->rxEventCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (uart_rx_mgr_ptr == nullptr)
    {
        return;
    }
    uart_rx_mgr_ptr->errorCallback(huart);
}
