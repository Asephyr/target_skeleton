/**
 *******************************************************************************
 * @file      :ins_comm.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INSTANCE_INS_COMM_HPP_
#define INSTANCE_INS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "uart_rx_mgr.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
namespace hw_comm = hello_world::comm;
/* 获取遥控器串口接收管理器单例。 */
hello_world::comm::UartRxMgr* CreateRcRxMgr(void);

/* 注：CAN 收发组件不在骨架提供范围内。若你的 CAN 层需要单例对象，
 * 可以在这里自行添加 getter，或直接在 Task/Src/comm_task.cpp 里静态定义。 */

#endif /* INSTANCE_INS_COMM_HPP_ */
