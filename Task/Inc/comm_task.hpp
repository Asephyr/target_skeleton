/**
 *******************************************************************************
 * @file      : comm_task.hpp
 * @brief     : 通讯层任务（CAN / UART 收发管理）
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
#ifndef TASK_COMM_TASK_HPP_
#define TASK_COMM_TASK_HPP_

#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void CommTaskInit(void);
void CommTask(void);

#ifdef __cplusplus
}
#endif
#endif /* TASK_COMM_TASK_HPP_ */
