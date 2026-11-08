/**
 *******************************************************************************
 * @file      : main_task.hpp
 * @brief     : 主任务（1ms 控制周期入口）
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
#ifndef TASK_MAIN_TASK_HPP_
#define TASK_MAIN_TASK_HPP_

#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void MainTaskInit(void);
void MainTask(void);

#ifdef __cplusplus
}
#endif
#endif /* TASK_MAIN_TASK_HPP_ */
