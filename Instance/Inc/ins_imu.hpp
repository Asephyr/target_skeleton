/**
 *******************************************************************************
 * @file      :ins_imu.hpp
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
#ifndef INSTANCE_INS_IMU_HPP_
#define INSTANCE_INS_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板 IMU 单例，对象在 ins_imu.cpp 中静态定义。 */
hello_world::imu::Imu *CreateImu(void);

#endif /* INSTANCE_INS_IMU_HPP_ */
