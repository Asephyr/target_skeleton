/**
 *******************************************************************************
 * @file      :ins_motor.hpp
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
#ifndef INSTANCE_INS_MOTOR_HPP_
#define INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hw_motor = hello_world::motor;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取左轮电机（M3508）单例，对象在 ins_motor.cpp 中静态定义。 */
hw_motor::Motor* CreateMotorLeft();
/* 获取右轮电机（M3508）单例。 */
hw_motor::Motor* CreateMotorRight();
/* 获取旋转电机（GM6020）单例。 */
hw_motor::Motor* CreateMotorRotate();

#endif /* INSTANCE_INS_MOTOR_HPP_ */
