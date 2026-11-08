/**
 *******************************************************************************
 * @file      :ins_filter.hpp
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
#ifndef INSTANCE_INS_FILTER_HPP_
#define INSTANCE_INS_FILTER_HPP_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace filter {
class Td;  // 自行实现，见 ins_filter.cpp
}  // namespace filter
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
namespace hw_filter = hello_world::filter;
/* 获取旋转电机速度反馈用的滤波器单例。 */
hw_filter::Td *CreateTdSpeedZ(void);

#endif /* INSTANCE_INS_FILTER_HPP_ */
