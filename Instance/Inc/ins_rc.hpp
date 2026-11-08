/**
 *******************************************************************************
 * @file      :ins_rc.hpp
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
#ifndef INSTANCE_INS_RC_HPP_
#define INSTANCE_INS_RC_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
namespace hw_rc = hello_world::remote_control;
/* 获取遥控器（DT7）解码对象单例。 */
hw_rc::DT7* CreateRemoteControl(void);

#endif /* INSTANCE_INS_RC_HPP_ */
