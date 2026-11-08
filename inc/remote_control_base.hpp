/**
 *******************************************************************************
 * @file      : remote_control_base.hpp
 * @brief     : 遥控器基本定义
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
#ifndef HW_COMPONENTS_DEVICES_REMOTE_CONTROL_REMOTE_CONTROL_BASE_HPP_
#define HW_COMPONENTS_DEVICES_REMOTE_CONTROL_REMOTE_CONTROL_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace hello_world
{
namespace remote_control
{
/* Exported macro ------------------------------------------------------------*/

enum class SwitchState : uint8_t {
  kErr = 0u,
  kUp = 1u,
  kMid = 3u,
  kDown = 2u,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace remote_control
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REMOTE_CONTROL_REMOTE_CONTROL_BASE_HPP_ */
