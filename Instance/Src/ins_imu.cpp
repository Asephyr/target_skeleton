/**
 *******************************************************************************
 * @file      :ins_imu.cpp
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
#include "ins_imu.hpp"

#include "spi.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace {

hello_world::imu::Imu::Config MakeImuConfig() {
  hello_world::imu::Imu::Config config;
  config.sample_num = 1000;
  config.kp = 1.0f;
  config.ki = 0.0f;
  config.samp_freq = 1000.0f;

  for (size_t i = 0; i < 9; ++i) {
    config.rot_mat_flatten[i] = (i % 4 == 0) ? 1.0f : 0.0f;
  }

  config.bmi088_hw_config.hspi = &hspi1;
  config.bmi088_hw_config.acc_cs_port = GPIOA;
  config.bmi088_hw_config.acc_cs_pin = GPIO_PIN_4;
  config.bmi088_hw_config.gyro_cs_port = GPIOB;
  config.bmi088_hw_config.gyro_cs_pin = GPIO_PIN_0;
  return config;
}

}  // namespace
/* Exported function definitions ---------------------------------------------*/
hello_world::imu::Imu *CreateImu(void) {
  static hello_world::imu::Imu imu(MakeImuConfig());
  return &imu;
}
/* Private function definitions ----------------------------------------------*/
