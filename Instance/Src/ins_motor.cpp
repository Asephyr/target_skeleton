/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace {

constexpr uint8_t kLeftMotorId = 1;
constexpr uint8_t kRightMotorId = 2;
constexpr uint8_t kRotateMotorId = 1;
constexpr float kWheelExternalReduction = 268.0f / 17.0f;

hw_motor::OptionalParams MakeWheelParams(hw_motor::Dir dir) {
  hw_motor::OptionalParams opt;
  opt.input_type = hw_motor::InputType::kTorq;
  opt.angle_range = hw_motor::AngleRange::kNegInfToPosInf;
  opt.dir = dir;
  opt.remove_build_in_reducer = false;
  opt.ex_redu_rat = kWheelExternalReduction;
  opt.offline_tick_thres = 100;
  return opt;
}

hw_motor::OptionalParams MakeRotateParams() {
  hw_motor::OptionalParams opt;
  opt.input_type = hw_motor::InputType::kCurr;
  opt.angle_range = hw_motor::AngleRange::kNegInfToPosInf;
  opt.dir = hw_motor::kDirFwd;
  opt.ex_redu_rat = 1.0f;
  opt.offline_tick_thres = 100;
  return opt;
}

}  // namespace
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *CreateMotorLeft() {
  static hw_motor::M3508 motor(kLeftMotorId,
                               MakeWheelParams(hw_motor::kDirFwd));
  return &motor;
}

hello_world::motor::Motor *CreateMotorRight() {
  static hw_motor::M3508 motor(kRightMotorId,
                               MakeWheelParams(hw_motor::kDirRev));
  return &motor;
}

hello_world::motor::Motor *CreateMotorRotate() {
  static hw_motor::GM6020 motor(kRotateMotorId, MakeRotateParams());
  return &motor;
}
/* Private function definitions ----------------------------------------------*/
