/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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
#include "ins_pid.hpp"

#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace {

hw_pid::Pid::Config MakePidConfig() {
  hw_pid::Pid::Config config;
  config.kp = 0.05f;              // 比例系数，先小后大
  config.ki = 0.20f;              // 积分系数，用于消除稳态误差
  config.kd = 0.00f;              // 速度反馈噪声大，微分先置 0
  config.integral_limit = 3.0f;   // 积分项限幅，单位 A
  config.output_limit = 3.0f;     // 输出限幅，GM6020 电流 ±3A
  config.dt = 0.001f;             // 控制周期 1ms
  return config;
}

}  // namespace
/* Exported function definitions ---------------------------------------------*/
hw_pid::Pid *CreatePidMotorRotate() {
  /* TODO【新人任务】:
   * 返回旋转电机（GM6020）速度环 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp / Ki / Kd、积分限幅、
   *      输出限幅），建议单独新建文件，类名放到 hello_world::pid 命名空间并
   *      命名为 Pid（或改这里的返回类型以匹配你的类名）。
   *   2. 增益自行整定（考核会检查稳 / 快 / 无超调）。
   *   3. 静态定义该 PID 对象并返回地址。
   *
   * 约束 / 提示：
   *   - 输出幅值：GM6020 电流模式建议限幅在 ±3A 量级。
   *   - 控制周期 1ms；积分项与限幅的单位请结合电机输入类型（电流 / A）确认。
   */
  static hw_pid::Pid pid(MakePidConfig());  // 旋转电机速度环 PID 单例
  return &pid;
}
/* Private function definitions ----------------------------------------------*/
