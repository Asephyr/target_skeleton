/**
 *******************************************************************************
 * @file      :pid.cpp
 * @brief     : 通用 PID 控制器
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "pid.hpp"

namespace hello_world {
namespace pid {
namespace {

float Clamp(float value, float limit) {
  if (limit <= 0.0f) {
    return value;  // limit <= 0 表示不做限幅
  }
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

}  // namespace

float Pid::update(float ref, float fdb) {
  if (!initialized_) {
    last_fdb_ = fdb;  // 首次记录反馈，避免微分冲击
    initialized_ = true;
  }

  const float error = ref - fdb;  // 目标误差

  if (config_.dt > 0.0f) {
    integral_ += error * config_.dt;  // 误差积分
  }

  float i_term = config_.ki * integral_;  // 积分项输出
  if (config_.integral_limit > 0.0f) {
    i_term = Clamp(i_term, config_.integral_limit);  // 积分项限幅
    if (config_.ki != 0.0f) {
      integral_ = i_term / config_.ki;  // 反算积分累加值，防止饱和
    }
  }

  float d_term = 0.0f;  // 微分项输出
  if (config_.dt > 0.0f) {
    d_term = -config_.kd * (fdb - last_fdb_) / config_.dt;  // 微分作用于反馈
  }

  output_ = config_.kp * error + i_term + d_term;  // PID 总输出
  output_ = Clamp(output_, config_.output_limit);  // 总输出限幅

  last_fdb_ = fdb;  // 保存本次反馈
  return output_;
}

void Pid::reset() {
  integral_ = 0.0f;      // 清空积分
  last_fdb_ = 0.0f;      // 清空微分记忆
  output_ = 0.0f;        // 清空输出
  initialized_ = false;  // 下次重新记录反馈
}

}  // namespace pid
}  // namespace hello_world
