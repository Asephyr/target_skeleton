/**
 *******************************************************************************
 * @file      :td.cpp
 * @brief     : 一阶低通滤波器（速度反馈滤波）
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "td.hpp"

#include <cmath>

namespace hello_world {
namespace filter {

void Td::setConfig(const Config& config) {
  config_ = config;
  if (config_.dt > 0.0f && config_.cutoff_hz > 0.0f) {
    constexpr float kPi = 3.14159265358979323846f;
    alpha_ = 1.0f - expf(-2.0f * kPi * config_.cutoff_hz * config_.dt);
  } else {
    alpha_ = 1.0f;  // 参数非法时不滤波
  }
}

float Td::update(float input) {
  if (!initialized_) {
    output_ = input;  // 首次直接跟随输入，避免从 0 缓慢爬升
    initialized_ = true;
    return output_;
  }

  output_ += alpha_ * (input - output_);  // 一阶低通
  return output_;
}

void Td::reset() {
  output_ = 0.0f;        // 清空滤波输出
  initialized_ = false;  // 下次重新跟随输入
}

}  // namespace filter
}  // namespace hello_world
