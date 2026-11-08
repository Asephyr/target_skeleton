/**
 *******************************************************************************
 * @file      :td.hpp
 * @brief     : 一阶低通滤波器（速度反馈滤波）
 * @attention : 截止频率越低，滤波越强，但相位滞后越大
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INSTANCE_ALGORITHM_TD_HPP_
#define INSTANCE_ALGORITHM_TD_HPP_

namespace hello_world {
namespace filter {

class Td {
 public:
  struct Config {
    float cutoff_hz = 50.0f;  // 截止频率，单位 Hz
    float dt = 0.001f;        // 采样周期，单位 s
  };

  Td() = default;
  explicit Td(const Config& config) { setConfig(config); }

  void setConfig(const Config& config);
  const Config& config() const { return config_; }

  /**
   * @brief  输入一个采样值，返回滤波后的值
   * @param  input: 原始输入
   * @retval 滤波后的输出
   */
  float update(float input);

  void reset();  // 清空滤波器状态

 private:
  Config config_{};          // 滤波器参数
  float alpha_ = 1.0f;       // 一阶低通系数
  float output_ = 0.0f;      // 上次滤波输出
  bool initialized_ = false; // 是否已经记录首次输入
};

}  // namespace filter
}  // namespace hello_world

#endif /* INSTANCE_ALGORITHM_TD_HPP_ */
