/**
 *******************************************************************************
 * @file      :pid.hpp
 * @brief     : 通用 PID 控制器
 * @attention : 速度环建议先按 PI 调，Kd 先置 0
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INSTANCE_ALGORITHM_PID_HPP_
#define INSTANCE_ALGORITHM_PID_HPP_

namespace hello_world {
namespace pid {

class Pid {
 public:
  struct Config {
    float kp = 0.0f;              // 比例系数，决定响应速度
    float ki = 0.0f;              // 积分系数，用于消除稳态误差
    float kd = 0.0f;              // 微分系数，用于抑制超调
    float integral_limit = 0.0f;  // 积分项限幅，单位与输出相同(A)
    float output_limit = 0.0f;    // 总输出限幅，单位与输出相同(A)
    float dt = 0.001f;            // 控制周期，单位 s
  };

  Pid() = default;
  explicit Pid(const Config& config) : config_(config) {}

  void setConfig(const Config& config) { config_ = config; }
  const Config& config() const { return config_; }

  /**
   * @brief  计算一次 PID 输出
   * @param  ref: 目标值
   * @param  fdb: 反馈值
   * @retval PID 输出
   */
  float update(float ref, float fdb);

  void reset();  // 清空积分、微分和输出状态

 private:
  Config config_{};            // PID 参数
  float integral_ = 0.0f;      // 误差积分累加值
  float last_fdb_ = 0.0f;      // 上次反馈值，用于微分
  float output_ = 0.0f;        // 上次输出值
  bool initialized_ = false;   // 是否已经记录首次反馈
};

}  // namespace pid
}  // namespace hello_world

#endif /* INSTANCE_ALGORITHM_PID_HPP_ */
