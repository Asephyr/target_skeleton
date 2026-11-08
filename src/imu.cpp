/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : IMU 设备组件
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-12-01      Jinletian       1. 初版编写完成
 *******************************************************************************
 * @attention :
 *  1.update前，需要先调用initHardware()，否则update会失败，
 *    该函数内部会阻塞运行，请不要在中断中调用
 *  2.ImuConfig中大部分参数给出了默认值，可根据实际需要修改
 *    初始旋转矩阵rot_mat_ptr和硬件配置bmi088_hw_config需要用户自行配置
 *  3.计算传感器零漂时，姿态角会根据当前零漂计算值持续更新
 *  4.如果要用默认零漂值，则将零漂采样次数sample_num设为0
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

/* 开启 SPI 与 GPIO 才允许编译 */
#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

#include "arm_math.h"
#include "assert.hpp"
#include "base.hpp"

namespace hello_world
{
namespace imu
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void Imu::getRawData()
{
  HW_ASSERT(bmi088_ptr_ != nullptr, "BMI088 pointer is nullptr");
  bmi088_ptr_->getData(raw_acc_.data, raw_gyro_.data, &temp_);
};

void Imu::calcOffset()
{
  if (config_.sample_num == 0) {  // 采用默认值，不实时计算零漂
    memcpy(&gyro_offset_, config_.default_gyro_offset, sizeof(gyro_offset_));
    status_ = Status::kWorking;
    return;
  }
  if (sample_cnt_ < config_.sample_num) {
    bool is_gyro_overflow = false;
    float threshold = config_.gyro_stationary_threshold;
    for (size_t i = 0; i < 3; i++) {
      /* 如果角速度超过阈值，舍弃此次数据，不计入采样次数 */
      if (!IsInRange(raw_gyro_.data[i], -threshold, threshold)) {
        is_gyro_overflow = true;
        break;
      }
      /* 递推平均值 */
      gyro_offset_.data[i] += ((raw_gyro_.data[i] - gyro_offset_.data[i]) / (float)(sample_cnt_ + 1));
    }
    if (is_gyro_overflow == false) {
      sample_cnt_++;
    }
  } else {
    status_ = Status::kWorking;
  }
};

void Imu::updateAccGyro()
{
  for (size_t i = 0; i < 3; i++) {
    /* 加速度过滤短时撞击 */
    acc_.data[i] = Bound(raw_acc_.data[i], -config_.acc_threshold, config_.acc_threshold);

    /* 角速度去零漂 */
    last_gyro_.data[i] = gyro_.data[i];
    gyro_.data[i] = raw_gyro_.data[i] - gyro_offset_.data[i];

    /* 角加速度计算 */
    ang_acc_.data[i] =
        config_.ang_acc_beta * ang_acc_.data[i] +
        (1 - config_.ang_acc_beta) * (gyro_.data[i] - last_gyro_.data[i]) * config_.samp_freq;
  }
};

void Imu::updateMahony()
{
  HW_ASSERT(mahony_ptr_ != nullptr, "Mahony pointer is nullptr");

  if (acc_enabled_) {
    mahony_ptr_->update(acc_.data, gyro_.data);
  } else {
    /* 如果不使用加速度计，则只用角速度更新姿态 */
    static float zero_acc[3] = {0, 0, 0};
    mahony_ptr_->update(zero_acc, gyro_.data);
  }

  mahony_ptr_->getEulerAngle(ang_.data);
};

/* Exported function definitions ---------------------------------------------*/

void Imu::initHardware()
{
  HW_ASSERT(config_.acc_threshold > 0, "Acceleration threshold must be greater than 0");
  HW_ASSERT(config_.gyro_stationary_threshold > 0, "Gyro stationary threshold must be greater than 0");

  bmi088_ptr_ = new BMI088(config_.bmi088_hw_config, config_.rot_mat_flatten, config_.bmi088_config);

  while (bmi088_ptr_->imuInit() != BMI088ErrState::kBMI088ErrStateNoErr) {
    status_ = Status::kHardwareNotInited;
  }

  float quat_init[4] = {0};
  if (config_.enable_calc_init_quat) {
    while (raw_acc_.data[0] == 0 && raw_acc_.data[1] == 0 && raw_acc_.data[2] == 0) {
      getRawData();
    }
    calcInitQuat(raw_acc_.data, quat_init);
  } else {
    /* 使用默认四元数 */
    memcpy(quat_init, config_.quat_init, sizeof(config_.quat_init));
  }
  mahony_ptr_ = new Mahony(quat_init, config_.samp_freq, config_.kp, config_.ki);
  status_ = Status::kCalcingOffset;
};

bool Imu::update()
{
  if (status_ == Status::kHardwareNotInited) {
    return false;
  }
  /* 获取IMU原始数据 */
  getRawData();

  if (status_ == Status::kCalcingOffset) {
    calcOffset();
  }

  /* 数据处理 */
  updateAccGyro();

  /* 更新姿态角 */
  updateMahony();
  return true;
};

void Imu::getQuat(float quat[4]) const
{
  HW_ASSERT(mahony_ptr_ != nullptr, "Mahony pointer is nullptr");
  mahony_ptr_->getQuat(quat);
};

void Imu::calcInitQuat(const float acc_data[3], float quat[4])
{
  float32_t tmp;
  float32_t roll, pitch;
  float32_t sin_r2, cos_r2, sin_p2, cos_p2;

  arm_atan2_f32(acc_data[1], acc_data[2], &roll);
  arm_sqrt_f32(acc_data[1] * acc_data[1] + acc_data[2] * acc_data[2], &tmp);
  arm_atan2_f32(-acc_data[0], tmp, &pitch);

  arm_sin_cos_f32(Rad2Deg(0.5f * roll), &sin_r2, &cos_r2);
  arm_sin_cos_f32(Rad2Deg(0.5f * pitch), &sin_p2, &cos_p2);

  quat[0] = cos_r2 * cos_p2;
  quat[1] = sin_r2 * cos_p2;
  quat[2] = cos_r2 * sin_p2;
  quat[3] = 0.0f;
};

}  // namespace imu
}  // namespace hello_world

#endif /* HAL_SPI_MODULE_ENABLED && HAL_GPIO_MODULE_ENABLED */