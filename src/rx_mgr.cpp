/**
 *******************************************************************************
 * @file      : rx_mgr.cpp
 * @brief     : 通信接收管理器基类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  不建议用户使用及继承该类，而是使用其派生类
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "rx_mgr.hpp"

#include "allocator.hpp"
#include "assert.hpp"

namespace hello_world
{
namespace comm
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START
RxMgr &RxMgr::operator=(const RxMgr &other)
{
  if (this != &other) {
    MemMgr::operator=(other);
    receiver_ptr_list_ = other.receiver_ptr_list_;
  }

  return *this;
}

RxMgr::RxMgr(RxMgr &&other)
    : MemMgr(std::move(other)),
      receiver_ptr_list_(
          std::move(other.receiver_ptr_list_))
{
}

RxMgr &RxMgr::operator=(RxMgr &&other)
{
  if (this != &other) {
    MemMgr::operator=(std::move(other));
    receiver_ptr_list_ =
        std::move(other.receiver_ptr_list_);
  }

  return *this;
}

void RxMgr::addReceiver(Receiver *new_receiver_ptr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(new_receiver_ptr != nullptr, "new_receiver_ptr is nullptr");
#pragma endregion

  /* 查看接收端是否已存在，当不存在时再添加 */
  for (auto &it : receiver_ptr_list_) {
    if (it == new_receiver_ptr) {
      return;
    }
  }
  receiver_ptr_list_.push_back(new_receiver_ptr);
}

bool RxMgr::deleteReceiver(Receiver *receiver_ptr)
{
  for (auto it = receiver_ptr_list_.begin();
       it != receiver_ptr_list_.end(); ++it) {
    if (*it == receiver_ptr) {
      receiver_ptr_list_.erase(it);
      return true;
    }
  }

  return false;
}

void RxMgr::clearReceiver(void)
{
  receiver_ptr_list_.clear();
}

size_t RxMgr::decode(size_t len, const uint8_t *rx_buf, uint32_t rx_id)
{
  size_t cnt = 0;

  /* 遍历查找匹配的 ID */
  for (auto &receiver_ptr : receiver_ptr_list_) {
    for (auto &id : receiver_ptr->rxIds()) {
      if (id == rx_id) {
        /* 解包 */
        if (receiver_ptr->decode(len, rx_buf, rx_id)) {
          cnt++;
        }
      }
    }
  }
  return cnt;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world
