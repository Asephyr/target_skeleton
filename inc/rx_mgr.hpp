/**
 *******************************************************************************
 * @file      : rx_mgr.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_RX_MGR_HPP_
#define HW_COMPONENTS_BSP_COMM_RX_MGR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "list.hpp"
#include "receiver.hpp"
#include "system.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
/**
 * @brief       通信接收管理器基类
 * @note        不建议用户使用及继承该类，而是使用其派生类
 */
class RxMgr : public MemMgr
{
 public:
  RxMgr(void) = default;
  RxMgr(const RxMgr &) = default;
  RxMgr &operator=(const RxMgr &other);
  RxMgr(RxMgr &&other);
  RxMgr &operator=(RxMgr &&other);

  virtual ~RxMgr(void) = default;

  /* 功能性方法 */

  /**
   * @brief       开启接收
   * @retval       None
   * @note        仅需调用一次
   */
  virtual void startReceive(void) = 0;

  virtual void stopReceive(void) = 0;

  /**
   * @brief       添加接收端
   * @param        new_receiver_ptr: 接收端指针，在运行期间不能被释放，建议为静态变
   *               量或全局变量
   * @retval       None
   * @note        已有的接收端不会被重复添加
   */
  void addReceiver(Receiver *new_receiver_ptr);

  /**
   * @brief       根据接收 ID 移除接收端
   * @param        rx_id: 接收 ID
   * @retval       移除成功返回 true，否则返回 false
   * @note        不建议在运行过程中调用
   */
  bool deleteReceiver(Receiver *receiver_ptr);

  /**
   * @brief       清除接收端
   * @retval       None
   * @note        不建议在运行过程中调用
   */
  void clearReceiver(void);

 protected:
  typedef tools::list<Receiver*> ReceiverPtrList;
  /* 功能性方法 */

  /**
   * @brief       解码，依次调用对应 ID 的接收端的 decode 方法
   * @param        len: 数据长度
   * @param        rx_buf: 数据接收缓冲区
   * @param        rx_id: 接收端 ID
   * @retval       返回成功解包的接收端数量
   * @note        None
   */
  size_t decode(size_t len, const uint8_t *rx_buf, uint32_t rx_id);

  ReceiverPtrList receiver_ptr_list_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /*  HW_COMPONENTS_BSP_COMM_RX_MGR_HPP_*/
