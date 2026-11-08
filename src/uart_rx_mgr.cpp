/**
 *******************************************************************************
 * @file      : uart_rx_mgr.cpp
 * @brief     : UART 接收管理器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请在 STM32CubeMX 中开启 UART 的 DMA 发送（Circular），开启串口全局中断，同时
 *  尽可能保证 DMA 接收中断的 Preemption Priority 和 Sub Priority 值低于串口全局中断
 *  的 Preemption Priority 和 Sub Priority 值（即优先级更高），具体硬件配置详见 Wiki
 *  2. 由于内部使用了硬件句柄，因此如果计划将实例作为全局变量时（全局变量初始化时对应的
 *  硬件句柄可能会还未初始化完毕），建议采取一下方法：
 *    1）声明指针，后续通过 `new` 的方式进行初始化
 *    2）声明指针，后续通过返回函数（CreateXXXIns）中的静态变量（因为该变量只有在第一
 *    次调用该函数时才会运行初始化程序）进行初始化
 *    3）使用无参构造函数，后续调用 `init` 方法进行初始化
 *    4）使用无参构造函数，后续使用拷贝赋值函数或是移动赋值函数进行初始化
 *  3. 若不注册数据处理函数，则只有 ID 为 0 的接收器会收到数据
 *  4. H7 系列的 DMA 有工作区域要求，详见 Wiki 上的配置方法
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "uart_rx_mgr.hpp"

/* 开启 UART 与 DMA 才允许编译 */
#if defined(HAL_UART_MODULE_ENABLED) && defined(HAL_DMA_MODULE_ENABLED)

#include <cstring>

#include "allocator.hpp"
#include "assert.hpp"
#include "base.hpp"

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

UartRxMgr::UartRxMgr(UART_HandleTypeDef *huart, EofType eof_type,
                     size_t buf_len, size_t max_process_data_len)
    : RxMgr(),
      huart_(huart),
      eof_type_(eof_type),
      buf_len_(buf_len),
      max_process_data_len_(max_process_data_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(eof_type == EofType::kIdle || eof_type == EofType::kManual,
            "eof_type is invalid");
  HW_ASSERT(buf_len > 0, "buf_len <= 0");
  HW_ASSERT(max_process_data_len > 0, "max_process_data_len <= 0");
#pragma endregion

  rx_buf_ = Allocator<uint8_t>::allocate(buf_len_);
  rx_data_ = Allocator<uint8_t>::allocate(buf_len_);
  is_alloc_buf_ = true;
  processed_data_ = Allocator<uint8_t>::allocate(max_process_data_len_);

  memset(rx_buf_, 0, buf_len_);
  memset(processed_data_, 0, max_process_data_len_);
}

UartRxMgr::UartRxMgr(UART_HandleTypeDef *huart, EofType eof_type,
                     uint8_t *rx_buf, size_t buf_len,
                     size_t max_process_data_len)
    : RxMgr(),
      huart_(huart),
      eof_type_(eof_type),
      buf_len_(buf_len),
      max_process_data_len_(max_process_data_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(eof_type == EofType::kIdle || eof_type == EofType::kManual,
            "eof_type is invalid");
  HW_ASSERT(buf_len > 0, "buf_len <= 0");
  HW_ASSERT(max_process_data_len > 0, "max_process_data_len <= 0");
#pragma endregion

  rx_buf_ = rx_buf;
  rx_data_ = Allocator<uint8_t>::allocate(buf_len_);
  is_alloc_buf_ = false;
  processed_data_ = Allocator<uint8_t>::allocate(max_process_data_len_);

  memset(rx_buf_, 0, buf_len_);
  memset(processed_data_, 0, max_process_data_len_);
}

UartRxMgr &UartRxMgr::operator=(const UartRxMgr &other)
{
  if (this != &other) {
    RxMgr::operator=(other);

    huart_ = other.huart_;
    eof_type_ = other.eof_type_;
    buf_len_ = other.buf_len_;
    max_process_data_len_ = other.max_process_data_len_;
    rx_buf_ = other.rx_buf_;
    rx_data_ = other.rx_data_;
    processed_data_ = other.processed_data_;
    is_alloc_buf_ = other.is_alloc_buf_;
    process_data_func_ = other.process_data_func_;
    start_receive_ = other.start_receive_;
    buf_handled_idx_ = other.buf_handled_idx_;
    data_idx_ = other.data_idx_;
    last_rx_event_type_ = other.last_rx_event_type_;
    status_ = other.status_;
  }

  return *this;
}

UartRxMgr::UartRxMgr(UartRxMgr &&other) : RxMgr(std::move(other))
{
  huart_ = other.huart_;
  eof_type_ = other.eof_type_;
  buf_len_ = other.buf_len_;
  max_process_data_len_ = other.max_process_data_len_;
  rx_buf_ = other.rx_buf_;
  rx_data_ = other.rx_data_;
  processed_data_ = other.processed_data_;
  is_alloc_buf_ = other.is_alloc_buf_;
  process_data_func_ = other.process_data_func_;
  start_receive_ = other.start_receive_;
  buf_handled_idx_ = other.buf_handled_idx_;
  data_idx_ = other.data_idx_;
  last_rx_event_type_ = other.last_rx_event_type_;
  status_ = other.status_;

  other.huart_ = nullptr;
  other.rx_buf_ = nullptr;
  other.rx_data_ = nullptr;
  other.processed_data_ = nullptr;
  other.is_alloc_buf_ = false;
  process_data_func_ = ProcessData;
}

UartRxMgr &UartRxMgr::operator=(UartRxMgr &&other)
{
  if (this != &other) {
    RxMgr::operator=(std::move(other));

    huart_ = other.huart_;
    eof_type_ = other.eof_type_;
    buf_len_ = other.buf_len_;
    max_process_data_len_ = other.max_process_data_len_;
    rx_buf_ = other.rx_buf_;
    rx_data_ = other.rx_data_;
    processed_data_ = other.processed_data_;
    is_alloc_buf_ = other.is_alloc_buf_;
    process_data_func_ = other.process_data_func_;
    start_receive_ = other.start_receive_;
    buf_handled_idx_ = other.buf_handled_idx_;
    data_idx_ = other.data_idx_;
    last_rx_event_type_ = other.last_rx_event_type_;
    status_ = other.status_;

    other.huart_ = nullptr;
    other.rx_buf_ = nullptr;
    other.rx_data_ = nullptr;
    other.processed_data_ = nullptr;
    other.is_alloc_buf_ = false;
    other.process_data_func_ = ProcessData;
  }

  return *this;
}

UartRxMgr::~UartRxMgr(void)
{
  if (is_alloc_buf_) {
    Allocator<uint8_t>::deallocate(rx_buf_, buf_len_);
  }
  Allocator<uint8_t>::deallocate(rx_data_, buf_len_);
  Allocator<uint8_t>::deallocate(processed_data_, max_process_data_len_);
}

void UartRxMgr::startReceive(void)
{
  start_receive_ = true;

  HAL_StatusTypeDef status =
      HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buf_, buf_len_);
  if (status == HAL_OK) {
    buf_handled_idx_ = 0;
    last_rx_event_type_ = HAL_UART_RXEVENT_TC;
  } else if (status == HAL_ERROR) {
    SetBits(kUartRxStatusRecvStartErr, status_);
  }
}

void UartRxMgr::stopReceive(void)
{
  start_receive_ = false;
  ClearBits(kUartRxStatusProcessingData, status_);

  HAL_UART_AbortReceive_IT(huart_);
}

void UartRxMgr::init(
    UART_HandleTypeDef *huart, EofType eof_type, size_t buf_len,
    size_t max_process_data_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(eof_type == EofType::kIdle || eof_type == EofType::kManual,
            "eof_type is invalid");
  HW_ASSERT(buf_len > 0, "buf_len <= 0");
  HW_ASSERT(max_process_data_len > 0, "max_process_data_len <= 0");
#pragma endregion

  if (is_alloc_buf_) {
    Allocator<uint8_t>::deallocate(rx_buf_, buf_len_);
  }

  huart_ = huart;
  eof_type_ = eof_type;
  buf_len_ = buf_len;
  max_process_data_len_ = max_process_data_len;

  rx_buf_ = Allocator<uint8_t>::allocate(buf_len_);
  rx_data_ = Allocator<uint8_t>::allocate(buf_len_);
  processed_data_ = Allocator<uint8_t>::allocate(max_process_data_len_);
  is_alloc_buf_ = true;

  memset(rx_buf_, 0, buf_len_);
  memset(processed_data_, 0, max_process_data_len_);

  status_ = kUartRxStatusOk;
  start_receive_ = false;
  decode_success_cnt_ = 0;
  last_rx_event_type_ = HAL_UART_RXEVENT_TC;

  data_idx_ = 0;
  buf_handled_idx_ = 0;

  process_data_func_ = ProcessData;
}

void UartRxMgr::init(
    UART_HandleTypeDef *huart, EofType eof_type, uint8_t *rx_buf,
    size_t buf_len, size_t max_process_data_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(eof_type == EofType::kIdle || eof_type == EofType::kManual,
            "eof_type is invalid");
  HW_ASSERT(buf_len > 0, "buf_len <= 0");
  HW_ASSERT(max_process_data_len > 0, "max_process_data_len <= 0");
#pragma endregion

  if (is_alloc_buf_) {
    Allocator<uint8_t>::deallocate(rx_buf_, buf_len_);
  }

  huart_ = huart;
  eof_type_ = eof_type;
  buf_len_ = buf_len;
  max_process_data_len_ = max_process_data_len;

  rx_buf_ = rx_buf;
  rx_data_ = Allocator<uint8_t>::allocate(buf_len_);
  processed_data_ = Allocator<uint8_t>::allocate(max_process_data_len_);
  is_alloc_buf_ = false;

  memset(rx_buf_, 0, buf_len_);
  memset(processed_data_, 0, max_process_data_len_);

  status_ = kUartRxStatusOk;
  start_receive_ = false;
  decode_success_cnt_ = 0;
  last_rx_event_type_ = HAL_UART_RXEVENT_TC;

  data_idx_ = 0;
  buf_handled_idx_ = 0;

  process_data_func_ = ProcessData;
}

void UartRxMgr::registerProcessDataFunc(pProcessData func)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(func != nullptr, "func is nullptr");
#pragma endregion

  process_data_func_ = func;
}

void UartRxMgr::rxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
#pragma endregion

  /* 避免重入 */
  if (huart != huart_ || !start_receive_ ||
      IsBitsSet(kUartRxStatusProcessingData, status_)) {
    return;
  }

  SetBits(kUartRxStatusProcessingData, status_);

  /* 获取实际接收到的数据 */
  size_t rx_data_len = 0;
  if (!getData(size, rx_data_len)) {
    last_rx_event_type_ = HAL_UARTEx_GetRxEventType(huart_);
    ClearBits(kUartRxStatusProcessingData, status_);
    return;
  }

  receive_cnt_++;

  /* 数据处理 */
  size_t rx_data_processed_len = 0;
  do {
    uint32_t id = 0;
    size_t processed_data_len = max_process_data_len_;
    rx_data_len -= rx_data_processed_len;

    /* 一次只处理一个包的数据 */
    if (!process_data_func_(
            rx_data_ + rx_data_processed_len, rx_data_len,
            processed_data_len, processed_data_, rx_data_processed_len, id)) {
      break;
    }

    /* 数据解包 */
    decode_success_cnt_ += decode(processed_data_len, processed_data_, id);
  } while (rx_data_processed_len < rx_data_len);

  last_rx_event_type_ = HAL_UARTEx_GetRxEventType(huart_);
  ClearBits(kUartRxStatusProcessingData, status_);
}

void UartRxMgr::errorCallback(UART_HandleTypeDef *huart)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
#pragma endregion

  if (huart != huart_ || !start_receive_) {
    return;
  }

  startReceive();
}

bool UartRxMgr::getData(uint16_t size, size_t &data_len)
{
  /* 避免数据重复处理 */
  if (size < buf_handled_idx_ &&
      last_rx_event_type_ != HAL_UART_RXEVENT_TC) {
    return false;
  }

  /* 取出缓冲区中未处理的数据 */
  size_t i = buf_handled_idx_;
  while (i != size) {
    if (i >= buf_len_) {
      i -= buf_len_;
    }

    if (data_idx_ < buf_len_) {
      rx_data_[data_idx_++] = rx_buf_[i];
    } else {
      SetBits(kUartRxStatusDataOverflow, status_);
      break;
    }

    i++;
  }
  buf_handled_idx_ = size;

  if (eof_type_ == EofType::kIdle) {
    /* 当为空闲帧触发时认为一帧接收完毕 */
    if (HAL_UARTEx_GetRxEventType(huart_) != HAL_UART_RXEVENT_IDLE) {
      return false;
    }

    /* 重置 DMA 接收 */
    __HAL_DMA_DISABLE(huart_->hdmarx);
    __HAL_DMA_SET_COUNTER(huart_->hdmarx, buf_len_);
    __HAL_DMA_ENABLE(huart_->hdmarx);
    __HAL_DMA_CLEAR_FLAG(
        huart_->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart_->hdmarx));

    /* 空闲作为一帧结束 */
    data_len = data_idx_;
    buf_handled_idx_ = 0;
    data_idx_ = 0;  // 接收重置
  } else {
    /* 自行判断则直接返回 */
    data_len = data_idx_;
    data_idx_ = 0;  // 接收重置
  }

  if (data_len == 0) {
    return false;
  } else {
    return true;
  }
}

bool UartRxMgr::ProcessData(
    const uint8_t *rx_data, size_t rx_data_len, size_t &processed_data_len,
    uint8_t *processed_data, size_t &rx_data_processed_len, uint32_t &id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(rx_data != nullptr, "rx_data is nullptr");
  HW_ASSERT(rx_data_len > 0, "rx_data_len <= 0");
  HW_ASSERT(processed_data != nullptr, "processed_data is nullptr");
  HW_ASSERT(processed_data_len > 0, "*processed_data_len <= 0");
#pragma endregion

  /* 避免数据溢出 */
  processed_data_len = std::min(rx_data_len, processed_data_len);
  memcpy(processed_data, rx_data, processed_data_len);

  rx_data_processed_len = processed_data_len;
  id = 0;

  return true;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_UART_MODULE_ENABLED && HAL_DMA_MODULE_ENABLED */