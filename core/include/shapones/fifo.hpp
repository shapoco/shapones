#ifndef SHAPONES_FIFO_HPP
#define SHAPONES_FIFO_HPP

#include "shapones/common.hpp"

namespace nes {

template <typename T, uint32_t prm_ADDR_BITS>
class AsyncFifo {
 public:
  static constexpr uint32_t ADDR_BITS = prm_ADDR_BITS;
  static constexpr uint32_t CAPACITY = 1 << ADDR_BITS;

 private:
  T buffer[CAPACITY];
  volatile uint32_t rd_ptr;
  volatile uint32_t wr_ptr;

 public:
  AsyncFifo() : rd_ptr(0), wr_ptr(0) {}

  void clear() {
    rd_ptr = 0;
    wr_ptr = 0;
  }

  SHAPONES_INLINE bool is_empty() const { return rd_ptr == wr_ptr; }

  SHAPONES_INLINE bool is_full() const {
    uint32_t wp = (wr_ptr + 1) & (CAPACITY - 1);
    return wp == rd_ptr;
  }

  SHAPONES_INLINE bool try_push(const T &item) {
    uint32_t wp = wr_ptr;
    uint32_t wp_next = (wp + 1) & (CAPACITY - 1);
    if (wp_next == rd_ptr) {
      return false;  // full
    }
    buffer[wp] = item;
    SHAPONES_THREAD_FENCE_SEQ_CST();
    wr_ptr = wp_next;
    return true;
  }

  SHAPONES_INLINE void push_blocking(const T &item) {
    uint32_t wp, wp_next;
    do {
      wp = wr_ptr;
      wp_next = (wp + 1) & (CAPACITY - 1);
    } while (wp_next == rd_ptr);  // wait until not full
    SHAPONES_THREAD_FENCE_SEQ_CST();
    buffer[wp] = item;
    wr_ptr = wp_next;
  }

  SHAPONES_INLINE bool try_peek(T *out_item) {
    uint32_t rp = rd_ptr;
    if (rp == wr_ptr) {
      return false;  // empty
    }
    *out_item = buffer[rp];
    return true;
  }

  SHAPONES_INLINE bool try_pop(T *out_item) {
    uint32_t rp = rd_ptr;
    if (rp == wr_ptr) {
      return false;  // empty
    }
    *out_item = buffer[rp];
    SHAPONES_THREAD_FENCE_SEQ_CST();
    rd_ptr = (rp + 1) & (CAPACITY - 1);
    return true;
  }

  SHAPONES_INLINE T pop_blocking() {
    uint32_t rp;
    do {
      rp = rd_ptr;
    } while (rp == wr_ptr);  // wait until not empty
    T item = buffer[rp];
    SHAPONES_THREAD_FENCE_SEQ_CST();
    rd_ptr = (rp + 1) & (CAPACITY - 1);
    return item;
  }
};
}  // namespace nes

#endif
