// Host stub of the few Zephyr kernel calls the sketch uses
#pragma once
#include <cstdint>
#include <cstddef>
struct k_timer  { int dummy; };
struct k_thread { int dummy; };
typedef struct { int dummy; } k_thread_stack_t;
typedef struct k_thread* k_tid_t;
typedef struct { int64_t us; } k_timeout_t;
#define K_USEC(x)  (k_timeout_t{ (int64_t)(x) })
#define K_NO_WAIT  (k_timeout_t{ 0 })
uint32_t k_cycle_get_32();
inline int  sys_clock_hw_cycles_per_sec() { return 160000000; }
inline void k_timer_init(struct k_timer*, void*, void*) {}
inline void k_timer_start(struct k_timer*, k_timeout_t, k_timeout_t) {}
inline uint32_t k_timer_status_sync(struct k_timer*) { return 1; }
k_thread_stack_t* k_thread_stack_alloc(size_t, int);
k_tid_t k_thread_create(struct k_thread*, k_thread_stack_t*, size_t, void (*)(void*, void*, void*),
                        void*, void*, void*, int, uint32_t, k_timeout_t);
