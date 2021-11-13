#ifndef TIMER_H__
#define TIMER_H__

#include <stdint.h>

void timer_isr(void);
void timer_init(void);
void timer_read(uint32_t* s, uint32_t* us);

#endif