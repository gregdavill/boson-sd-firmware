#ifndef TIME_H__
#define TIME_H__

#include <time.h>

void time_isr();
void time_init(void);
void time_read(uint32_t* s, uint32_t* us);

#endif