#ifndef TIMER_H
#define TIMER_H

#include "types.h"
#include "demo.h"

//#define TIME_FREQ 18.2f
#define TIME_FREQ 1000.0f

void initTimer();
void deinitTimer();

uint32 getTime();
void drawFps(Screen *screen);

#endif
