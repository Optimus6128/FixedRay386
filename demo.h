#ifndef DEMO_H
#define DEMO_H

#include "types.h"

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480
#define SCREEN_BPP 8

typedef struct Screen
{
	int width, height, bpp;
	void *data;
} Screen;


extern uint8 *VGAptr;
extern uint8 *TXTptr;
extern uint16 *my_clock;

void requestQuit();

#endif
