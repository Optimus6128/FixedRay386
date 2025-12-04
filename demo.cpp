#include <stdio.h>
#include <conio.h>
#include <string.h>

#ifdef __DJGPP__
	#include <sys/nearptr.h>
#endif

#include "demo.h"
#include "video.h"
#include "timer.h"
#include "tinyfont.h"

#include "fx_ray.h"


static vmode *vm;
static bool vsync = false;

static uint32 nframe = 0;
static bool quit = false;

uint8 *VGAptr = (uint8*)0xA0000;
uint8 *TXTptr = (uint8*)0xB8000;
uint16 *my_clock = (uint16*)0x046C;

static Screen screen;

static void init(vmode *vm)
{
	screen.width = vm->width;
	screen.height = vm->height;
	screen.bpp = vm->bpp;
	screen.data = getRenderBuffer(vm);

	clearFrame(vm);

	initTinyFonts();
}

static void input()
{
	if (kbhit())
	{
		uint8 key = getch();
		switch(key)
		{
			case 27:	// Escape
				quit = true;
			break;

			case 'v':
				vsync = !vsync;
			break;
		}
	}
}

int main(int argc, char **argv)
{
#ifdef __DJGPP__
	__djgpp_nearptr_enable();

	VGAptr += __djgpp_conventional_base;
	TXTptr += __djgpp_conventional_base;
	my_clock = (uint16*)((uint8*)my_clock + __djgpp_conventional_base);
#endif

	initTimer();

	initVideoModeInfo();

	vm = setVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, true);
	if (vm==0) {
		printf("Video Mode not found\n");
		return 0;
	}

	init(vm);

	//setTextMode();	// for debugging

	fxRayInit(&screen);

	while(!quit)
	{
		input();

		fxRayRun(&screen, getTime());

		drawFps(&screen);

		updateFrame(vm, vsync);

		++nframe;
	}
	uint32 timeEnd = getTime();

	setTextMode();

#ifdef __DJGPP__
	__djgpp_nearptr_disable();
#endif

	deinitTimer();

	return 0;
}
