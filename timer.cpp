#include "timer.h"

#include <string.h>
#include <conio.h>
#include <dos.h>
#include <stdlib.h>

#include "tinyfont.h"
#include "demo.h"
#include "types.h"

static uint32 startingFpsTime = 0;
static int fpsFontsInit = 0;
static int nFrames = 0;

#define TIMER_INTERRUPT 0x08

// Taken and modified from https://expiredpopsicle.com/articles/2017-04-13-DOS_Timer_Stuff/2017-04-13-DOS_Timer_Stuff.html
// Pragma AUX don't work for me in OpenWatcom? Inlined assembly in functions for single CLI/STI, oof. But ok as we call these very rarely.

/*
#pragma aux timerClearInterrupt =		\
	"mov al,20H",		\
	"out 20H,al"

#pragma aux doCli =		\
	"cli"

#pragma aux doSti =		\
	"sti"
*/

void timerClearInterrupt()
{
	#ifdef __DJGPP__
		outportb(0x20, 0x20);
	#else
		_asm
		{
			mov al,20h
			out 20h,al
		}
	#endif
}

void doCli()
{
	#ifdef __DJGPP__
		// TODO
	#else
		_asm
		{
			cli
		}
	#endif
}

void doSti()
{
	#ifdef __DJGPP__
		// TODO
	#else
		_asm
		{
			sti
		}
	#endif
}

static uint32 timeValue = 0;
static int32 nextOldTimer = 0;
static uint32 timerInitCounter = 0;
static bool timerInterruptInit = false;

static void (__interrupt __far *oldDosTimerInterrupt)();

static void __interrupt __far newTimerInterrupt()
{
	timeValue++;

	nextOldTimer -= 10;
	if(nextOldTimer <= 0) {
	nextOldTimer += 182;
	oldDosTimerInterrupt();
	} else {
	// Make sure we still execute the "HEY I'M DONE WITH THIS
	// INTERRUPT" signal.
	timerClearInterrupt();
	}
}

static void timerInterruptStart()
{
	if (timerInterruptInit) return;

	// The clock we're dealing with here runs at 1.193182mhz, so we
	// just divide 1.193182 by the number of triggers we want per
	// second to get our divisor.
	uint32 c = 1193181 / (uint32)1000;

	// Swap out interrupt handlers.
	oldDosTimerInterrupt = _dos_getvect(TIMER_INTERRUPT);
	_dos_setvect(TIMER_INTERRUPT, newTimerInterrupt);

	doCli();

	// There's a ton of options encoded into this one byte I'm going
	// to send to the PIT here so...

	// 0x34 = 0011 0100 in binary.

	// 00  = Select counter 0 (counter divisor)
	// 11  = Command to read/write counter bits (low byte, then high
	//		 byte, in sequence).
	// 010 = Mode 2 - rate generator.
	// 0   = Binary counter 16 bits (instead of BCD counter).

	outp(0x43, 0x34);

	// Set divisor low byte.
	outp(0x40, (uint8)(c & 0xff));

	// Set divisor high byte.
	outp(0x40, (uint8)((c >> 8) & 0xff));

	doSti();

	timerInterruptInit = true;
}

static void timerInterruptEnd()
{
	if (!timerInterruptInit) return;

	doCli();

	// Send the same command we sent in timer_init() just so we can
	// set the timer divisor back.
	outp(0x43, 0x34);

	// FIXME: I guess giving zero here resets it? Not sure about this.
	// Maybe we should save the timer values first.
	outp(0x40, 0);
	outp(0x40, 0);

	doSti();

	// Restore original timer interrupt handler.
	_dos_setvect(TIMER_INTERRUPT, oldDosTimerInterrupt);

	timerInterruptInit = false;
}

uint32 getTime()
{
	return timeValue;
}

void drawFps(Screen *screen)
{
	uint32 dt = getTime() - startingFpsTime;
	static int fps = 0;
	const int secsTime = 1000;

	nFrames++;
	if (dt >= secsTime)
	{
		fps = (nFrames * secsTime) / dt;
		startingFpsTime = getTime();
		nFrames = 0;
	}
	drawNumber(8, 8, fps, screen);
}

void initTimer()
{
	timerInterruptStart();
}

void deinitTimer()
{
	timerInterruptEnd();
}
