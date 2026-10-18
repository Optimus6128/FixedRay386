#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "fx_ray.h"

#include "gfxtools.h"
#include "raytrace.h"


void fxRayInit(Screen *screen, bool useFloat)
{
	makeAndSetPal(0, 255, 0, 0, 0, 63, 63, 63);

	makeAndSetPal(0, 63, 0, 0, 0, 52, 23, 63);
	makeAndSetPal(64, 127-SPEC_COLS-1, 0, 0, 0, 53, 31, 23); makeAndSetPal(127-SPEC_COLS, 127, 53, 31, 23, 53, 43, 63);
	makeAndSetPal(128, 191-SPEC_COLS-1, 0, 0, 0, 15, 43, 31); makeAndSetPal(191-SPEC_COLS, 191, 15, 43, 31, 53, 43, 63);
	makeAndSetPal(192, 255-SPEC_COLS-1, 0, 0, 0, 23, 31, 63); makeAndSetPal(255-SPEC_COLS, 255, 23, 31, 63, 53, 43, 63);

	if (useFloat) {
		raytraceInit0(screen);
	} else {
		raytraceInit1();
	}
}

void fxRayRun(Screen *screen, uint32 ticks, bool useFloat, int subdiv)
{
	if (useFloat) {
		raytraceRun0(screen, ticks);
	} else {
		raytraceRun1(screen, ticks, subdiv);
	}
}
