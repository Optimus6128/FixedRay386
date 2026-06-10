#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "fx_ray.h"

#include "gfxtools.h"
#include "raytrace.h"


void fxRayInit(Screen *screen)
{
	makeAndSetPal(0, 255, 0, 0, 0, 63, 63, 63);
	makeAndSetPal(0, 63, 0, 0, 0, 63, 63, 63);
	makeAndSetPal(64, 127, 0, 0, 0, 63, 31, 15);
	makeAndSetPal(128, 191, 0, 0, 0, 15, 63, 31);
	makeAndSetPal(192, 255, 0, 0, 0, 15, 31, 63);

	raytraceInit0(screen);
	raytraceInit1();
}

void fxRayRun(Screen *screen, uint32 ticks, bool useFloat, int subdiv)
{
	if (useFloat) {
		raytraceRun0(screen, ticks);
	} else {
		raytraceRun1(screen, ticks, subdiv);
	}
}
