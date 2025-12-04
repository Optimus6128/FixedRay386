#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "fx_plasm.h"
#include "video.h"
#include "gfxtools.h"


void fxRayInit(Screen *screen)
{

	makeAndSetPal(0, 255, 0, 0, 0, 63, 63, 63);
}

void fxRayRun(Screen *screen, uint32 ticks)
{
}
