#ifndef VIDEO_H
#define VIDEO_H

#include "types.h"

#define VGA_WIDTH 320
#define VGA_HEIGHT 200

enum {	VMODE_TEXT,
		VMODE_320x200_8BPP,
		VMODE_640x400_8BPP,
		VMODE_640x480_8BPP,
		VMODE_800x600_8BPP,
		VMODE_1024x768_8BPP,
		VMODE_1280x1024_8BPP,
		VMODE_320x200_16BPP,
		VMODES_NUM};

typedef struct vmode
{
	vmode(){};
	vmode(uint16 m, uint16 w, uint16 h, uint8 b, bool vs, uint8 *v) : 
		mode(m), width(w), height(h), bpp(b), vesa(vs), vram(v), buffer(0) {}

	uint16 mode;
	uint16 width;
	uint16 height;
	uint8 bpp;
	bool vesa;
	uint8 *vram;
	uint8 *buffer;
}vmode;

void initVideoModeInfo();
vmode *setVideoMode(uint16 width, uint16 height, uint8 bpp, bool needsBuffer = false);
void setTextMode();

void waitForVsync();
void clearFrame(vmode *vm);
void updateFrame(vmode *vm, bool vsync = false);
uint8 *getRenderBuffer(vmode *vm);

void setPalFromTab(uint8 colstart, uint8 *paltab, uint16 colnum);
void setSingleColorPal(uint8 color, uint8 r, uint8 g, uint8 b);
void setGradPal(int c0, int c1, int r0, int g0, int b0, int r1, int g1, int b1);

#endif
