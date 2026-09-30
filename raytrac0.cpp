#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#include "types.h"
#include "gfxtools.h"
#include "raytrace.h"
#include "vector.h"
#include "mathutil.h"


#define FAR_F 1000000000.0f

typedef struct Object
{
	int type;
	float reflectivity;
	Vec3 pos, dir1, dir2;
	float radius;
	float a, b, c, d;
	int index;
} Object;

typedef struct Hit
{
	Object* obj;
	Vec3 pos, normal, reflectNormal;
} Hit;

typedef struct PerPixelPrecs
{
	Vec3* viewDir;
	float* dirDot;
} PerPixelPrecs;

#define CYBER_TEX_W 64
#define CYBER_TEX_H 64

#define CLOUD_TEX_W 128
#define CLOUD_TEX_H 128

static uint8 floorTex[CYBER_TEX_W * CYBER_TEX_H];
static uint8 ceilingTex[CLOUD_TEX_W * CLOUD_TEX_H];


#define NUM_PLANES 2
#define NUM_SPHERES 3

static Object plane[NUM_PLANES];
static Object sphere[NUM_SPHERES];
static Vec3 lightDir;

static PerPixelPrecs perPixelPrecs;

static void setupViewDirPerPixelLUTs(Screen* buffer)
{
    const int screenWidth = buffer->width;
    const int screenHeight = buffer->height;
    const float aspectRatio = (float)screenHeight / (float)screenWidth;

    const float Xinc = 2.0f / (screenWidth - 1);
    const float Yinc = (2.0f / (screenHeight - 1)) * aspectRatio;
	const float distFromCamera = 1.0f;	// maybe this doesn't need to be high (Xinc,Yinc left right are also normalized)
										// For a moment I thought when I removed viewer pos at -Z something, that it would be some stupid pinhole distorted view from 0,0,0 stretch over the pixelres and maybe it was a mistake to simplify the old, but let's see
										// EDIT: we can control the FOV from here. change 1.0f to 0.5f see what happens or to 1.5f

	Vec3* dir = perPixelPrecs.viewDir;
	float* dirDot = perPixelPrecs.dirDot;

    float Ydir = (screenHeight / 2) * Yinc;
    for (int y = 0; y < screenHeight; y++) {
        float Xdir = (-screenWidth / 2) * Xinc;
        for (int x = 0; x < screenWidth; x++) {
			dir->x = Xdir;
			dir->y = Ydir;
			dir->z = distFromCamera;

			Vnormalize(dir);

			*dirDot++ = 2.0f * (dir->x * dir->x + dir->y * dir->y + dir->z * dir->z);
			dir++;
            Xdir += Xinc;
        }
        Ydir -= Yinc;
    }
}

static void calcABCD(Object* obj)
{
	Vec3 v;
	switch (obj->type) {

		case OBJ_PLANE:
			Vcross(&obj->dir1, &obj->dir2, &v);
			Vnormalize(&v);
			obj->a = v.x;
			obj->b = v.y;
			obj->c = v.z;
			obj->d = obj->pos.y;
			break;

		case OBJ_SPHERE:
			obj->a = obj->pos.x;
			obj->b = obj->pos.y;
			obj->c = obj->pos.z;
			obj->d = 2.0f * (obj->pos.x * obj->pos.x + obj->pos.y * obj->pos.y + obj->pos.z * obj->pos.z - obj->radius * obj->radius);
			break;

		default:
			break;
	}
}

static inline float intersectPlane2(Object *obj, Vec3 *pos, Vec3 *dir)
{
	const float denom = obj->a * dir->x + obj->b * dir->y + obj->c * dir->z;

	if (denom != 0) {
		return -(obj->a * pos->x + obj->b * pos->y + obj->c * pos->z + obj->d) / denom;
	}
	return -1;
}

static inline float intersectSphere2(Object *obj, Vec3 *pos, Vec3 *dir)
{
	const float a = dir->x * dir->x + dir->y * dir->y + dir->z * dir->z;
	const float b = 2 * (dir->x * (pos->x - obj->pos.x) + dir->y * (pos->y - obj->pos.y) + dir->z * (pos->z - obj->pos.z));
	const float c = (pos->x - obj->pos.x) * (pos->x - obj->pos.x) + (pos->y - obj->pos.y) * (pos->y - obj->pos.y) + (pos->z - obj->pos.z) * (pos->z - obj->pos.z) - obj->radius * obj->radius;

	float d = b * b - 4 * a * c;
	if (d >= 0)
	{
		d = sqrt(d);

		float t1 = (-b + d) / (2 * a);
		float t2 = (-b - d) / (2 * a);

		if (t1 <= 0 && t2 <= 0) {
			return -1;
		}

		if (t1 < t2) {
			if (t1 > 0) {
				return t1;
			} else {
				return t2;
			}
		} else {
			if (t2 > 0) {
				return t2;
			} else {
				return t1;
			}
		}
	}
	return -1;
}

static inline float intersectPlane(Object* obj, Vec3* dir)
{
	const float denom = obj->a * dir->x + obj->b * dir->y + obj->c * dir->z;
	if (denom != 0) {
		return -obj->d / denom;
	}
	return -1;
}

static inline float intersectSphere(Object* obj, Vec3* dir, float dirDot)
{
	if (dirDot == 0.0f) return -1;

	const float b = 2 * (dir->x * (-obj->pos.x) + dir->y * (-obj->pos.y) + dir->z * (-obj->pos.z));
	const float c = obj->d;

	float d = b * b - dirDot * c;
	if (d >= 0) {
		d = (float)sqrt(d);

		float t = (-b - d) / dirDot;
		if (t <= 0) {
			t = (-b + d) / dirDot;
		}
		return t;
	}
	return -1;
}

static void traceRayFromReflectPosition(Hit *hit)
{
	float t, tMin = FAR_F;

	Object *prevHitObj = hit->obj;

	Vec3 reflectPos;
	reflectPos.x = hit->pos.x;
	reflectPos.y = hit->pos.y;
	reflectPos.z = hit->pos.z;

	hit->obj = NULL;

	for (int i=0; i<NUM_PLANES; ++i) {
		if (prevHitObj != &plane[i]) {
			t = intersectPlane2(&plane[i], &reflectPos, &hit->reflectNormal);
			if (t > 0.0f) { tMin = t; hit->obj = &plane[i]; }
		}
	}

	for (int i=0; i<NUM_SPHERES; ++i) {
		if (prevHitObj != &sphere[i]) {
			t = intersectSphere2(&sphere[i], &reflectPos, &hit->reflectNormal); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[i]; }
		}
	}

	if (tMin < FAR_F) {
		hit->pos.x = reflectPos.x + tMin * hit->reflectNormal.x;
		hit->pos.y = reflectPos.y + tMin * hit->reflectNormal.y;
		hit->pos.z = reflectPos.z + tMin * hit->reflectNormal.z;
	}
}

static void traceRayFromOriginView(Vec3 *dir, float dirDot, Hit *hit)
{
	float t, tMin = FAR_F;

	hit->obj = NULL;

	for (int i=0; i<NUM_PLANES; ++i) {
		t = intersectPlane(&plane[i], dir);
		if (t > 0.0f) { tMin = t; hit->obj = &plane[i]; }
	}

	for (int i=0; i<NUM_SPHERES; ++i) {
		t = intersectSphere(&sphere[i], dir, dirDot); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[i]; }
	}

	if (tMin < FAR_F) {
		hit->pos.x = tMin * dir->x;
		hit->pos.y = tMin * dir->y;
		hit->pos.z = tMin * dir->z;
	}
}

static void setObjectPlane(Object* obj, Vec3 pos, Vec3 dir1, Vec3 dir2, int index, float reflectivity)
{
	obj->pos = pos;
	obj->dir1 = dir1;
	obj->dir2 = dir2;
	obj->type = OBJ_PLANE;
	obj->reflectivity = reflectivity;
	obj->index = index;
}

static void setObjectSphere(Object* obj, Vec3 pos, float radius, int index, float reflectivity)
{
	obj->pos = pos;
	obj->radius = radius;
	obj->type = OBJ_SPHERE;
	obj->reflectivity = reflectivity;
	obj->index = index;
}

static void setupObjects()
{
	setObjectPlane(&plane[0], Vec3(0, -1, 0), Vec3(1, 0, 0), Vec3(0, 0, 1), 2, 0.0f);
	setObjectPlane(&plane[1], Vec3(0, 1, 0), Vec3(1, 0, 0), Vec3(0, 0, 1), 3, 0.0f);
	setObjectSphere(&sphere[0], Vec3(0, -1, 4), 0.25f, 1, 0.0f);
	setObjectSphere(&sphere[1], Vec3(-1, -1, 4), 0.375f, 2, 1.0f);
	setObjectSphere(&sphere[2], Vec3(1, -1, 4), 0.325f, 3, 0.0f);

	for (int i = 0; i < NUM_PLANES; ++i) {
		calcABCD(&plane[i]);
	}
	for (int i = 0; i < NUM_SPHERES; ++i) {
		calcABCD(&sphere[i]);
	}
}

static void updateObjects(int ticks)
{
	for (int i = 0; i < 3; ++i) {
		float t = (float)((i + 1) << 10) + (float)ticks * 0.5f;
		float posX = (float)sin(t / 650.0f) * 0.5f;
		float posY = (float)sin(t / 480.0f) * 0.375f;
		float posZ = 1.75f + (float)sin(t / 880.0f) * 0.325f;

		sphere[i].pos = Vec3(posX, posY, posZ);
		calcABCD(&sphere[i]);
	}
}


static void raytraceInitTextures()
{
	ImggenParams paramsClouds = generateImageParamsCloud(CLOUD_TEX_W, CLOUD_TEX_H, 0,63, 1,255,127,7,3);
	genCloudTexture(CLOUD_TEX_W, CLOUD_TEX_H, 0, 63, &paramsClouds, ceilingTex);

	genGridTexture(CYBER_TEX_W, CYBER_TEX_H, floorTex);
}

void raytraceInit0(Screen* buffer)
{
	perPixelPrecs.viewDir = new Vec3[buffer->width * buffer->height];
	perPixelPrecs.dirDot = new float[buffer->width * buffer->height];

	setupViewDirPerPixelLUTs(buffer);

	setupObjects();

	raytraceInitTextures();
}

static Vec3 getReflectDir(Vec3 *viewDir, Vec3 *hitNormal)
{
	Vec3 reflectDir;

	Vscale(hitNormal, &reflectDir, 2.0f * Vdot(viewDir, hitNormal));
	Vsub(viewDir, &reflectDir, &reflectDir);
	Vnormalize(&reflectDir);

	return reflectDir;
}

static inline int computeColor(Vec3* dir, Hit* hit, int ticks)
{
	Object* obj = hit->obj;
	Vec3 *normal = &hit->normal;
	int colDiffuse = 0;
	int colSpecular = 0;
	int colFinal = 0;

	if (obj) {
		switch (obj->type) {
			case OBJ_PLANE:
			{
				uint8 *texture;
				int planeTexW;
				int planeTexH;
				float scaleW;
				float scaleH;
				float zOff;

				normal->x = obj->a;
				normal->y = obj->b;
				normal->z = obj->c;

				if (obj->pos.y < 0) {
					scaleW = 1.0f;
					scaleH = 1.0f;
					zOff = (float)ticks * 0.0005f;
					texture = floorTex;
					planeTexW = CYBER_TEX_W;
					planeTexH = CYBER_TEX_H;
				} else {
					normal->y = -normal->y;
					scaleW = 0.125f;
					scaleH = 0.25f;
					zOff = 0.5f;
					texture = ceilingTex;
					planeTexW = CLOUD_TEX_W;
					planeTexH = CLOUD_TEX_H;
				}

				colDiffuse = (int)(Vdot(dir, normal) * 256);
				//CLAMP(colDiffuse, 0, 255);
				int u = (int)((scaleW * hit->pos.x + 0.5f) * planeTexW) & (planeTexW - 1);
				int v = (int)((scaleH * hit->pos.z + 0.5f + zOff) * planeTexH) & (planeTexH - 1);
				colFinal = (texture[v * planeTexW + u] * colDiffuse) >> 7;
				CLAMP(colFinal, 0, COLS_OBJ_RANGE - 1)
			}
			break;

			case OBJ_SPHERE:
			{
				normal->x = -(hit->pos.x - obj->a);
				normal->y = -(hit->pos.y - obj->b);
				normal->z = -(hit->pos.z - obj->c);

				Vnormalize(normal);

				colDiffuse = (int)(Vdot(&lightDir, normal) * (DIFFUSE_COLS - 1));
				CLAMP(colDiffuse, 0, DIFFUSE_COLS-1);

				hit->reflectNormal = getReflectDir(dir, normal);
				colSpecular = (int)(pow(Vdot(&lightDir, &hit->reflectNormal), 16.0f) * (SPEC_COLS - 1));
				CLAMP(colSpecular, 0, SPEC_COLS - 1);

				colFinal = colDiffuse + colSpecular;

				break;
			}
		}
	}
	return colFinal;
}

static void renderObjects(Screen* buffer, int ticks)
{
	int width = buffer->width;
	int height = buffer->height;
	int count = buffer->width * buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Hit hit;
	Vec3* dir = perPixelPrecs.viewDir;
	float* dirDot = perPixelPrecs.dirDot;

	lightDir = Vec3(-0.25f, -0.5f, 0.875f);
	Vnormalize(&lightDir);

	for (int y=0; y<height; ++y) {
		for (int x=0; x<width; ++x) {
			traceRayFromOriginView(dir, *dirDot, &hit);

			uint8 c = computeColor(dir, &hit, ticks);
			uint8 reflectAdd = 0;

			if (hit.obj && hit.obj->reflectivity > 0.0f) {
				traceRayFromReflectPosition(&hit);
				reflectAdd = 4;
				c = (c + 3*computeColor(dir, &hit, ticks)) >> 2;
			}

			if (hit.obj) {
				CLAMP(c, 0, COLS_OBJ_RANGE);
				c += (hit.obj->index << COL_BITS_OBJ);
			} else {
				c = 208 - (y >> 3) + reflectAdd;
			}

			*dst++ = c;

			dir++;
			dirDot++;
		}
	}
}

void raytraceRun0(Screen* buffer, int ticks)
{
	updateObjects(ticks);

	renderObjects(buffer, ticks);
}
