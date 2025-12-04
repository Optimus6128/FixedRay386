#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#include "types.h"
#include "raytrace.h"
#include "vector.h"
#include "mathutil.h"


#define FAR_F 1000000000.0f

typedef struct Object
{
	int type;
	Vec3 pos, dir1, dir2;
	float radius;
	float a, b, c, d;
	int index;
} Object;

typedef struct Hit
{
	Object* obj;
	Vec3 pos;
} Hit;

static Object plane;
static Object sphere;


static Vec3 *viewDirPerPixel;
static float* viewDirDotPerPixel;

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

	Vec3* dir = viewDirPerPixel;
	float* dirDot = viewDirDotPerPixel;

    float Ydir = (screenHeight / 2) * Yinc;
    for (int y = 0; y < screenHeight; y++) {
        float Xdir = (-screenWidth / 2) * Xinc;
        for (int x = 0; x < screenWidth; x++) {
			dir->x = Xdir;
			dir->y = Ydir;
			dir->z = distFromCamera;

			//Don't normalize yet, if we can get the vis without it..
			//Vnormalize(dir);

			*dirDot++ = 2.0f * (dir->x * dir->x + dir->y * dir->y + dir->z * dir->z);
			dir++;
            Xdir += Xinc;
        }
        Ydir -= Yinc;
    }
}

static void renderViewDir(Screen* buffer)
{
	const int width = buffer->width;
	const int height = buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Vec3* src = viewDirPerPixel;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			uint32 c = (uint32)((src->x * src->x + src->y * src->y) * 255.0f);
			if (c > 255) c = 255;
			*dst++ = (uint8)(c >> 1);
			++src;
		}
	}
}

static void renderViewDirDot(Screen* buffer)
{
	const int width = buffer->width;
	const int height = buffer->height;
	uint8* dst = (uint8*)buffer->data;

	float* src = viewDirDotPerPixel;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			uint32 c = (uint32)(*src * 16);
			if (c > 255) c = 255;
			*dst++ = (uint8)c;
			++src;
		}
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
			obj->d = (float)-sqrt((obj->pos.x * obj->pos.x) + (obj->pos.y * obj->pos.y) + (obj->pos.z * obj->pos.z));
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

// We will assume not viewer pos is 0,0,0
static inline float intersect(Object* obj, Vec3* dir, float dirDot)
{
	switch (obj->type) {

		case OBJ_PLANE:
		{
			const float denom = obj->a * dir->x + obj->b * dir->y + obj->c * dir->z;

			if (denom != 0) {
				return -obj->d / denom;
			}
			return -1;
		}
		break;

		case OBJ_SPHERE:
		{
			const float a = dirDot;
			if (a == 0.0f) return -1;

			const float b = 2 * (dir->x * (-obj->pos.x) + dir->y * (-obj->pos.y) + dir->z * (-obj->pos.z));
			const float c = obj->d;

			float d = b * b - a * c;
			if (d >= 0) {
				d = (float)sqrt(d);

				// No div by zero despite warning (a happens to be well above zero)
				float t1 = (-b + d) / a;
				float t2 = (-b - d) / a;

				if (t1 <= 0 && t2 <= 0) {
					return -1;
				}

				if (t1 < t2) {
					if (t1 > 0) {
						return t1;
					} else {
						return t2;
					}
				}
				else {
					if (t2 > 0) {
						return t2;
					} else {
						return t1;
					}
				}
			}
		}
		break;
	}
	return -1;
}

static void traceRay(Vec3 *dir, float *dirDot, Hit *hit)
{
	float t0, t1, min = FAR_F;

	hit->obj = NULL;
	t0 = intersect(&plane, dir, 0.0f);
	t1 = intersect(&sphere, dir, *dirDot);
	if (t0 > 0.0f) { min = t0; hit->obj = &plane; }
	if (t1 > 0.0f && t1 < min) { min = t1; hit->obj = &sphere; }

	if (min < FAR_F) {
		hit->pos.x = min * dir->x;
		hit->pos.y = min * dir->y;
		hit->pos.z = min * dir->z;
	}
}

static void setObjectPlane(Object* obj, Vec3 pos, Vec3 dir1, Vec3 dir2)
{
	obj->pos = pos;
	obj->dir1 = dir1;
	obj->dir2 = dir2;
	obj->type = OBJ_PLANE;
	obj->index = 0;
}

static void setObjectSphere(Object* obj, Vec3 pos, float radius, int index)
{
	obj->pos = pos;
	obj->radius = radius;
	obj->type = OBJ_SPHERE;
	obj->index = index;
}

static void printABCD(Object *obj)
{
	printf("R0_%d) %f %f %f %f\n", obj->type, obj->a, obj->b, obj->c, obj->d);
}

static void setupObjects()
{
	setObjectPlane(&plane, Vec3(0, 0, -1), Vec3(1, 0, 0), Vec3(0, 0, 1));
	setObjectSphere(&sphere, Vec3(0, -1, 4), 1, 2);

	calcABCD(&plane);
	calcABCD(&sphere);

	printABCD(&plane);
	printABCD(&sphere);
}

static void updateObjects(int ticks)
{
	float posX = (float)sin(ticks / 650.0f) * 0.25f;
	float posY = (float)sin(ticks / 480.0f) * 0.25f;
	float posZ = 1.0f + (float)sin(ticks / 880.0f) * 0.5f;
	float radius = 0.25f;

	setObjectSphere(&sphere, Vec3(posX, posY, posZ), radius, 2);

	calcABCD(&plane);
	calcABCD(&sphere);
}

void raytraceInit0(Screen* buffer)
{
	viewDirPerPixel = new Vec3[buffer->width * buffer->height];
	viewDirDotPerPixel = new float[buffer->width * buffer->height];

	setupViewDirPerPixelLUTs(buffer);

	setupObjects();
}

static inline int computeColor(Vec3* dir, Hit* hit)
{
	Object* obj = hit->obj;
	Vec3 normal;
	int ci = 0;

	if (obj) {
		switch (obj->type) {
		case OBJ_PLANE:
			normal.x = obj->a;
			normal.y = obj->b;
			normal.z = obj->c;

			ci = (int)(Vdot(dir, &normal) * 256);
			CLAMP(ci, 0, 255);
			ci = ((((int)(hit->pos.x * 16) & 15) ^ ((int)(hit->pos.z * 16) & 15)) * ci) >> 6;
			break;

		case OBJ_SPHERE:
			normal.x = -(hit->pos.x - obj->a);
			normal.y = -(hit->pos.y - obj->b);
			normal.z = -(hit->pos.z - obj->c);

			Vnormalize(&normal);

			ci = (int)(Vdot(dir, &normal) * 256);
			CLAMP(ci, 0, 255);
			break;
		}
	}
	return ci;
}

static inline int computeColorObjIndex(Hit* hit)
{
	Object* obj = hit->obj;
	int ci = 0;

	if (obj) {
		if (obj->type == OBJ_PLANE) {
			return 31;
		} else {
			return ((obj->index + 1) << 5);
		}
	}
	return ci;
}

static void renderObjects(Screen* buffer)
{
	int count = buffer->width * buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Hit hit;
	Vec3* dir = viewDirPerPixel;
	float* dirDot = viewDirDotPerPixel;

	do {
		traceRay(dir, dirDot, &hit);

		*dst++ = computeColorObjIndex(&hit);
		//*dst++ = computeColor(dir, &hit);

		dir++;
		dirDot++;
	} while (--count != 0);
}

void raytraceRun0(Screen* buffer, int ticks)
{
	updateObjects(ticks);

	//renderViewDir(buffer);
	//renderViewDirDot(buffer);

	renderObjects(buffer);
}
