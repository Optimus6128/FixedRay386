#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#include "types.h"
#include "vectorFp.h"
#include "raytrace.h"
#include "mathutil.h"


#define FAR_FI (1 << 30)

typedef struct Object
{
	int type;
	Vec3fp pos, dir1, dir2;
	int radius;
	int invR;
	int a, b, c, d;
	int index;
} Object;

typedef struct Hit
{
	Object* obj;
	Vec3fp pos;
} Hit;

typedef struct PerPixelVars
{
	int dirDot, invDirDot;
}PerPixelVars;

typedef struct PerPixelPrecs
{
	Vec3fp* viewDir;
	PerPixelVars* vars;
} PerPixelPrecs;


static Object plane;
static Object sphere[3];
static Vec3fp lightDir0;

static PerPixelPrecs perPixelPrecs;

#define HIT_BUFF_WIDTH ((320 / 2) + 1)
#define HIT_BUFF_HEIGHT ((200 / 2) + 1)

static Hit* hitBuffer;

static int *planeYt;
static int planeTicks;

static void setupPlaneYprecs(Screen* buffer)
{
	const int screenWidth = buffer->width;
	const int screenHeight = buffer->height;

	Object* obj = &plane;

	planeYt = new int[screenHeight+1];

	for (int y = 0; y <= screenHeight; ++y) {
		Vec3fp* dir = &perPixelPrecs.viewDir[y * screenWidth];
		const int denom = FP_MUL_T(obj->a, dir->x) + FP_MUL_T(obj->b, dir->y) + FP_MUL_T(obj->c, dir->z);
		int t = -1;
		if (denom != 0) {
			t = -FP_DIV_T(obj->d, denom);
		}
		planeYt[y] = t;
	}
}

static void setupViewDirPerPixelLUTs(Screen* buffer)
{
	const int screenWidth = buffer->width;
	const int screenHeight = buffer->height;
	const float aspectRatio = (float)screenHeight / (float)screenWidth;

	const int Xinc = FLT_TO_FP(2.0f / (screenWidth - 1));
	const int Yinc = FLT_TO_FP((2.0f / (screenHeight - 1)) * aspectRatio);

	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	int Ydir = (screenHeight / 2) * Yinc;
	for (int y = 0; y <= screenHeight; y++) {
		int Xdir = (-screenWidth / 2) * Xinc;
		for (int x = 0; x <= screenWidth; x++) {
			int dd;
			dir->x = Xdir;
			dir->y = Ydir;
			dir->z = INT_TO_FP(1);

			//Don't normalize yet, if we can get the vis without it..
			//Vnormalize(dir);

			dd = 2 * (FP_MUL_T(dir->x, dir->x) + FP_MUL_T(dir->y, dir->y) + FP_MUL_T(dir->z, dir->z));

			perPixelVars->dirDot = dd;
			if (dd == 0) dd = 1;	// move a bit just in case to avoid division by zero (didn't happen though)
			perPixelVars->invDirDot = FP_DIV_T(FLT_TO_FP(1.0f), dd);

			perPixelVars++;
			dir++;
			Xdir += Xinc;
		}
		Ydir -= Yinc;
	}
}

static int Vdot(Vec3fp* v1, Vec3fp* v2)
{
    return FP_MUL_T(v1->x, v2->x) + FP_MUL_T(v1->y, v2->y) + FP_MUL_T(v1->z, v2->z);
}

static int Vlength(Vec3fp *v)
{
	return FP_SQRT(Vdot(v, v));
}

static void Vnormalize (Vec3fp* v)
{
	int r = Vlength(v);
	if (r!=0) {
		v->x = FP_DIV_T(v->x, r);
		v->y = FP_DIV_T(v->y, r);
		v->z = FP_DIV_T(v->z, r);
	}
}

static void Vcross(Vec3fp* v1, Vec3fp* v2, Vec3fp* v)
{
    v->x = FP_MUL_T(v1->y, v2->z) - FP_MUL_T(v2->y, v1->z);
    v->y = FP_MUL_T(v1->z, v2->x) - FP_MUL_T(v2->z, v1->x);
    v->z = FP_MUL_T(v1->x, v2->y) - FP_MUL_T(v2->x, v1->y);
}

static void calcABCD(Object* obj)
{
	Vec3fp v;
	switch (obj->type) {

		case OBJ_PLANE:
			Vcross(&obj->dir1, &obj->dir2, &v);
			Vnormalize(&v);
			obj->a = v.x;
			obj->b = v.y;
			obj->c = v.z;
			obj->d = -Vlength(&obj->pos);
			break;

		case OBJ_SPHERE:
			obj->a = obj->pos.x;
			obj->b = obj->pos.y;
			obj->c = obj->pos.z;
			obj->d = 2 * (FP_MUL_T(obj->pos.x, obj->pos.x) + FP_MUL_T(obj->pos.y, obj->pos.y) + FP_MUL_T(obj->pos.z, obj->pos.z) - FP_MUL_T(obj->radius, obj->radius));
			break;

		default:
			break;
	}
}

static inline int intersectPlane(Object* obj, Vec3fp* dir)
{
	const int denom = FP_MUL_T(obj->a, dir->x) + FP_MUL_T(obj->b, dir->y) + FP_MUL_T(obj->c, dir->z);
	if (denom!=0) {
		return -FP_DIV_T(obj->d, denom);
	}
	return -1;
}

static inline int intersectSphere(Object* obj, Vec3fp* dir, PerPixelVars *perPixelVars)
{
	const int b = 2 * (FP_MUL_T(dir->x, -obj->pos.x) + FP_MUL_T(dir->y, -obj->pos.y) + FP_MUL_T(dir->z, -obj->pos.z));
	const int c = obj->d;

	int d = FP_MUL_T(b, b) - FP_MUL_T(perPixelVars->dirDot, c);
	if (d >= 0) {
		int t;
		int invDirDot = perPixelVars->invDirDot;
		d = FP_SQRT(d);

		t = FP_MUL_T(-b - d, invDirDot);
		if (t <= 0) {
			t = FP_MUL_T(-b + d, invDirDot);
		}
		return t;
	}
	return -1;
}

static void traceRay(Vec3fp *dir, PerPixelVars *perPixelVars, Hit *hit, int y)
{
	int t, tMin = FAR_FI;

	hit->obj = NULL;
	//t = intersectPlane(&plane, dir);
	t = planeYt[y];	// cheat, we don't even call intersectPlane as it's a very trivial parallel to ground plane, prestored t hit per scanline
	if (t > 0) { tMin = t; hit->obj = &plane; }

	t = intersectSphere(&sphere[0], dir, perPixelVars); if (t > 0 && t < tMin) { tMin = t; hit->obj = &sphere[0]; }
	t = intersectSphere(&sphere[1], dir, perPixelVars); if (t > 0 && t < tMin) { tMin = t; hit->obj = &sphere[1]; }
	t = intersectSphere(&sphere[2], dir, perPixelVars); if (t > 0 && t < tMin) { tMin = t; hit->obj = &sphere[2]; }

	if (tMin < FAR_FI) {
		hit->pos.x = FP_MUL_T(tMin, dir->x);
		hit->pos.y = FP_MUL_T(tMin, dir->y);
		hit->pos.z = FP_MUL_T(tMin, dir->z);
	}
}

static void setObjectPlane(Object* obj, Vec3fp pos, Vec3fp dir1, Vec3fp dir2)
{
	obj->pos = pos;
	obj->dir1 = dir1;
	obj->dir2 = dir2;
	obj->type = OBJ_PLANE;
	obj->index = 0;
}

static void setObjectSphere(Object* obj, Vec3fp pos, int radius, int index)
{
	obj->pos = pos;
	obj->radius = radius;
	obj->type = OBJ_SPHERE;
	obj->index = index;
	obj->invR = FP_DIV_T(FLT_TO_FP(1.0f), obj->radius);
}

/*static void printABCD(Object* obj)
{
	printf("R1d_%d) %d %d %d %d\n", obj->type, obj->a, obj->b, obj->c, obj->d);
	printf("R1f_%d) %f %f %f %f\n", obj->type, FP_TO_FLT(obj->a), FP_TO_FLT(obj->b), FP_TO_FLT(obj->c), FP_TO_FLT(obj->d));
}*/

static void setupObjects()
{
	setObjectPlane(&plane, VEC_FLT_TO_FP(0, 0, -1), VEC_FLT_TO_FP(1, 0, 0), VEC_FLT_TO_FP(0, 0, 1));
	setObjectSphere(&sphere[0], VEC_FLT_TO_FP(0, -1, 4), FLT_TO_FP(0.25f), 1);
	setObjectSphere(&sphere[1], VEC_FLT_TO_FP(-1, -1, 4), FLT_TO_FP(0.25f), 2);
	setObjectSphere(&sphere[2], VEC_FLT_TO_FP(1, -1, 4), FLT_TO_FP(0.25f), 3);

	calcABCD(&plane);
	for (int i = 0; i < 3; ++i) {
		calcABCD(&sphere[i]);
	}
}

static void updateObjects(int ticks)
{
	for (int i = 0; i < 3; ++i) {
		float t = (float)(((i + 1) << 10) + ticks);
		float posX = (float)sin(t / 650.0f) * 0.5f;
		float posY = (float)sin(t / 480.0f) * 0.25f;
		float posZ = 1.75f + (float)sin(t / 880.0f) * 0.25f;

		sphere[i].pos = VEC_FLT_TO_FP(posX, posY, posZ);
		calcABCD(&sphere[i]);
	}

	planeTicks = ticks << 2;
}

void raytraceInit1(Screen* buffer)
{
	const int bufferSize = (buffer->width + 1) * (buffer->height + 1);

	hitBuffer = new Hit[HIT_BUFF_WIDTH * HIT_BUFF_HEIGHT];

	perPixelPrecs.viewDir = new Vec3fp[bufferSize];
	perPixelPrecs.vars = new PerPixelVars[bufferSize];

	setupViewDirPerPixelLUTs(buffer);

	setupObjects();

	setupPlaneYprecs(buffer);

	lightDir0 = VEC_FLT_TO_FP(-0.25f, -0.5f, 0.875f);
	Vnormalize(&lightDir0);
}

static inline int computeColor(Vec3fp* dir, Hit* hit)
{
	Object* obj = hit->obj;
	Vec3fp normal;
	int ci = 0;
	int u, v;

	if (obj) {
		switch (obj->type) {
		case OBJ_PLANE:
		{
			normal.x = obj->a;
			normal.y = obj->b;
			normal.z = obj->c;

			ci = Vdot(dir, &normal);
			CLAMP(ci, 0, ((1 << RP_BITS) - 1));
			u = (hit->pos.x >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
			v = ((hit->pos.z + planeTicks) >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
			ci = FP_MUL((u ^ v), ci);
		}
		break;

		case OBJ_SPHERE:
		{
			normal.x = -FP_MUL_T((hit->pos.x - obj->a), obj->invR);
			normal.y = -FP_MUL_T((hit->pos.y - obj->b), obj->invR);
			normal.z = -FP_MUL_T((hit->pos.z - obj->c), obj->invR);

			ci = Vdot(&lightDir0, &normal) >> (RP_BITS - COL_BITS_OBJ);
			CLAMP(ci, 0, COLS_OBJ_RANGE);
		}
		break;
		}
		ci += (obj->index << COL_BITS_OBJ);
	}
	return ci;
}
static inline int computeColorObjIndex(Hit* hit)
{
	Object* obj = hit->obj;
	if (obj) {
		return (((obj->index+1) << 6) - 1);
	}
	return 0;
}

static void renderObjects(Screen* buffer, int ticks)
{
	const int width = buffer->width;
	const int height = buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Hit hit;
	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	for (int y=0; y<height; ++y) {
		for (int x = 0; x < width; ++x) {
			traceRay(dir, perPixelVars, &hit, y);

			//*dst++ = computeColorObjIndex(&hit);
			*dst++ = computeColor(dir, &hit);

			dir++;
			perPixelVars++;
		}
		dir++;
		perPixelVars++;
	}
}

static int computeColorNothing(Vec3fp* /*dir*/, Hit* /*hit*/)
{
	return 0;
}

static int computeColorPlane(Vec3fp* dir, Hit* hit)
{
	Object* obj = hit->obj;
	Vec3fp normal;
	int ci = 0;
	int u, v;

	normal.x = obj->a;
	normal.y = obj->b;
	normal.z = obj->c;

	ci = Vdot(dir, &normal);
	CLAMP(ci, 0, ((1 << RP_BITS) - 1));
	u = (hit->pos.x >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
	v = ((hit->pos.z + planeTicks) >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
	ci = FP_MUL((u ^ v), ci);
	ci += (obj->index << COL_BITS_OBJ);

	return ci;
}

static int computeColorSphere(Vec3fp* /*dir*/, Hit* hit)
{
	Object* obj = hit->obj;
	Vec3fp normal;
	int ci = 0;
	//int u, v;

	normal.x = -FP_MUL_T((hit->pos.x - obj->a), obj->invR);
	normal.y = -FP_MUL_T((hit->pos.y - obj->b), obj->invR);
	normal.z = -FP_MUL_T((hit->pos.z - obj->c), obj->invR);

	ci = Vdot(&lightDir0, &normal) >> (RP_BITS - COL_BITS_OBJ);
	CLAMP(ci, 0, COLS_OBJ_RANGE);
	ci += (obj->index << COL_BITS_OBJ);

	return ci;
}

static int (*computeColorObject[OBJ_TYPES_COUNT])(Vec3fp*, Hit*) = { computeColorNothing, computeColorPlane, computeColorSphere };

static void renderSubdiv2x(Screen* buffer, int ticks)
{
	const int width = buffer->width;
	const int height = buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	Hit* hitBuff = hitBuffer;
	for (int y = 0; y < height; y += 2) {
		for (int x = 0; x < width; x += 2) {
			Hit* hitUL = &hitBuff[0];
			Hit* hitUR = &hitBuff[1];
			Hit* hitLL = &hitBuff[HIT_BUFF_WIDTH];
			Hit* hitLR = &hitBuff[HIT_BUFF_WIDTH+1];

			Hit hitP;
			int objType = OBJ_NOTHING;

			if (hitUL->obj) {
				objType = hitUL->obj->type;
			}
			int (*computeColorFunc)(Vec3fp*, Hit*) = computeColorObject[objType];
			*dst = computeColorFunc(dir, hitUL);

			hitP.obj = hitUL->obj;
			if (hitUL->obj == hitUR->obj && hitLL->obj == hitLR->obj && hitUL->obj == hitLL->obj) {
				if (objType == OBJ_NOTHING) {
					*dst = 0;
					*(dst+1) = 0;
					*(dst + width) = 0;
					*(dst + width + 1) = 0;
				} else {
					const int posULx = hitUL->pos.x;
					const int posULy = hitUL->pos.y;
					const int posULz = hitUL->pos.z;
					Vec3fp* posUR = &hitUR->pos;
					Vec3fp* posLL = &hitLL->pos;
					Vec3fp* posLR = &hitLR->pos;

					hitP.pos.x = (posULx + posUR->x) >> 1; hitP.pos.y = (posULy + posUR->y) >> 1; hitP.pos.z = (posULz + posUR->z) >> 1;
					*(dst + 1) = computeColorFunc(dir, &hitP);
					hitP.pos.x = (posULx + posLL->x) >> 1; hitP.pos.y = (posULy + posLL->y) >> 1; hitP.pos.z = (posULz + posLL->z) >> 1;
					*(dst + width) = computeColorFunc(dir, &hitP);
					hitP.pos.x = (posULx + posLR->x) >> 1; hitP.pos.y = (posULy + posLR->y) >> 1; hitP.pos.z = (posULz + posLR->z) >> 1;
					*(dst + width + 1) = computeColorFunc(dir, &hitP);
				}
			} else {
				traceRay(dir + 1, perPixelVars+1, &hitP, y); *(dst + 1) = computeColor(dir, &hitP);
				traceRay(dir + width + 1, perPixelVars + width + 1, &hitP, y + 1); *(dst + width) = computeColor(dir + width + 1, &hitP);
				traceRay(dir + width + 2, perPixelVars + width + 2, &hitP, y + 1); *(dst + width + 1) = computeColor(dir + width + 2, &hitP);
			}

			perPixelVars += 2;
			dir += 2;
			dst += 2;
			hitBuff++;
		}
		perPixelVars += width + 2;
		dir += width + 2;
		dst += width;
		hitBuff++;
	}
}

static void renderObjectsSubdiv(Screen* buffer, int ticks)
{
	const int width = buffer->width;
	const int height = buffer->height;

	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	Hit *hitBuff = hitBuffer;
	for (int y = 0; y <= height; y+=2) {
		for (int x = 0; x <= width; x+=2) {
			traceRay(dir, perPixelVars, hitBuff, y);

			perPixelVars+=2;
			dir += 2;
			hitBuff++;
		}
		perPixelVars += width;
		dir += width;
	}

	renderSubdiv2x(buffer, ticks);
}

void raytraceRun1(Screen* buffer, int ticks)
{
	updateObjects(ticks);

	//renderObjects(buffer, ticks);
	renderObjectsSubdiv(buffer, ticks);
}
