#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#include "types.h"
#include "vectorFp.h"
#include "raytrace.h"
#include "mathutil.h"


#define FAR_FI (1 << 30)

#define RT_WIDTH 320
#define RT_HEIGHT 200

#define RT_BUFF_WIDTH (RT_WIDTH + 1)
#define RT_BUFF_HEIGHT (RT_HEIGHT + 1)

#define HIT_BUFF_WIDTH ((RT_WIDTH / 2) + 1)
#define HIT_BUFF_HEIGHT ((RT_HEIGHT / 2) + 1)

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
	Vec3fp normal;
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


static Object nothing;
static Object plane;
static Object sphere[3];
static Vec3fp lightDir0;

static PerPixelPrecs perPixelPrecs;

static Hit* hitBuffer;

static int *planeYt;
static int planeTicks;

static void setupPlaneYprecs()
{
	Object* obj = &plane;

	planeYt = new int[RT_BUFF_HEIGHT];

	for (int y = 0; y < RT_BUFF_HEIGHT; ++y) {
		Vec3fp* dir = &perPixelPrecs.viewDir[y * RT_BUFF_WIDTH];
		const int denom = FP_MUL_T(obj->a, dir->x) + FP_MUL_T(obj->b, dir->y) + FP_MUL_T(obj->c, dir->z);
		int t = -1;
		if (denom != 0) {
			t = -FP_DIV_T(obj->d, denom);
		}
		planeYt[y] = t;
	}
}

static void setupViewDirPerPixelLUTs()
{
	const float aspectRatio = (float)RT_HEIGHT / (float)RT_WIDTH;

	const int Xinc = FLT_TO_FP(2.0f / (RT_WIDTH - 1));
	const int Yinc = FLT_TO_FP((2.0f / (RT_HEIGHT - 1)) * aspectRatio);

	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	int Ydir = (RT_HEIGHT / 2) * Yinc;
	for (int y = 0; y <= RT_HEIGHT; y++) {
		int Xdir = (-RT_WIDTH / 2) * Xinc;
		for (int x = 0; x <= RT_WIDTH; x++) {
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
	Object* obj = &nothing;

	//t = intersectPlane(&plane, dir);
	t = planeYt[y];	// cheat, we don't even call intersectPlane as it's a very trivial parallel to ground plane, prestored t hit per scanline
	if (t > 0) { tMin = t; obj = &plane; }

	t = intersectSphere(&sphere[0], dir, perPixelVars); if (t > 0 && t < tMin) { tMin = t; obj = &sphere[0]; }
	t = intersectSphere(&sphere[1], dir, perPixelVars); if (t > 0 && t < tMin) { tMin = t; obj = &sphere[1]; }
	t = intersectSphere(&sphere[2], dir, perPixelVars); if (t > 0 && t < tMin) { tMin = t; obj = &sphere[2]; }

	hit->obj = obj;
	if (obj) {
		hit->pos.x = FP_MUL_T(tMin, dir->x);
		hit->pos.y = FP_MUL_T(tMin, dir->y);
		hit->pos.z = FP_MUL_T(tMin, dir->z);

		if (obj->type == OBJ_PLANE) {
			hit->normal.x = obj->a;
			hit->normal.y = obj->b;
			hit->normal.z = obj->c;
		} else {
			hit->normal.x = -FP_MUL_T((hit->pos.x - obj->a), obj->invR);
			hit->normal.y = -FP_MUL_T((hit->pos.y - obj->b), obj->invR);
			hit->normal.z = -FP_MUL_T((hit->pos.z - obj->c), obj->invR);
		}
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
	nothing.type = OBJ_NOTHING;

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

static void outputGeneratedSubdivCross(FILE* f, int range, bool isSphere)
{
	for (int y = 0; y < range; ++y) {
		for (int x = 0; x < range; ++x) {
			if (x>0 || y>0) {
				int x0 = x;
				int y0 = y;
				int x1 = x;
				int y1 = y;

				if (x>0) {
					x0 = x - 1;
					x1 = x + 1;
				}
				if (y>0) {
					y0 = y - 1;
					y1 = y + 1;
				}

				fprintf(f, "\t\t\t\t\t\thit%d_%d->pos.x = (pos%d_%d->x + pos%d_%d->x) >> 1; hit%d_%d->pos.y = (pos%d_%d->y + pos%d_%d->y) >> 1; hit%d_%d->pos.z = (pos%d_%d->z + pos%d_%d->z) >> 1;\n", x, y, x0, y0, x1, y1, x, y, x0, y0, x1, y1, x, y, x0, y0, x1, y1);
				if (isSphere) {
					fprintf(f, "\t\t\t\t\t\thit%d_%d->normal.x = (norm%d_%d->x + norm%d_%d->x) >> 1; hit%d_%d->normal.y = (norm%d_%d->y + norm%d_%d->y) >> 1; hit%d_%d->normal.z = (norm%d_%d->z + norm%d_%d->z) >> 1;\n", x, y, x0, y0, x1, y1, x, y, x0, y0, x1, y1, x, y, x0, y0, x1, y1);
				}
				fprintf(f, "\t\t\t\t\t\thit%d_%d->obj = hitP.obj;\n", x, y);
				if (isSphere) {
					fprintf(f, "\t\t\t\t\t\t*(dst + %d * RT_WIDTH + %d) = computeColorSphere(hit%d_%d);\n", 2 * y, 2 * x, x, y);
				}
				else {
					fprintf(f, "\t\t\t\t\t\t*(dst + %d * RT_WIDTH + %d) = computeColorPlane(dir + %d * RT_BUFF_WIDTH + %d, hit%d_%d);\n", 2 * y, 2 * x, 2 * y, 2 * x, x, y);
				}
				if (y != range) fprintf(f, "\n");
			}
		}
	}
}

static void outputGeneratedSubdiv(int subDiv)
{
	static char filename[64];

	sprintf(filename, "renderSubdiv%dx.cpp", subDiv);
	FILE* f = fopen(filename, "w");

	fprintf(f, "static void renderSubdiv%dx(Screen* buffer)\n{\n", subDiv);
	fprintf(f, "\tuint8* dst = (uint8*)buffer->data;\n\n");

	fprintf(f, "\tVec3fp* dir = perPixelPrecs.viewDir;\n");
	fprintf(f, "\tPerPixelVars* perPixelVars = perPixelPrecs.vars;\n");
	fprintf(f, "\tHit* hitBuff = hitBuffer;\n\n");

	fprintf(f, "\tfor (int y = 0; y < RT_HEIGHT; y += %d) {\n", subDiv);
	fprintf(f, "\t\tfor (int x = 0; x < RT_WIDTH; x += %d) {\n", subDiv);

	const int range = subDiv / 2;

	for (int y = 0; y <= range; ++y) {
		for (int x = 0; x <= range; ++x) {
			if (!(x == range && y == range / 2) && !(x == range / 2 && y == range)) {
				fprintf(f, "\t\t\tHit* hit%d_%d = &hitBuff[%d*HIT_BUFF_WIDTH + %d];\n", x, y, y, x);
			}
		}
	}
	fprintf(f, "\n");

	fprintf(f, "\t\t\tHit hitP;\n");
	fprintf(f, "\t\t\thitP.obj = hit0_0->obj;\n");
	fprintf(f, "\t\t\tif (hit0_0->obj == hit%d_0->obj && hit0_%d->obj == hit%d_%d->obj && hit0_0->obj == hit0_%d->obj) {\n", range, range, range, range, range);

	fprintf(f, "\t\t\t\tconst int objType = hit0_0->obj->type;\n\n");

	fprintf(f, "\t\t\t\tif (objType == OBJ_NOTHING) {\n");
	switch (subDiv) {
		case 2:
			fprintf(f, "\t\t\t\t\tuint16 * dst16 = (uint16*)dst;\n");
			fprintf(f, "\t\t\t\t\t*dst16 = 0;\n");
			fprintf(f, "\t\t\t\t\t*(dst16 + RT_WIDTH / 2) = 0;\n");
			break;

		case 4:
			fprintf(f, "\t\t\t\t\tuint32 * dst32 = (uint32*)dst;\n");
			for (int i = 0; i < 4; ++i) {
				fprintf(f, "\t\t\t\t\t*(dst32 + (%d * RT_WIDTH / 4)) = 0;\n", i);
			}
			break;

		case 8:
			fprintf(f, "\t\t\t\t\tuint32 * dst32 = (uint32*)dst;\n");
			for (int k = 0; k < 2; ++k) {
				for (int j = 0; j < 2; ++j) {
					for (int i = 0; i < 4; ++i) {
						fprintf(f, "\t\t\t\t\t*(dst32 + (%d * RT_WIDTH / 4) + %d) = 0;\n", i + 4 * k, j);
					}
				}
			}
			break;
	}
	fprintf(f, "\t\t\t\t} else {\n");

	for (int y = 0; y <= range; y += range) {
		for (int x = 0; x <= range; x += range) {
			fprintf(f, "\t\t\t\t\tVec3fp* pos%d_%d = &hit%d_%d->pos;\n", x, y, x, y);
		}
	}
	fprintf(f, "\n");

	fprintf(f, "\t\t\t\t\t*dst = computeColor(dir, hit0_0);\n");
	fprintf(f, "\t\t\t\t\thitP.obj = hit0_0->obj;\n\n");

	fprintf(f, "\t\t\t\t\tif (objType == OBJ_PLANE) {\n");

	outputGeneratedSubdivCross(f, range, false);

	fprintf(f,"\t\t\t\t\t} else {\n");
	for (int y = 0; y <= range; y += range) {
		for (int x = 0; x <= range; x += range) {
			fprintf(f, "\t\t\t\t\t\tVec3fp* norm%d_%d = &hit%d_%d->normal;\n", x, y, x, y);
		}
	}
	fprintf(f, "\n");

	outputGeneratedSubdivCross(f, range, true);
	fprintf(f, "\t\t\t\t\t}\n");
	fprintf(f, "\t\t\t\t}\n");
	fprintf(f, "\t\t\t} else {\n");

	fprintf(f, "\t\t\t\tHit* hit%d_%d = &hitBuff[%d*HIT_BUFF_WIDTH + %d];\n", range, range / 2, range / 2, range);
	fprintf(f, "\t\t\t\tHit* hit%d_%d = &hitBuff[%d*HIT_BUFF_WIDTH + %d];\n\n", range / 2, range, range, range / 2);

	fprintf(f, "\t\t\t\tif (!hit1_0->obj) traceRay(dir + %d, perPixelVars + %d, hit1_0, y);\n", range, range);
	fprintf(f, "\t\t\t\tif (!hit0_1->obj) traceRay(dir + %d * RT_BUFF_WIDTH, perPixelVars + %d * RT_BUFF_WIDTH, hit0_1, y + %d);\n", range, range, range);
	fprintf(f, "\t\t\t\tif (!hit1_2->obj) traceRay(dir + %d * RT_BUFF_WIDTH + %d, perPixelVars + %d * RT_BUFF_WIDTH + %d, hit1_2, y + %d);\n", 2 * range, range, 2 * range, range, 2 * range);
	fprintf(f, "\t\t\t\tif (!hit2_1->obj) traceRay(dir + %d * RT_BUFF_WIDTH + %d, perPixelVars + %d * RT_BUFF_WIDTH + %d, hit2_1, y + %d);\n", range, 2 * range, range, 2 * range, range);
	fprintf(f, "\t\t\t\tif (!hit1_1->obj) traceRay(dir + %d * RT_BUFF_WIDTH + %d, perPixelVars + %d * RT_BUFF_WIDTH + %d, hit1_1, y + %d);\n\n", range, range, range, range, range);

	fprintf(f, "\t\t\t\thandleRender%dx%d(hit0_0, dir, perPixelVars, y, dst);\n", range, range);
	fprintf(f, "\t\t\t\thandleRender%dx%d(hit1_0, dir + %d, perPixelVars + %d, y, dst + 2);\n", range, range, range, range);
	fprintf(f, "\t\t\t\thandleRender%dx%d(hit0_1, dir + %d * RT_BUFF_WIDTH, perPixelVars + %d * RT_BUFF_WIDTH, y + %d, dst + %d * RT_WIDTH);\n", range, range, range, range, range, range);
	fprintf(f, "\t\t\t\thandleRender%dx%d(hit1_1, dir + %d * RT_BUFF_WIDTH + %d, perPixelVars + %d * RT_BUFF_WIDTH + %d, y + %d, dst + %d * RT_WIDTH + %d);\n", range, range, range, range, range, range, range, range, range);

	fprintf(f, "\t\t\t}\n");
	fprintf(f, "\t\t\tperPixelVars += %d;\n", subDiv);
	fprintf(f, "\t\t\tdir += %d;\n", subDiv);
	fprintf(f, "\t\t\tdst += %d;\n", subDiv);
	fprintf(f, "\t\t\thitBuff += %d;\n", range);
	fprintf(f, "\t\t}\n");

	fprintf(f, "\t\tperPixelVars += %d * RT_BUFF_WIDTH + 1;\n", subDiv-1);
	fprintf(f, "\t\tdir += %d * RT_BUFF_WIDTH + 1;\n", subDiv - 1);
	fprintf(f, "\t\tdst += %d * RT_WIDTH;\n", subDiv - 1);
	fprintf(f, "\t\thitBuff += %d * HIT_BUFF_WIDTH + 1;\n", range - 1);
	fprintf(f, "\t}\n}\n\n");

	fclose(f);
}

void raytraceInit1()
{
	const int rtBuffSize = RT_BUFF_WIDTH * RT_BUFF_HEIGHT;
	const int hitBuffSize = HIT_BUFF_WIDTH * HIT_BUFF_HEIGHT;

	perPixelPrecs.viewDir = new Vec3fp[rtBuffSize];
	perPixelPrecs.vars = new PerPixelVars[rtBuffSize];

	hitBuffer = new Hit[hitBuffSize];

	setupViewDirPerPixelLUTs();

	setupObjects();

	setupPlaneYprecs();

	lightDir0 = VEC_FLT_TO_FP(-0.25f, -0.5f, 0.875f);
	Vnormalize(&lightDir0);

	//outputGeneratedSubdiv(4);
}

static inline int computeColor(Vec3fp* dir, Hit* hit)
{
	Object* obj = hit->obj;
	int ci = 0;
	int u, v;

	if (obj) {
		switch (obj->type) {
		case OBJ_PLANE:
		{
			//ci = Vdot(dir, &hit->normal);
			ci = FP_MUL_T(dir->y, hit->normal.y);	// hack for speed as plane parallel to ground happens to have Y vector up and everything else is zero.

			CLAMP(ci, 0, ((1 << RP_BITS) - 1));
			u = (hit->pos.x >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
			v = ((hit->pos.z + planeTicks) >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
			ci = FP_MUL((u ^ v), ci);
		}
		break;

		case OBJ_SPHERE:
		{
			ci = Vdot(&lightDir0, &hit->normal) >> (RP_BITS - COL_BITS_OBJ);
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

static void renderObjects(Screen* buffer)
{
	uint8* dst = (uint8*)buffer->data;

	Hit hit;
	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	for (int y=0; y<RT_HEIGHT; ++y) {
		for (int x = 0; x < RT_WIDTH; ++x) {
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

static inline int computeColorPlane(Vec3fp* dir, Hit* hit)
{
	Object* obj = hit->obj;
	int u, v;

	//Vec3fp normal;
	//normal.x = obj->a;
	//normal.y = obj->b;
	//normal.z = obj->c;

	//ci = Vdot(dir, &normal);
	int ci = FP_MUL_T(dir->y, obj->b);	// hack for speed as plane parallel to ground happens to have Y vector up and everything else is zero.

	CLAMP(ci, 0, ((1 << RP_BITS) - 1));
	u = (hit->pos.x >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
	v = ((hit->pos.z + planeTicks) >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
	ci = FP_MUL((u ^ v), ci);
	ci += (obj->index << COL_BITS_OBJ);

	return ci;
}

static inline int computeColorSphere(Hit* hit)
{
	int ci = Vdot(&lightDir0, &hit->normal) >> (RP_BITS - COL_BITS_OBJ);
	CLAMP(ci, 0, COLS_OBJ_RANGE);
	ci += (hit->obj->index << COL_BITS_OBJ);

	return ci;
}

static void handleRender2x2(Hit* hitBuff, Vec3fp* dir, PerPixelVars* perPixelVars, int y, uint8* dst)
{
	Hit* hitUL = &hitBuff[0];
	Hit* hitUR = &hitBuff[1];
	Hit* hitLL = &hitBuff[HIT_BUFF_WIDTH];
	Hit* hitLR = &hitBuff[HIT_BUFF_WIDTH + 1];

	Hit hitP;
	if (hitUL->obj == hitUR->obj && hitLL->obj == hitLR->obj && hitUL->obj == hitLL->obj) {
		const int objType = hitUL->obj->type;

		if (objType == OBJ_NOTHING) {
			uint16* dst16 = (uint16*)dst;
			*dst16 = 0;
			*(dst16 + RT_WIDTH / 2) = 0;
		} else {
			const int posULx = hitUL->pos.x;
			const int posULy = hitUL->pos.y;
			const int posULz = hitUL->pos.z;
			Vec3fp* posUR = &hitUR->pos;
			Vec3fp* posLL = &hitLL->pos;
			Vec3fp* posLR = &hitLR->pos;

			*dst = computeColor(dir, hitUL);
			hitP.obj = hitUL->obj;

			if (objType == OBJ_PLANE) {
				hitP.pos.x = (posULx + posUR->x) >> 1; hitP.pos.y = (posULy + posUR->y) >> 1; hitP.pos.z = (posULz + posUR->z) >> 1;
				*(dst + 1) = computeColorPlane(dir + 1, &hitP);
				hitP.pos.x = (posULx + posLL->x) >> 1; hitP.pos.y = (posULy + posLL->y) >> 1; hitP.pos.z = (posULz + posLL->z) >> 1;
				*(dst + RT_WIDTH) = computeColorPlane(dir + RT_BUFF_WIDTH, &hitP);
				hitP.pos.x = (posULx + posLR->x) >> 1; hitP.pos.y = (posULy + posLR->y) >> 1; hitP.pos.z = (posULz + posLR->z) >> 1;
				*(dst + RT_WIDTH + 1) = computeColorPlane(dir + RT_BUFF_WIDTH + 1, &hitP);
			} else {
				const int normULx = hitUL->normal.x;
				const int normULy = hitUL->normal.y;
				const int normULz = hitUL->normal.z;
				Vec3fp* normUR = &hitUR->normal;
				Vec3fp* normLL = &hitLL->normal;
				Vec3fp* normLR = &hitLR->normal;

				hitP.pos.x = (posULx + posUR->x) >> 1; hitP.pos.y = (posULy + posUR->y) >> 1; hitP.pos.z = (posULz + posUR->z) >> 1;
				hitP.normal.x = (normULx + normUR->x) >> 1; hitP.normal.y = (normULy + normUR->y) >> 1; hitP.normal.z = (normULz + normUR->z) >> 1;
				*(dst + 1) = computeColorSphere(&hitP);
				hitP.pos.x = (posULx + posLL->x) >> 1; hitP.pos.y = (posULy + posLL->y) >> 1; hitP.pos.z = (posULz + posLL->z) >> 1;
				hitP.normal.x = (normULx + normLL->x) >> 1; hitP.normal.y = (normULy + normLL->y) >> 1; hitP.normal.z = (normULz + normLL->z) >> 1;
				*(dst + RT_WIDTH) = computeColorSphere(&hitP);
				hitP.pos.x = (posULx + posLR->x) >> 1; hitP.pos.y = (posULy + posLR->y) >> 1; hitP.pos.z = (posULz + posLR->z) >> 1;
				hitP.normal.x = (normULx + normLR->x) >> 1; hitP.normal.y = (normULy + normLR->y) >> 1; hitP.normal.z = (normULz + normLR->z) >> 1;
				*(dst + RT_WIDTH + 1) = computeColorSphere(&hitP);
			}
		}
	} else {
		*dst = computeColor(dir, hitUL);
		traceRay(dir + 1, perPixelVars + 1, &hitP, y); *(dst + 1) = computeColor(dir, &hitP);
		traceRay(dir + RT_BUFF_WIDTH, perPixelVars + RT_BUFF_WIDTH, &hitP, y + 1); *(dst + RT_WIDTH) = computeColor(dir + RT_BUFF_WIDTH, &hitP);
		traceRay(dir + RT_BUFF_WIDTH + 1, perPixelVars + RT_BUFF_WIDTH + 1, &hitP, y + 1); *(dst + RT_WIDTH + 1) = computeColor(dir + RT_BUFF_WIDTH + 1, &hitP);
	}
}

static void renderSubdiv2x(Screen* buffer)
{
	uint8* dst = (uint8*)buffer->data;

	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	Hit* hitBuff = hitBuffer;
	for (int y = 0; y < RT_HEIGHT; y += 2) {
		for (int x = 0; x < RT_WIDTH; x += 2) {

			handleRender2x2(hitBuff, dir, perPixelVars, y, dst);

			perPixelVars += 2;
			dir += 2;
			dst += 2;
			hitBuff++;
		}
		perPixelVars += RT_BUFF_WIDTH + 1;
		dir += RT_BUFF_WIDTH + 1;
		dst += RT_WIDTH;
		hitBuff++;
	}
}

/*
A * 0 * B
* * * * _
1 * 2 * _
* * * * _
C _ _ _ D
*/

static void renderSubdiv4x(Screen* buffer)
{
	uint8* dst = (uint8*)buffer->data;

	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;
	Hit* hitBuff = hitBuffer;

	for (int y = 0; y < RT_HEIGHT; y += 4) {
		for (int x = 0; x < RT_WIDTH; x += 4) {
			Hit* hit0_0 = &hitBuff[0 * HIT_BUFF_WIDTH + 0];
			Hit* hit1_0 = &hitBuff[0 * HIT_BUFF_WIDTH + 1];
			Hit* hit2_0 = &hitBuff[0 * HIT_BUFF_WIDTH + 2];
			Hit* hit0_1 = &hitBuff[1 * HIT_BUFF_WIDTH + 0];
			Hit* hit1_1 = &hitBuff[1 * HIT_BUFF_WIDTH + 1];
			Hit* hit0_2 = &hitBuff[2 * HIT_BUFF_WIDTH + 0];
			Hit* hit2_2 = &hitBuff[2 * HIT_BUFF_WIDTH + 2];

			Hit hitP;
			hitP.obj = hit0_0->obj;
			if (hit0_0->obj == hit2_0->obj && hit0_2->obj == hit2_2->obj && hit0_0->obj == hit0_2->obj) {
				const int objType = hit0_0->obj->type;

				if (objType == OBJ_NOTHING) {
					uint32* dst32 = (uint32*)dst;
					*(dst32 + (0 * RT_WIDTH / 4)) = 0;
					*(dst32 + (1 * RT_WIDTH / 4)) = 0;
					*(dst32 + (2 * RT_WIDTH / 4)) = 0;
					*(dst32 + (3 * RT_WIDTH / 4)) = 0;
				}
				else {
					Vec3fp* pos0_0 = &hit0_0->pos;
					Vec3fp* pos2_0 = &hit2_0->pos;
					Vec3fp* pos0_2 = &hit0_2->pos;
					Vec3fp* pos2_2 = &hit2_2->pos;

					*dst = computeColor(dir, hit0_0);
					hitP.obj = hit0_0->obj;

					if (objType == OBJ_PLANE) {
						hit1_0->pos.x = (pos0_0->x + pos2_0->x) >> 1; hit1_0->pos.y = (pos0_0->y + pos2_0->y) >> 1; hit1_0->pos.z = (pos0_0->z + pos2_0->z) >> 1;
						hit1_0->obj = hitP.obj;
						*(dst + 0 * RT_WIDTH + 2) = computeColorPlane(dir + 0 * RT_BUFF_WIDTH + 2, hit1_0);

						hit0_1->pos.x = (pos0_0->x + pos0_2->x) >> 1; hit0_1->pos.y = (pos0_0->y + pos0_2->y) >> 1; hit0_1->pos.z = (pos0_0->z + pos0_2->z) >> 1;
						hit0_1->obj = hitP.obj;
						*(dst + 2 * RT_WIDTH + 0) = computeColorPlane(dir + 2 * RT_BUFF_WIDTH + 0, hit0_1);

						hit1_1->pos.x = (pos0_0->x + pos2_2->x) >> 1; hit1_1->pos.y = (pos0_0->y + pos2_2->y) >> 1; hit1_1->pos.z = (pos0_0->z + pos2_2->z) >> 1;
						hit1_1->obj = hitP.obj;
						*(dst + 2 * RT_WIDTH + 2) = computeColorPlane(dir + 2 * RT_BUFF_WIDTH + 2, hit1_1);
					}
					else {
						Vec3fp* norm0_0 = &hit0_0->normal;
						Vec3fp* norm2_0 = &hit2_0->normal;
						Vec3fp* norm0_2 = &hit0_2->normal;
						Vec3fp* norm2_2 = &hit2_2->normal;

						hit1_0->pos.x = (pos0_0->x + pos2_0->x) >> 1; hit1_0->pos.y = (pos0_0->y + pos2_0->y) >> 1; hit1_0->pos.z = (pos0_0->z + pos2_0->z) >> 1;
						hit1_0->normal.x = (norm0_0->x + norm2_0->x) >> 1; hit1_0->normal.y = (norm0_0->y + norm2_0->y) >> 1; hit1_0->normal.z = (norm0_0->z + norm2_0->z) >> 1;
						hit1_0->obj = hitP.obj;
						*(dst + 0 * RT_WIDTH + 2) = computeColorSphere(hit1_0);

						hit0_1->pos.x = (pos0_0->x + pos0_2->x) >> 1; hit0_1->pos.y = (pos0_0->y + pos0_2->y) >> 1; hit0_1->pos.z = (pos0_0->z + pos0_2->z) >> 1;
						hit0_1->normal.x = (norm0_0->x + norm0_2->x) >> 1; hit0_1->normal.y = (norm0_0->y + norm0_2->y) >> 1; hit0_1->normal.z = (norm0_0->z + norm0_2->z) >> 1;
						hit0_1->obj = hitP.obj;
						*(dst + 2 * RT_WIDTH + 0) = computeColorSphere(hit0_1);

						hit1_1->pos.x = (pos0_0->x + pos2_2->x) >> 1; hit1_1->pos.y = (pos0_0->y + pos2_2->y) >> 1; hit1_1->pos.z = (pos0_0->z + pos2_2->z) >> 1;
						hit1_1->normal.x = (norm0_0->x + norm2_2->x) >> 1; hit1_1->normal.y = (norm0_0->y + norm2_2->y) >> 1; hit1_1->normal.z = (norm0_0->z + norm2_2->z) >> 1;
						hit1_1->obj = hitP.obj;
						*(dst + 2 * RT_WIDTH + 2) = computeColorSphere(hit1_1);

					}
				}
			}
			else {
				Hit* hit2_1 = &hitBuff[1 * HIT_BUFF_WIDTH + 2];
				Hit* hit1_2 = &hitBuff[2 * HIT_BUFF_WIDTH + 1];

				if (!hit1_0->obj) traceRay(dir + 2, perPixelVars + 2, hit1_0, y);
				if (!hit0_1->obj) traceRay(dir + 2 * RT_BUFF_WIDTH, perPixelVars + 2 * RT_BUFF_WIDTH, hit0_1, y + 2);
				if (!hit1_2->obj) traceRay(dir + 4 * RT_BUFF_WIDTH + 2, perPixelVars + 4 * RT_BUFF_WIDTH + 2, hit1_2, y + 4);
				if (!hit2_1->obj) traceRay(dir + 2 * RT_BUFF_WIDTH + 4, perPixelVars + 2 * RT_BUFF_WIDTH + 4, hit2_1, y + 2);
				if (!hit1_1->obj) traceRay(dir + 2 * RT_BUFF_WIDTH + 2, perPixelVars + 2 * RT_BUFF_WIDTH + 2, hit1_1, y + 2);

				handleRender2x2(hit0_0, dir, perPixelVars, y, dst);
				handleRender2x2(hit1_0, dir + 2, perPixelVars + 2, y, dst + 2);
				handleRender2x2(hit0_1, dir + 2 * RT_BUFF_WIDTH, perPixelVars + 2 * RT_BUFF_WIDTH, y + 2, dst + 2 * RT_WIDTH);
				handleRender2x2(hit1_1, dir + 2 * RT_BUFF_WIDTH + 2, perPixelVars + 2 * RT_BUFF_WIDTH + 2, y + 2, dst + 2 * RT_WIDTH + 2);
			}
			perPixelVars += 4;
			dir += 4;
			dst += 4;
			hitBuff += 2;
		}
		perPixelVars += 3 * RT_BUFF_WIDTH + 1;
		dir += 3 * RT_BUFF_WIDTH + 1;
		dst += 3 * RT_WIDTH;
		hitBuff += 1 * HIT_BUFF_WIDTH + 1;
	}
}

static void renderObjectsSubdiv4x(Screen* buffer)
{
	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	Hit *hitBuff = hitBuffer;
	for (int y = 0; y < RT_BUFF_HEIGHT; y+=4) {
		for (int x = 0; x < RT_BUFF_WIDTH; x+=4) {
			traceRay(dir, perPixelVars, hitBuff, y);

			perPixelVars+=4;
			dir += 4;
			hitBuff+=2;
		}
		perPixelVars += 3 * RT_BUFF_WIDTH - 3;
		dir += 3 * RT_BUFF_WIDTH - 3;
		hitBuff +=  HIT_BUFF_WIDTH - 1;
	}

	renderSubdiv4x(buffer);
}

static void renderObjectsSubdiv2x(Screen* buffer)
{
	Vec3fp* dir = perPixelPrecs.viewDir;
	PerPixelVars* perPixelVars = perPixelPrecs.vars;

	Hit *hitBuff = hitBuffer;
	int i = 0;
	for (int y = 0; y < RT_BUFF_HEIGHT; y+=2) {
		for (int x = 0; x < RT_BUFF_WIDTH; x+=2) {
			traceRay(dir, perPixelVars, hitBuff, y);

			perPixelVars+=2;
			dir += 2;
			i += 2;
			hitBuff++;
		}
		perPixelVars += RT_BUFF_WIDTH - 1;
		dir += RT_BUFF_WIDTH - 1;
	}

	renderSubdiv2x(buffer);
}

static void resetHitBuffer()
{
	for (int i=0; i<HIT_BUFF_WIDTH * HIT_BUFF_HEIGHT; ++i) {
		hitBuffer[i].obj = NULL;
	}
}

void raytraceRun1(Screen* buffer, int ticks)
{
	//memset(buffer->data, 0, buffer->width * buffer->height);

	resetHitBuffer();

	updateObjects(ticks);

	//renderObjects(buffer);
	//renderObjectsSubdiv2x(buffer);
	renderObjectsSubdiv4x(buffer);
}