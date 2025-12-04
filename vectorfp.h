#ifndef VECTOR_FP_H
#define VECTOR_FP_H

#define RP_BITS 16

#define FLOAT_TO_FIXED(f,b) ((int)((f) * (1 << b)))
#define INT_TO_FIXED(i,b) ((i) << b)
#define FIXED_TO_INT(x,b) ((x) >> b)
#define FIXED_TO_FLOAT(x,b) ((float)(x) / (1 << b))
#define FIXED_MUL(x,y,b) (((x) * (y)) >> b)
#define FIXED_MUL_64(x,y,b) ((int)(((long long int)(x) * (long long int)(y)) >> b))
#define FIXED_DIV(x,y,b) (((x) << b) / (y))
#define FIXED_DIV_64(x,y,b) (((long long int)(x) << b) / (y))
#define FIXED_SQRT(x,b) (sqrt((x) << b))

#define FLT_TO_FP(f)    FLOAT_TO_FIXED(f,RP_BITS)
#define INT_TO_FP(i)    INT_TO_FIXED(i,RP_BITS)
#define FP_TO_INT(x)    FIXED_TO_INT(x,RP_BITS)
#define FP_TO_FLT(x)    FIXED_TO_FLOAT(x,RP_BITS)
#define FP_MUL(x,y)     FIXED_MUL(x,y,RP_BITS)
#define FP_MUL_64(x,y)  FIXED_MUL_64(x,y,RP_BITS)
#define FP_DIV(x,y)     FIXED_DIV(x,y,RP_BITS)
#define FP_DIV_64(x,y)  FIXED_DIV_64(x,y,RP_BITS)

#define VEC_INT_TO_FP(x,y,z) Vec3fp(INT_TO_FP(x), INT_TO_FP(y), INT_TO_FP(z))
#define VEC_FLT_TO_FP(x,y,z) Vec3fp(FLT_TO_FP(x), FLT_TO_FP(y), FLT_TO_FP(z))

#define FP_MUL_T(x,y) FP_MUL_64(x,y)
#define FP_DIV_T(x,y) FP_DIV_64(x,y)

struct Vec3fp
{
    int x, y, z;

    Vec3fp(float x, float y, float z) : x(FLT_TO_FP(x)), y(FLT_TO_FP(y)), z(FLT_TO_FP(z)) {}
    Vec3fp(int x, int y, int z) : x(x), y(y), z(z) {}
    Vec3fp() { x=0; y=0; z=0; }

    inline Vec3fp operator+(const Vec3fp& v) {
        return Vec3fp(x + v.x, y + v.y, z + v.z);
    }

    inline Vec3fp operator-(const Vec3fp& v) {
        return Vec3fp(x - v.x, y - v.y, z - v.z);
    }

    inline Vec3fp& operator+=(Vec3fp& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    inline Vec3fp& operator-=(Vec3fp& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    inline Vec3fp operator*(int s) {
        return Vec3fp(FP_MUL(x,s), FP_MUL(y,s), FP_MUL(z,s));
    }

    inline Vec3fp& operator*=(int s) {
        x = FP_MUL(x,s);
        y = FP_MUL(y,s);
        z = FP_MUL(z,s);
        return *this;
    }

    inline Vec3fp operator/(int d) {
        return Vec3fp(FP_DIV(x, d), FP_DIV(y, d), FP_DIV(z, d));
    }

    inline Vec3fp& operator/=(int d) {
        x = FP_DIV(x, d);
        y = FP_DIV(y, d);
        z = FP_DIV(z, d);
        return *this;
    }
};

#endif
