#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "mathutil.h"
#include "types.h"

int isqrt(int x) {
	long long int q = 1;	// very high numbers over ((1<<30)-1) will freeze in while if this wasn't 64bit
	int r = 0;
	while (q <= x) {
		q <<= 2;
	}
	while (q > 1) {
		int t;
		q >>= 2;
		t = x - r - q;
		r >>= 1;
		if (t >= 0) {
			x = t;
			r += q;
		}
	}
	return r;
} 

int getRand(int from, int to)
{
	int rnd;
	if (from > to) return 0;
	if (from==to) return to;
	rnd = from + (rand() % (to - from));
	return rnd;
}
