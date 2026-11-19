/*
 * cs_pos.h — distance filtering + 2D trilateration for Channel Sounding
 *
 * What it fixes:
 *  1. A timeout on one beacon no longer drops the fix: the last valid
 *     distance is used while it is younger than CS_POS_MAX_AGE_MS.
 *  2. Outliers (a jump > CS_POS_OUTLIER_M from the median) are rejected.
 *     If the next measurement agrees with the rejected one, that is real
 *     movement: the buffer restarts from these two values.
 *  3. Distance = median of the last CS_POS_WIN measurements.
 *  4. Position = least squares (works for 3+ beacons) + Gauss-Newton refinement,
 *     with a residual as the fix quality metric.
 *  5. Light EMA smoothing of the position.
 *
 * main.c uses 1-3 (cs_pos_add / cs_pos_dist); the position itself is still computed
 * with Sofia's trilateration. cs_pos_solve() is kept as an alternative solver.
 *
 * Units: distances in metres (as they arrive from CS), coordinates in mm.
 */
#ifndef CS_POS_H
#define CS_POS_H

#include <stdint.h>
#include <stdbool.h>

#define CS_POS_MAX_BEACONS   8
#define CS_POS_WIN           5      /* median window */
#define CS_POS_MAX_AGE_MS    10000  /* ~2 cycles: how long a distance stays valid after a timeout */
#define CS_POS_OUTLIER_M     0.30f  /* outlier threshold */
#define CS_POS_STALE_MS      6000   /* older than one cycle = stale */
#define CS_POS_EMA_ALPHA     0.5f   /* 1.0 = no smoothing */

typedef struct {
	float x_mm, y_mm;               /* beacon position */
	float buf[CS_POS_WIN];
	uint8_t n, head;
	float rejected;                 /* last rejected value, 0 = none */
	int64_t last_ms;                /* time of the last accepted measurement */
	bool ever;
} cs_beacon_t;

typedef struct {
	cs_beacon_t b[CS_POS_MAX_BEACONS];
	uint8_t count;
	float px, py;                   /* smoothed position */
	bool have_pos;
} cs_pos_t;

typedef struct {
	float x_mm, y_mm;
	float resid_mm;                 /* RMS residual: < ~100 mm good, > 250 mm suspicious */
	uint8_t used;                   /* how many beacons were used */
	uint8_t stale;                  /* how many of them came from an old value (after a timeout) */
} cs_fix_t;

void cs_pos_init(cs_pos_t *s, const float xy_mm[][2], uint8_t count);

/* Call on every CS result. valid=false for a timeout. */
void cs_pos_add(cs_pos_t *s, uint8_t idx, bool valid, float dist_m, int64_t now_ms);

/* Filtered distance; false if there is none or it is too old. */
bool cs_pos_dist(const cs_pos_t *s, uint8_t idx, int64_t now_ms, float *dist_m);

/* Compute a position. false if fewer than 3 fresh beacons. */
bool cs_pos_solve(cs_pos_t *s, int64_t now_ms, cs_fix_t *out);

#endif
