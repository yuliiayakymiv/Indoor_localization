#include "cs_pos.h"
#include <math.h>
#include <string.h>

void cs_pos_init(cs_pos_t *s, const float xy_mm[][2], uint8_t count)
{
	memset(s, 0, sizeof(*s));
	if (count > CS_POS_MAX_BEACONS) {
		count = CS_POS_MAX_BEACONS;
	}
	s->count = count;
	for (uint8_t i = 0; i < count; i++) {
		s->b[i].x_mm = xy_mm[i][0];
		s->b[i].y_mm = xy_mm[i][1];
	}
}

static float median(const float *v, uint8_t n)
{
	float t[CS_POS_WIN];
	memcpy(t, v, n * sizeof(float));
	for (uint8_t i = 1; i < n; i++) {          /* insertion sort, n <= 5 */
		float k = t[i];
		int j = i - 1;
		while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
		t[j + 1] = k;
	}
	return (n & 1) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

static void push(cs_beacon_t *b, float d)
{
	b->buf[b->head] = d;
	b->head = (b->head + 1) % CS_POS_WIN;
	if (b->n < CS_POS_WIN) {
		b->n++;
	}
}

void cs_pos_add(cs_pos_t *s, uint8_t idx, bool valid, float d, int64_t now_ms)
{
	if (idx >= s->count || !valid || !(d > 0.0f) || d > 50.0f) {
		return;                             /* timeout / garbage: keep the old value */
	}
	cs_beacon_t *b = &s->b[idx];

	/* old buffer → start over */
	if (b->ever && now_ms - b->last_ms > CS_POS_MAX_AGE_MS) {
		b->n = b->head = 0;
	}

	if (b->n >= 3 && fabsf(d - median(b->buf, b->n)) > CS_POS_OUTLIER_M) {
		if (b->rejected > 0.0f && fabsf(d - b->rejected) < CS_POS_OUTLIER_M) {
			/* two consistent "outliers" in a row → it really moved */
			b->n = b->head = 0;
			push(b, b->rejected);
		} else {
			b->rejected = d;            /* single spike, reject */
			return;
		}
	}
	b->rejected = 0.0f;
	push(b, d);
	b->last_ms = now_ms;
	b->ever = true;
}

bool cs_pos_dist(const cs_pos_t *s, uint8_t idx, int64_t now_ms, float *d)
{
	const cs_beacon_t *b = &s->b[idx];
	if (!b->ever || b->n == 0 || now_ms - b->last_ms > CS_POS_MAX_AGE_MS) {
		return false;
	}
	*d = median(b->buf, b->n);
	return true;
}

bool cs_pos_solve(cs_pos_t *s, int64_t now_ms, cs_fix_t *out)
{
	float bx[CS_POS_MAX_BEACONS], by[CS_POS_MAX_BEACONS], r[CS_POS_MAX_BEACONS];
	uint8_t n = 0, stale = 0;

	for (uint8_t i = 0; i < s->count; i++) {
		float d;
		if (cs_pos_dist(s, i, now_ms, &d)) {
			bx[n] = s->b[i].x_mm;
			by[n] = s->b[i].y_mm;
			r[n] = d * 1000.0f;
			/* stale = no fresh measurement in the current cycle */
			if (now_ms - s->b[i].last_ms > CS_POS_STALE_MS) {
				stale++;
			}
			n++;
		}
	}
	if (n < 3) {
		return false;
	}

	/* 1) Linear LS: subtract beacon 0's equation from the rest */
	float a11 = 0, a12 = 0, a22 = 0, c1 = 0, c2 = 0;
	for (uint8_t i = 1; i < n; i++) {
		float ax = 2.0f * (bx[i] - bx[0]);
		float ay = 2.0f * (by[i] - by[0]);
		float rhs = r[0] * r[0] - r[i] * r[i]
			  + bx[i] * bx[i] - bx[0] * bx[0]
			  + by[i] * by[i] - by[0] * by[0];
		a11 += ax * ax; a12 += ax * ay; a22 += ay * ay;
		c1 += ax * rhs; c2 += ay * rhs;
	}
	float det = a11 * a22 - a12 * a12;
	if (fabsf(det) < 1e-6f) {
		return false;                       /* beacons on one line */
	}
	float x = (a22 * c1 - a12 * c2) / det;
	float y = (a11 * c2 - a12 * c1) / det;

	/* 2) Gauss-Newton on the true distances (more accurate than the linearisation) */
	for (int it = 0; it < 5; it++) {
		float h11 = 0, h12 = 0, h22 = 0, g1 = 0, g2 = 0;
		for (uint8_t i = 0; i < n; i++) {
			float dx = x - bx[i], dy = y - by[i];
			float dist = sqrtf(dx * dx + dy * dy);
			if (dist < 1.0f) {
				dist = 1.0f;
			}
			float jx = dx / dist, jy = dy / dist;
			float e = dist - r[i];
			h11 += jx * jx; h12 += jx * jy; h22 += jy * jy;
			g1 += jx * e;   g2 += jy * e;
		}
		float hd = h11 * h22 - h12 * h12;
		if (fabsf(hd) < 1e-9f) {
			break;
		}
		float sx = (h22 * g1 - h12 * g2) / hd;
		float sy = (h11 * g2 - h12 * g1) / hd;
		x -= sx;
		y -= sy;
		if (fabsf(sx) + fabsf(sy) < 0.5f) {
			break;
		}
	}

	float ss = 0;
	for (uint8_t i = 0; i < n; i++) {
		float dx = x - bx[i], dy = y - by[i];
		float e = sqrtf(dx * dx + dy * dy) - r[i];
		ss += e * e;
	}

	/* 3) EMA */
	if (!s->have_pos) {
		s->px = x; s->py = y; s->have_pos = true;
	} else {
		s->px += CS_POS_EMA_ALPHA * (x - s->px);
		s->py += CS_POS_EMA_ALPHA * (y - s->py);
	}

	out->x_mm = s->px;
	out->y_mm = s->py;
	out->resid_mm = sqrtf(ss / n);
	out->used = n;
	out->stale = stale;
	return true;
}
