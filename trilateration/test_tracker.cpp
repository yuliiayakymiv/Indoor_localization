// PC test of the position tracker and the laser height logic (no board needed):
//   g++ -std=c++17 -O2 -I. test_tracker.cpp tracker.cpp trilateration.cpp height_fusion.cpp -o test_tracker
//   ./test_tracker
//
// Simulates the real timing of the mobile node: one beacon every 1.5 s (round-robin),
// distance noise 0.15 m, 5 % reflections (+0.9 m), and a laser at 10 Hz on the drone.
// The tracker is compared with the old firmware filter (median of the last 5 accepted
// distances with the 0.3 m outlier rule, trilateration once per round).

#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <random>
#include <vector>

#include "tracker.hpp"
#include "height_fusion.hpp"

static std::mt19937 rng(42);
static float gauss(float s) { std::normal_distribution<float> n(0.0f, s); return n(rng); }
static float uni() { std::uniform_real_distribution<float> u(0.0f, 1.0f); return u(rng); }

static int failures = 0;
static void check(bool ok, const char* what) {
    printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) failures++;
}

static const Point3D BEACONS[4] = {
    {0.0f, 0.0f, 0.3f}, {2.5f, 0.0f, 0.3f}, {1.25f, 2.17f, 0.3f}, {1.25f, 0.72f, 2.3f}};

static float dist(const Point3D& a, const Point3D& b) {
    return sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

static float measured(const Point3D& truth, int i) {
    float d = dist(truth, BEACONS[i]) + gauss(0.15f);
    if (uni() < 0.05f) d += 0.9f; // reflection
    return d;
}

// Rectangle 1.6 x 1.0 m inside the beacon triangle, driven at `speed` m/s.
static Point3D rect(double t, float speed, float z) {
    const float w = 1.6f, h = 1.0f, x0 = 0.45f, y0 = 0.45f;
    float s = fmodf((float)t * speed, 2 * (w + h));
    if (s < w) return {x0 + s, y0, z};
    s -= w;
    if (s < h) return {x0 + w, y0 + s, z};
    s -= h;
    if (s < w) return {x0 + w - s, y0 + h, z};
    s -= w;
    return {x0, y0 + h - s, z};
}

// ---- the old firmware filter (cs_pos.c), for comparison ----
struct OldBeacon { std::vector<float> buf; float rej = 0; };
static float median(std::vector<float> v) {
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}
static void old_add(OldBeacon& o, float d) {
    if (o.buf.size() >= 3 && fabsf(d - median(o.buf)) > 0.3f) {
        if (o.rej > 0 && fabsf(d - o.rej) < 0.3f) {
            o.buf = {o.rej};
        } else {
            o.rej = d;
            return;
        }
    }
    o.rej = 0;
    o.buf.push_back(d);
    if (o.buf.size() > 5) o.buf.erase(o.buf.begin());
}

struct CarResult { float rms_new, rms_old; int ignored4; bool z_fixed; };

static CarResult run_car(float speed) {
    Tracker::Config cfg;
    cfg.car_z_m = 0.1f;
    Tracker::RangeTracker t;
    t.configure(cfg);
    for (int i = 0; i < 4; i++) t.set_anchor(i, BEACONS[i]);
    t.set_mode(Tracker::Mode::Car);

    OldBeacon old[3];
    Point3D old_pos = {0, 0, 0};
    double ts = 0, se = 0, se_old = 0;
    int n = 0;
    CarResult r = {0, 0, 0, true};

    for (int k = 0; k < 2000; k++) {
        ts += 1.5;
        Point3D truth = rect(ts, speed, cfg.car_z_m);
        // the firmware skips the B4 turn in CAR mode; feed B4 here anyway to check that the
        // tracker ignores it
        if (k % 10 == 0 && t.add_range(3, measured(truth, 3), ts) == Tracker::RangeResult::Ignored) {
            r.ignored4++;
        }
        int b = k % 3;
        float d = measured(truth, b);
        t.add_range(b, d, ts);
        old_add(old[b], d);
        if (b == 2) {
            Anchor a[3];
            bool ok = true;
            for (int i = 0; i < 3; i++) {
                if (old[i].buf.empty()) { ok = false; break; }
                float m = median(old[i].buf), dz = BEACONS[i].z - cfg.car_z_m;
                a[i].pos = BEACONS[i];
                a[i].distance = sqrtf(fmaxf(m * m - dz * dz, 0.0025f));
            }
            if (ok) Trilateration::calculate_2d_position(a, 3, old_pos);
        }
        Tracker::Fix f = t.fix(ts);
        if (k > 30 && f.valid) {
            se += pow(f.pos.x - truth.x, 2) + pow(f.pos.y - truth.y, 2);
            se_old += pow(old_pos.x - truth.x, 2) + pow(old_pos.y - truth.y, 2);
            n++;
            if (fabsf(f.pos.z - cfg.car_z_m) > 1e-4f) r.z_fixed = false;
        }
    }
    r.rms_new = sqrtf(se / n);
    r.rms_old = sqrtf(se_old / n);
    return r;
}

static void test_car() {
    printf("CAR: rectangle, 3 beacons + B4 (ignored), new tracker vs old median filter\n");
    const float speeds[] = {0.0f, 0.05f, 0.1f, 0.2f, 0.3f};
    bool better = true, ign = true, zf = true;
    for (float v : speeds) {
        CarResult r = run_car(v);
        printf("  %.2f m/s: xy RMS %.2f m (old filter %.2f m)\n", v, r.rms_new, r.rms_old);
        if (v > 0.0f && r.rms_new > r.rms_old) better = false;
        if (v == 0.0f && r.rms_new > r.rms_old + 0.03f) better = false;
        if (r.ignored4 != 200) ign = false;
        if (!r.z_fixed) zf = false;
    }
    check(ign, "B4 ignored in CAR mode");
    check(zf, "z stays at the car antenna height");
    check(better, "as precise as the old filter standing still, better when moving");
}

static void test_drone() {
    printf("DRONE: 4 beacons (B4 at 2.3 m), laser at 10 Hz, 0.7 m box on the floor\n");
    Tracker::Config cfg;
    Tracker::RangeTracker t, t_ble;
    t.configure(cfg);
    t_ble.configure(cfg);
    for (int i = 0; i < 4; i++) {
        t.set_anchor(i, BEACONS[i]);
        t_ble.set_anchor(i, BEACONS[i]);
    }
    t.set_mode(Tracker::Mode::Drone);
    t_ble.set_mode(Tracker::Mode::Drone);
    HeightFusion::Estimator hf;

    const float box_x0 = 1.0f, box_x1 = 1.6f, box_h = 0.7f;
    double ts = 0.0, next_range = 0.0;
    int beacon = 0;
    double se_xy = 0, se_z = 0, se_z_ble = 0, se_z_floor = 0;
    int n = 0, n_floor = 0, obstacles = 0, false_obstacles = 0;
    float obstacle_h = 0;

    for (int k = 0; k < 6000; k++) {
        ts += 0.1;
        float x = 1.25f + 0.9f * sinf((float)ts * 0.05f);  // slow sweep over the box
        Point3D truth = {x, 0.9f, 1.2f + 0.2f * sinf((float)ts * 0.03f)};

        if (ts >= next_range) {
            float d = measured(truth, beacon);
            t.add_range(beacon, d, ts);
            t_ble.add_range(beacon, d, ts);
            beacon = (beacon + 1) % 4;
            next_range = ts + 1.5;
        }

        float roll = gauss(0.05f), pitch = gauss(0.05f);
        bool over_box = truth.x > box_x0 && truth.x < box_x1;
        float below = truth.z - (over_box ? box_h : 0.0f);
        float range = below / (cosf(roll) * cosf(pitch)) + gauss(0.01f);

        Tracker::Fix f = t.fix(ts);
        Tracker::Fix fb = t_ble.fix(ts);
        if (f.valid) {
            auto r = hf.update(f.pos.z, range, roll, pitch, ts);
            if (r.use_for_z) t.add_height(r.laser_h_m, hf.config().laser_sigma_m, ts);
            if (r.new_obstacle) {
                if (over_box) { obstacles++; obstacle_h = r.obstacle_h_m; }
                else false_obstacles++;
            }
        }
        if (k > 600 && f.valid && fb.valid) {
            se_xy += pow(f.pos.x - truth.x, 2) + pow(f.pos.y - truth.y, 2);
            se_z += pow(f.pos.z - truth.z, 2);
            se_z_ble += pow(fb.pos.z - truth.z, 2);
            n++;
            if (!over_box) {
                se_z_floor += pow(f.pos.z - truth.z, 2);
                n_floor++;
            }
        }
    }
    float rxy = sqrtf(se_xy / n), rz = sqrtf(se_z / n), rzb = sqrtf(se_z_ble / n);
    float rzf = sqrtf(se_z_floor / n_floor);
    printf("  xy RMS %.2f m; z RMS: BLE only %.2f m, with laser %.3f m (over the floor %.3f m;\n"
           "  over the box the laser is not used for z, as specified)\n", rxy, rzb, rz, rzf);
    printf("  box edges detected: %d (height %.2f m), false obstacles: %d\n",
           obstacles, obstacle_h, false_obstacles);
    check(rxy < 0.35f, "xy RMS < 0.35 m");
    check(rz < rzb && rzf < 0.05f, "laser makes z better than BLE alone (< 5 cm over the floor)");
    check(obstacles >= 3 && fabsf(obstacle_h - box_h) < 0.15f, "box found, height within 15 cm");
    check(false_obstacles == 0, "no false obstacles");
}

static void test_drone_three_beacons() {
    printf("DRONE with only 3 beacons in one plane, no laser: stays above the plane\n");
    Tracker::Config cfg;
    Tracker::RangeTracker t;
    t.configure(cfg);
    for (int i = 0; i < 3; i++) t.set_anchor(i, BEACONS[i]);
    t.set_mode(Tracker::Mode::Drone);
    Point3D truth = {1.2f, 0.8f, 1.5f};
    double ts = 0;
    Tracker::Fix f = {};
    for (int k = 0; k < 60; k++) {
        ts += 1.5;
        t.add_range(k % 3, dist(truth, BEACONS[k % 3]) + gauss(0.1f), ts);
        f = t.fix(ts);
    }
    printf("  z = %.2f m (truth 1.5)\n", f.pos.z);
    check(f.valid && f.pos.z > 0.8f, "z above the beacon plane");
}

static void test_mode_switch() {
    printf("Mode switch CAR -> DRONE keeps the distances\n");
    Tracker::Config cfg;
    Tracker::RangeTracker t;
    t.configure(cfg);
    for (int i = 0; i < 4; i++) t.set_anchor(i, BEACONS[i]);
    Point3D truth = {1.2f, 0.8f, 1.0f};
    double ts = 0;
    for (int k = 0; k < 8; k++) { ts += 1.5; t.add_range(k % 4, dist(truth, BEACONS[k % 4]), ts); }
    t.set_mode(Tracker::Mode::Drone);
    Tracker::Fix f = t.fix(ts);
    check(f.valid && f.used == 4, "position right after the switch, from all 4 beacons");
    check(fabsf(f.pos.x - truth.x) < 0.1f && fabsf(f.pos.y - truth.y) < 0.1f &&
          fabsf(f.pos.z - truth.z) < 0.15f, "3D position correct");
}

int main() {
    test_car();
    test_drone();
    test_drone_three_beacons();
    test_mode_switch();
    printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
