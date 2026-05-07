#include "mission.h"
#include "config.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const char *phase_name(mission_phase_t p) {
    switch (p) {
        case MISSION_INIT:       return "INIT";
        case MISSION_PHASE1_GO:  return "PHASE1_GO";
        case MISSION_PHASE1_OK:  return "PHASE1_OK";
        case MISSION_PHASE2_GO:  return "PHASE2_GO";
        case MISSION_PHASE2_OK:  return "PHASE2_OK";
        case MISSION_DONE:       return "DONE";
        default:                 return "?";
    }
}

void compute_phase1_targets(int n, double *tx, double *ty) {
    if (n <= 0) return;
    double r = PHASE1_TARGET_RADIUS;
    for (int i = 0; i < n; i++) {
        double theta = 2.0 * M_PI * (double)i / (double)n;
        tx[i] = PHASE1_CENTER_X + r * cos(theta);
        ty[i] = PHASE1_CENTER_Y + r * sin(theta);
    }
}

static double dist2d(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

int check_phase1_complete(const drone_entry_t *snap, int n) {
    /* 모든 드론이 (0, 100) 으로부터 30m 이내 */
    for (int i = 0; i < n; i++) {
        double d = dist2d(snap[i].x, snap[i].y,
                          PHASE1_CENTER_X, PHASE1_CENTER_Y);
        if (d > PHASE1_RADIUS + VALIDATION_TOL) return 0;
    }
    /* 모든 드론 쌍이 10m 이상 떨어짐 */
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double d = dist2d(snap[i].x, snap[i].y,
                              snap[j].x, snap[j].y);
            if (d < PHASE1_MIN_GAP - VALIDATION_TOL) return 0;
        }
    }
    return 1;
}
