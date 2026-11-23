/* extruder.comp's pressure advance against a step generator that cannot go
 * faster than a CAN node (20 kHz at 705.5 steps/mm = 28.3 mm/s) and reports
 * its position one servo period late. LinuxCNC's following error is
 * motor-pos-cmd - motor-pos-fb (the component's pos-fb-out). */
#include <math.h>
#include <stdio.h>

#define RTAPI_MSG_ERR 1
static void rtapi_print_msg(int level, const char *fmt, ...) { (void)level; (void)fmt; }

/* pins / params / variables, as halcompile names them */
static double current_temp = 210, requested_temp, pid_pwm_in, vel_cmd, pos_cmd_in, pos_fb_in;
static double flow_percent = 100, pressure_advance, vel_x, vel_y;
static int machine_enabled = 1, cold_extrude_enable = 1, homing;
static double min_valid_temp = 5, max_valid_temp = 290, min_extrude_temp = 170, heatsink_fan_temp = 50;
static double watchdog_time = 20, min_extrude_vel = 0.000001, min_extrude_time = 0.05, pa_smooth_time = 0.040;
static double max_velocity, min_xy_vel = 0.1;
static double safe_target_temp, safe_pwm_out, pos_cmd_out, pos_fb_out, pa_offset;
static int heatsink_fan, temp_ready, cold_extrude, fault_range, fault_watchdog;
static double timer, saved_temp, moving_time, flow_base_in, flow_base_out, flow_factor = 1.0, pa_last_in, pa_value;
static int cold_reported, flow_started;
static const double fperiod = 0.001;

static void step(void) {
#include "body.inc"

#define NODE_MAX 28.3      /* mm/s: 20 kHz / 705.5 steps/mm */
#define JOINT_MAX 25.5     /* the extruder joint's MAX_VELOCITY: 90 % of it */

static int failures;
static void check(int ok, const char *what) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

struct result { double max_ferror, max_cmd_speed, max_pa; };

/* E (and X) commanded by LinuxCNC as trapezoids; the node follows the
 * component's pos-cmd-out at most NODE_MAX, feedback one period late. */
static struct result run(double e_dist, double e_speed, double xy_speed, double k, double vmax_param) {
    struct result r = {0, 0, 0};
    double e_cmd = 0, e_vel = 0, node_pos = 0, node_prev = 0, out_prev = 0;
    const double accel = 2000;                     /* mm/s^2 */
    pos_cmd_in = 0; pos_fb_in = 0; flow_started = 0; pa_value = 0; pa_last_in = 0;
    pressure_advance = k; max_velocity = vmax_param;
    for (int i = 0; i < 2000; i++) {
        double left = e_dist - e_cmd;
        double v_target = left > 1e-9 ? fmin(e_speed, sqrt(2 * accel * left)) : 0;
        if (e_vel < v_target) e_vel = fmin(v_target, e_vel + accel * fperiod);
        else e_vel = fmax(v_target, e_vel - accel * fperiod);
        e_cmd = fmin(e_dist, e_cmd + e_vel * fperiod);
        vel_x = left > 1e-9 ? xy_speed : 0; vel_y = 0;
        pos_cmd_in = e_cmd; vel_cmd = e_vel;
        pos_fb_in = node_prev;                    /* one period late */
        step();
        double ferror = fabs(pos_cmd_in - pos_fb_out);
        if (ferror > r.max_ferror) r.max_ferror = ferror;
        double cmd_speed = fabs(pos_cmd_out - out_prev) / fperiod;
        if (i > 0 && cmd_speed > r.max_cmd_speed) r.max_cmd_speed = cmd_speed;
        if (pa_offset > r.max_pa) r.max_pa = pa_offset;
        out_prev = pos_cmd_out;
        node_prev = node_pos;
        double d = pos_cmd_out - node_pos, lim = NODE_MAX * fperiod;
        node_pos += d > lim ? lim : d < -lim ? -lim : d;
    }
    return r;
}

int main(void) {
    struct result r;

    /* before: PA on every forward move, no speed limit (vel_x forced) */
    r = run(0.8, 35, 1000, 0.04, 0);
    printf("  old unretract 0.8 mm @ 35 mm/s, K 0.04: max ferror %.3f mm, stepgen asked %.1f mm/s\n", r.max_ferror, r.max_cmd_speed);
    check(r.max_ferror > 0.5, "the old behaviour lags the node far (the following error)");

    /* now: an unretract (no X / Y move) gets no PA */
    r = run(0.8, JOINT_MAX, 0, 0.04, JOINT_MAX);
    printf("  unretract 0.8 mm @ %.1f mm/s, K 0.04: max ferror %.3f mm, PA %.3f mm\n", JOINT_MAX, r.max_ferror, r.max_pa);
    check(r.max_pa == 0.0, "no pressure advance on an extruder-only move");
    check(r.max_ferror < 0.1, "the node keeps up with an unretract within MAX_VELOCITY");

    /* extruding during an X / Y move: PA applies, within the limit */
    r = run(5.0, 6, 150, 0.04, JOINT_MAX);
    printf("  extrusion @ 6 mm/s during a move, K 0.04: PA %.3f mm, ferror %.3f mm\n", r.max_pa, r.max_ferror);
    check(r.max_pa > 0.2, "pressure advance applies while X / Y move");
    check(r.max_ferror < 0.1, "and the node follows it");

    /* a fast extrusion: the offset only uses the speed left below max-velocity */
    r = run(10.0, 24, 150, 0.04, JOINT_MAX);
    printf("  extrusion @ 24 mm/s during a move, K 0.04: stepgen asked %.2f mm/s (limit %.1f)\n", r.max_cmd_speed, JOINT_MAX);
    check(r.max_cmd_speed <= JOINT_MAX + 1e-6, "PA never pushes the stepgen past max-velocity");
    check(r.max_ferror < 0.1, "so the node keeps up");

    return failures ? 1 : 0;
}
