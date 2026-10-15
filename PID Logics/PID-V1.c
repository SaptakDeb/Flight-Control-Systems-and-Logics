#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;   /* output limits */
    float i_limit;            /* integrator clamp */
    float d_alpha;            /* D low-pass, 0..1 (smaller = smoother) */
    float integ;
    float prev_meas;
    float d_filt;
    uint8_t first;
} PID;

void pid_reset(PID *p)
{
    p->integ = 0.0f; p->prev_meas = 0.0f; p->d_filt = 0.0f; p->first = 1;
}

void pid_init(PID *p, float kp, float ki, float kd,
              float out_min, float out_max, float i_limit, float d_alpha)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_min = out_min; p->out_max = out_max;
    p->i_limit = i_limit; p->d_alpha = d_alpha;
    pid_reset(p);
}

float pid_update(PID *p, float setpoint, float meas, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float err = setpoint - meas;
    float P = p->kp * err;

    float d_raw = 0.0f;
    if (!p->first) d_raw = -(meas - p->prev_meas) / dt;
    p->first = 0;
    p->prev_meas = meas;
    p->d_filt += p->d_alpha * (d_raw - p->d_filt);
    float D = p->kd * p->d_filt;

    p->integ += p->ki * err * dt;
    if (p->integ >  p->i_limit) p->integ =  p->i_limit;
    if (p->integ < -p->i_limit) p->integ = -p->i_limit;

    float out = P + p->integ + D;

    if (out > p->out_max) { p->integ -= (out - p->out_max); out = p->out_max; }
    if (out < p->out_min) { p->integ += (p->out_min - out); out = p->out_min; }

    return out;
}

#ifdef PID_DEMO
#include <stdio.h>
/* Demo: hold a toy first-order "bank angle" plant at a 20 deg setpoint. */
int main(void)
{
    PID pid;
    pid_init(&pid, 0.03f, 0.01f, 0.004f, -1.0f, 1.0f, 0.3f, 0.2f);
    float roll = 0.0f, dt = 0.01f;
    for (int i = 0; i <= 600; i++) {
        float u = pid_update(&pid, 20.0f, roll, dt);
        roll += (u * 60.0f - roll) * 3.0f * dt;     /* toy plant */
        if (i % 60 == 0) printf("t=%4.1f roll=%6.2f u=%5.2f\n", i * dt, roll, u);
    }
    return 0;
}
#endif
