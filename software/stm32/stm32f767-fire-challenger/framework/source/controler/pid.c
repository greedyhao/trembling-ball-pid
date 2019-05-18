#include "pid.h"

/**
 * @brief 
 * 
 * dout = kp(e-e_1)+ki*e+kd(e-2*e_1+e_2)
 * 
 * @param pid 
 */
void pid_compute(pid_ctrl_t pid, float error)
{
    float dout;

    dout = pid->kp * pid->prev_val.x_1 + pid->ki * pid->prev_val.x_2 + pid->ki * pid->prev_val.x_3;
    pid->out = pid->prev_val.u_1 + dout;

    if (pid->error >= pid->pid_limit.intg_max)
    {
        pid->ki = (float)0;
    }
    else
    {
        pid->ki = pid->pid_limit.intg_norm;
    }

    /* out put limit */
    if (pid->out >= pid->pid_limit.out_max)
    {
        pid->out = pid->pid_limit.out_max;
    }
    if (pid->out <= pid->pid_limit.out_min)
    {
        pid->out = pid->pid_limit.out_min;
    }

    pid->error = error;

    /* calculating p i d */
    pid->prev_val.x_1 = pid->error - pid->prev_val.error_last_1;
    pid->prev_val.x_2 = pid->error;
    pid->prev_val.x_3 = pid->error - 2*pid->prev_val.error_last_1 + pid->prev_val.error_last_2;

    pid->prev_val.u_1 = pid->out;
    pid->prev_val.error_last_2 = pid->prev_val.error_last_1;
    pid->prev_val.error_last_1 = pid->error;
}

void pid_init(pid_ctrl_t pid)
{
    pid->ts = (float)0.12;
    pid->kp = (float)8.0;
    pid->kd = (float)1.0;
    pid->ki = (float)0.0;

    pid->pid_limit.out_max = PID_OUT_MAX;
    pid->pid_limit.out_min = PID_OUT_MIN;


    pid->prev_val.x_1 = (float)0.0;
    pid->prev_val.x_2 = (float)0.0;
    pid->prev_val.x_3 = (float)0.0;

    pid->error = (float)0.0;
    pid->prev_val.error_last_1 = (float)0.0;
    pid->prev_val.error_last_2 = (float)0.0;

    pid->in = (float)0.0;
    pid->dest = (float)0.0;
    pid->out = (float)0.0;
    // pid->prev_val.y_1 = 0;
    pid->prev_val.u_1 = (float)305.0;
}
