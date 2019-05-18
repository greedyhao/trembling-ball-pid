#ifndef PID_H_
#define PID_H_

#define PID_OUT_MAX 105
#define PID_OUT_MIN 405

typedef struct
{
    float x_1;
    float x_2;
    float x_3;

    float error_last_1;
    float error_last_2;

    // float y_1;
    float u_1;
} pid_prev_val;

typedef struct
{
    int out_max;
    int out_min;

    int intg_max;
    int intg_min;
    int intg_norm;
} pid_argv_lim;

struct pid_ctrl
{
    float ts;
    float kp;
    float ki;
    float kd;

    int dest;
    float in;
    float out;
    float error;

    
    pid_argv_lim pid_limit;
    pid_prev_val prev_val;
};

typedef struct pid_ctrl * pid_ctrl_t;

void pid_init(pid_ctrl_t pid);
void pid_compute(pid_ctrl_t pid, float error);

#endif
