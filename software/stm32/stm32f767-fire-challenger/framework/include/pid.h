#ifndef PID_H_
#define PID_H_

#define PID_OUT_MAX 500
#define PID_OUT_MIN 110
// #define PID_OUT_BAL_X 262
// #define PID_OUT_BAL_Y 243
#define PID_OUT_BAL_X 250
#define PID_OUT_BAL_Y 250

typedef struct
{
    float x_1;
    float x_2;
    float x_3;

    float kp_1;
    float ki_1;
    float kd_1;

    float error_last_1;
    float error_last_2;

    // float y_1;
    float out_inc_1;
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

    float xitep;
    float xitei;
    float xited;
    float K;

    int dest;
    float in;
    float out_inc;
    int out_bal;
    float out;
    float error;

    pid_argv_lim pid_limit;
    pid_prev_val prev_val;
};

typedef struct pid_ctrl * pid_ctrl_t;

/*定义结构体和公用体*/
typedef struct
{
  float setpoint;               /*设定值*/
  float kcoef;                  /*神经元输出比例*/
  float kp;                     /*比例学习速度*/
  float ki;                     /*积分学习速度*/
  float kd;                     /*微分学习速度*/
  float lasterror;              /*前一拍偏差*/
  float preerror;               /*前两拍偏差*/
  float deadband;               /*死区*/
  float result;                 /*输出值*/
  float output;                 /*百分比输出值*/
  float maximum;                /*输出值的上限*/
  float minimum;                /*输出值的下限*/
  float wp;                     /*比例加权系数*/
  float wi;                     /*积分加权系数*/
  float wd;                     /*微分加权系数*/
}NEURALPID;

void pid_init(pid_ctrl_t pid);
void pid_compute(pid_ctrl_t pid, float error);
void pid_compute_N(pid_ctrl_t pid, float error);
void pid_set(pid_ctrl_t pid, int dest, int bal);
void pid_set_pid(pid_ctrl_t pid, float kp, float ki, float kd);

void n_pid_set(NEURALPID *pid, float sp);
void NeuralPID(NEURALPID *vPID,float pv);

#endif
