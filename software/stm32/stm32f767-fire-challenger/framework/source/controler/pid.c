#include "pid.h"
#include <math.h>
// #include "arm_math.h"

// #define abs(s) ((s) < 0 ? -(s):(s))

// void pid_s_fun(pid_ctrl_t pid)
// {
//     int k = 0;
//     sin(2*M_PI*(pid->ts));
// }

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

    dout = pid->kp * pid->prev_val.x_1 + pid->ki * pid->prev_val.x_2 + pid->kd * pid->prev_val.x_3;
    pid->out_inc = pid->prev_val.out_inc_1 + dout;
    pid->out = pid->out_bal + pid->out_inc;
    
    // /* 积分分离 */
    if (fabs(pid->error) >= 50)
    {
        pid->ki = (float)0;
    }
    // else if (fabs(pid->error) >= 30 && fabs(pid->error) < 50)
    // {
    //     pid->ki = (float)0.1;
    // }
    // else if (fabs(pid->error) >= 5 && fabs(pid->error) < 30)
    // {
    //     pid->ki = (float)0.5;
    // }
    else
    {
        pid->ki = (float)0.05;
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

    pid->prev_val.out_inc_1 = pid->out_inc;
    pid->prev_val.error_last_2 = pid->prev_val.error_last_1;
    pid->prev_val.error_last_1 = pid->error;
}

void pid_compute_N(pid_ctrl_t pid, float input)
{
    float wadd = 0;
    float w11 = 0;
    float w22 = 0;
    float w33 = 0;

    pid->error = pid->dest - input;
    // printf("%f %f\n", input, pid->error);

    pid->kp = pid->prev_val.kp_1 + pid->xitep*pid->error*(pid->prev_val.out_inc_1)*(2*pid->error-pid->prev_val.out_inc_1);
    pid->ki = pid->prev_val.ki_1 + pid->xitei*pid->error*(pid->prev_val.out_inc_1)*(2*pid->error-pid->prev_val.error_last_1);
    pid->kd = pid->prev_val.kd_1 + pid->xited*pid->error*(pid->prev_val.out_inc_1)*(2*pid->error-pid->prev_val.error_last_1);

    // printf("%f %f %f %f\n",input,pid->kp,pid->ki,pid->kd);
    // printf("%f %f %f %f %f\n",pid->prev_val.kp_1,pid->xitep,error,pid->prev_val.out_inc_1,pid->prev_val.out_inc_1);

    pid->prev_val.x_1 = pid->error - pid->prev_val.error_last_1;
    pid->prev_val.x_2 = pid->error;
    pid->prev_val.x_3 = pid->error - 2*pid->prev_val.error_last_1 + pid->prev_val.error_last_2;

    // wadd = absf(pid->kp) + absf(pid->ki) + absf(pid->kd);
    wadd = fabs(pid->kp) + fabs(pid->ki) + fabs(pid->kd);
    // wadd = (pid->kp) + (pid->ki) + (pid->kd);
    // printf("%f %f %f %f\n",wadd,pid->kp,pid->ki,pid->kd);
    w11 = pid->kp / wadd;
    w22 = pid->ki / wadd;
    w33 = pid->kd / wadd;
    // printf("%f %f %f\n",w11,w22,w33);

    pid->out_inc = pid->prev_val.out_inc_1 + pid->K * (w11*pid->prev_val.x_1 + w22*pid->prev_val.x_2 + w33*pid->prev_val.x_3);
    /* out put limit */
    if (pid->out_inc >= 200)
    {
        pid->out_inc = 200;
    }
    if (pid->out_inc <= -200)
    {
        pid->out_inc = -200;
    }
    
    pid->out = pid->out_bal + pid->out_inc;
    // printf("%f %f %f\n", pid->out_inc, pid->out, pid->out_bal);
    /* out put limit */
    if (pid->out >= pid->pid_limit.out_max)
    {
        pid->out = pid->pid_limit.out_max;
    }
    if (pid->out <= pid->pid_limit.out_min)
    {
        pid->out = pid->pid_limit.out_min;
    }

    pid->prev_val.out_inc_1 = pid->out_inc;
    pid->prev_val.error_last_2 = pid->prev_val.error_last_1;
    pid->prev_val.error_last_1 = pid->error;

    pid->prev_val.kp_1 = pid->kp;
    pid->prev_val.ki_1 = pid->ki;
    pid->prev_val.kd_1 = pid->kd;
}

void pid_set(pid_ctrl_t pid, int dest, int bal)
{
    pid->dest = dest;
    pid->out_bal = bal;
}

void pid_set_pid(pid_ctrl_t pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_init(pid_ctrl_t pid)
{
    pid->ts = (float)0.12;
    pid->kp = (float)1.0;
    pid->kd = (float)0.15;
    pid->ki = (float)0.05;

    pid->prev_val.x_1 = (float)0.1;
    pid->prev_val.x_2 = (float)0.1;
    pid->prev_val.x_3 = (float)0.1;
    pid->prev_val.kp_1 = (float)0.50;
    pid->prev_val.ki_1 = (float)0.0;
    pid->prev_val.kd_1 = (float)0.03;

    pid->xitep = (float)0.25;
    pid->xitei = (float)0.01;
    pid->xited = (float)0.05;
    pid->K = (float)0.18;

    pid->pid_limit.out_max = PID_OUT_MAX;
    pid->pid_limit.out_min = PID_OUT_MIN;
    pid->pid_limit.intg_max = 2;
    pid->pid_limit.intg_min = -2;
    pid->pid_limit.intg_norm = 0.9;

    pid->prev_val.x_1 = (float)1.0;
    pid->prev_val.x_2 = (float)0.0;
    pid->prev_val.x_3 = (float)0.0;

    pid->error = (float)0.0;
    pid->prev_val.error_last_1 = (float)0.0;
    pid->prev_val.error_last_2 = (float)0.0;

    pid->in = (float)0.0;
    pid->dest = 0;
    pid->out_bal = 300;
    pid->out = (float)250;
    // pid->prev_val.y_1 = 0;
    pid->prev_val.out_inc_1 = (float)20;
}


static void NeureLearningRules(NEURALPID *vPID,float zk,float uk,float *xi)
{
  vPID->wi=vPID->wi+vPID->ki*zk*uk*xi[0];
  vPID->wp=vPID->wp+vPID->kp*zk*uk*xi[1];
  vPID->wd=vPID->wd+vPID->kd*zk*uk*xi[2];
}

/* 神经网络参数自整定PID控制器，以增量型方式实现                              */
/* NEURALPID vPID，神经网络PID对象变量，实现数据交换与保存                    */
/* float pv，过程测量值，对象响应的测量数据，用于控制反馈                     */
void NeuralPID(NEURALPID *vPID,float pv)
{
  float x[3];
  float w[3];
  float sabs;
  float error;
  float result;
  float deltaResult;

  error=vPID->setpoint-pv;

  result=vPID->result;
  if(fabs(error)>vPID->deadband)
  {
    x[0]=error;
    x[1]=error-vPID->lasterror;
    x[2]=error-vPID->lasterror*2+vPID->preerror;

    sabs=fabs(vPID->wi)+fabs(vPID->wp)+fabs(vPID->wd);

    w[0]=vPID->wi/sabs;
    w[1]=vPID->wp/sabs;
    w[2]=vPID->wd/sabs;

    deltaResult=(w[0]*x[0]+w[1]*x[1]+w[2]*x[2])*vPID->kcoef;
    }
  else
  {
    deltaResult=0;
  }

  result=result+deltaResult;

  if(result>vPID->maximum)
  {
    result=vPID->maximum;
  }

  if(result<vPID->minimum)
  {
    result=vPID->minimum;
  }

  vPID->result=result;

  vPID->output=(vPID->result-vPID->minimum)*100/(vPID->maximum-vPID->minimum);

  //单神经元学习
  NeureLearningRules(vPID,error,result,x);
  vPID->preerror=vPID->lasterror;
  vPID->lasterror=error;
}

void n_pid_set(NEURALPID *pid, float sp)
{
    pid->maximum = PID_OUT_MAX;
    pid->minimum = PID_OUT_MIN;
    pid->setpoint = sp;
    pid->deadband = (PID_OUT_MAX-PID_OUT_MIN)*0.0005;
    pid->kcoef = 0.04;
    pid->kp = 0.4;
    pid->ki = 0.35;
    pid->kd = 0.4;
    pid->lasterror = 0;
    pid->preerror = 0;
    pid->wp = 0.10;
    pid->wi = 0.10;
    pid->wd = 0.10;
}
