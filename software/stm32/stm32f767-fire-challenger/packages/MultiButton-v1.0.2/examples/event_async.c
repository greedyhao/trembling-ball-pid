#include <rtthread.h> 
#include <rtdevice.h> 
#include <drv_gpio.h>
#include "multi_button.h"
#include "pid.h"

static struct button btn_1;
static struct button btn_2;
static uint32_t pos_1 = 258;
static uint32_t pos_2 = 307;
static float kp = 2;
static float ki = 0.9;
static uint32_t dir_1 = 0;
static uint32_t dir_2 = 0;
rt_mailbox_t pwm_1_ctrl_mb;
rt_mailbox_t pwm_2_ctrl_mb;

extern pid_ctrl_t pid_x;
extern pid_ctrl_t pid_y;

#define BUTTON_PIN_1 (GET_PIN(A,0))
#define BUTTON_PIN_2 (GET_PIN(C,13))

static uint8_t button_read_pin_1(void) 
{
    return rt_pin_read(BUTTON_PIN_1); 
}

static uint8_t button_read_pin_2(void) 
{
    return rt_pin_read(BUTTON_PIN_2); 
}

void button_callback_1(void *btn)
{
    uint32_t btn_event_val; 
    
    btn_event_val = get_button_event((struct button *)btn); 
    
    switch(btn_event_val)
    {
    case PRESS_DOWN:
        rt_kprintf("button1 press down\n");
    break; 

    case PRESS_UP: 
        rt_kprintf("button1 press up\n");
    break; 

    case PRESS_REPEAT: 
        rt_kprintf("button1 press repeat\n");
    break; 

    case SINGLE_CLICK: 
        rt_kprintf("button1 single click\n");
        if (dir_1)
        {
            // pos_1++; 
            kp -= (float)0.01;
            ki -= (float)0.1;
            // pid_set_pid(pid_x,kp,pid_x->ki,pid_x->kd);
            pid_set_pid(pid_x,pid_x->kp,ki,pid_x->kd);
            pid_set_pid(pid_y,pid_y->kp,ki,pid_y->kd);
        }
        else
        {
            kp += (float)0.01;
            ki += (float)0.1;
            // pid_set_pid(pid_x,kp,pid_x->ki,pid_x->kd);
            pid_set_pid(pid_x,pid_x->kp,ki,pid_x->kd);
            pid_set_pid(pid_y,pid_y->kp,ki,pid_y->kd);
            // pos_1--;
        }
        
        // rt_mb_send(pwm_1_ctrl_mb, pos_1);
        // rt_mb_send(pwm_1_ctrl_mb, kp);
    break; 

    case DOUBLE_CLICK: 
        rt_kprintf("button1 double click\n");
        dir_1 = ~dir_1;
    break; 

    case LONG_RRESS_START: 
        rt_kprintf("button1 long press start\n");
    break; 

    case LONG_PRESS_HOLD: 
        rt_kprintf("button1 long press hold\n");
        if (dir_1)
        {
            // pos_1++; 
            kp -= (float)0.01;
            pid_set_pid(pid_x,kp,pid_x->ki,pid_x->kd);
        }
        else
        {
            // pos_1--;
            kp += (float)0.01;
            pid_set_pid(pid_x,kp,pid_x->ki,pid_x->kd);
        }
        // rt_mb_send(pwm_1_ctrl_mb, pos_1);
    break; 
    }
    
}

void button_callback_2(void *btn)
{
    uint32_t btn_event_val; 
    
    btn_event_val = get_button_event((struct button *)btn); 
    
    switch(btn_event_val)
    {
    case PRESS_DOWN:
        rt_kprintf("button2 press down\n");
    break; 

    case PRESS_UP: 
        rt_kprintf("button2 press up\n");
    break; 

    case PRESS_REPEAT: 
        rt_kprintf("button2 press repeat\n");
    break; 

    case SINGLE_CLICK: 
        rt_kprintf("button2 single click\n");
        if (dir_2)
        {
            // pos_2++; 
            kp -= (float)0.01;
            pid_set_pid(pid_y,kp,pid_y->ki,pid_y->kd);
        }
        else
        {
            pos_2--;
            kp += (float)0.01;
            pid_set_pid(pid_y,kp,pid_y->ki,pid_y->kd);
        }
        
        // rt_mb_send(pwm_2_ctrl_mb, pos_2);
    break; 

    case DOUBLE_CLICK: 
        rt_kprintf("button2 double click\n");
        dir_2 = ~dir_2;
    break; 

    case LONG_RRESS_START: 
        rt_kprintf("button2 long press start\n");
    break; 

    case LONG_PRESS_HOLD: 
        rt_kprintf("button2 long press hold\n");
        if (dir_2)
        {
            // pos_2++; 
            kp -= (float)0.01;
            pid_set_pid(pid_y,kp,pid_y->ki,pid_y->kd);
        }
        else
        {
            // pos_2--;
            kp += (float)0.01;
            pid_set_pid(pid_y,kp,pid_y->ki,pid_y->kd);
        }
        // rt_mb_send(pwm_2_ctrl_mb, pos_2);
    break; 
    }
    
}

void btn_thread_entry(void* p)
{
    while(1)
    {
        /* 5ms */
        rt_thread_delay(RT_TICK_PER_SECOND/200); 
        button_ticks(); 
    }
}

int multi_button_test(void)
{
    rt_thread_t thread = RT_NULL;

    pwm_1_ctrl_mb = rt_mb_create("pwm_1_ctrl", 128, RT_IPC_FLAG_FIFO);
    pwm_2_ctrl_mb = rt_mb_create("pwm_2_ctrl", 128, RT_IPC_FLAG_FIFO);
    
    /* Create background ticks thread */
    thread = rt_thread_create("btn", btn_thread_entry, RT_NULL, 1024, 10, 10);
    if(thread == RT_NULL)
    {
        return RT_ERROR; 
    }
    rt_thread_startup(thread);

    /* low level drive */
    rt_pin_mode  (BUTTON_PIN_1, PIN_MODE_INPUT); 
    button_init  (&btn_1, button_read_pin_1, PIN_HIGH);
    // button_attach(&btn_1, PRESS_DOWN,       button_callback_1);
    // button_attach(&btn_1, PRESS_UP,         button_callback_1);
    // button_attach(&btn_1, PRESS_REPEAT,     button_callback_1);
    button_attach(&btn_1, SINGLE_CLICK,     button_callback_1);
    button_attach(&btn_1, DOUBLE_CLICK,     button_callback_1);
    // button_attach(&btn_1, LONG_RRESS_START, button_callback_1);
    button_attach(&btn_1, LONG_PRESS_HOLD,  button_callback_1);
    button_start (&btn_1);

    /* low level drive */
    rt_pin_mode  (BUTTON_PIN_2, PIN_MODE_INPUT); 
    button_init  (&btn_2, button_read_pin_2, PIN_HIGH);
    // button_attach(&btn_2, PRESS_DOWN,       button_callback_2);
    // button_attach(&btn_2, PRESS_UP,         button_callback_2);
    // button_attach(&btn_2, PRESS_REPEAT,     button_callback_2);
    button_attach(&btn_2, SINGLE_CLICK,     button_callback_2);
    button_attach(&btn_2, DOUBLE_CLICK,     button_callback_2);
    // button_attach(&btn_2, LONG_RRESS_START, button_callback_2);
    button_attach(&btn_2, LONG_PRESS_HOLD,  button_callback_2);
    button_start (&btn_2);

    return RT_EOK; 
}
INIT_APP_EXPORT(multi_button_test); 
