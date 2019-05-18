/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-15     greedyhao    first version
 */

#include <rtthread.h>
#include "pwm_gen.h"
#include "pca9685.h"

// #define FRM_DEBUG
#define LOG_TAG             "frm.pca9685"
#include <frm_log.h>

#define I2C_BUS    "i2c3"

#define EVENT_CN_PWM_LOOP		(1<<0)

static struct rt_timer timer_cntl_pwm;
static struct rt_event event_cntl_pwm;

static int pwm_1 = 0;
static int pwm_2 = 0;

static void timer_cntl_pwm_update(void* parameter)
{
	rt_event_send(&event_cntl_pwm, EVENT_CN_PWM_LOOP);
}

void cntl_set_pwm(int num, int pwm)
{
    if (num == 0)
    {
        pwm_1 = pwm;
    }
    else if (num == 1)
    {
        pwm_2 = pwm;
    }
    else
    {
        
    }
}

void cntl_pwm_loop(pca9685_device_t dev)
{
	pca9685_set_pwm(dev, 0, 0, pwm_1);
    pca9685_set_pwm(dev, 14, 0, pwm_1);
    pca9685_set_pwm(dev, 1, 0, pwm_2);
    pca9685_set_pwm(dev, 15, 0, pwm_2);
}

static void pwm_gen_entry(void *p)
{
    rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_CN_PWM_LOOP;
    pca9685_device_t dev = RT_NULL;

    dev = pca9685_init(I2C_BUS, RT_NULL);

    if (dev == RT_NULL)
        goto _exit;

    /* create event */
	res = rt_event_init(&event_cntl_pwm, "event_cntl_pwm", RT_IPC_FLAG_FIFO);

	/* register timer event */
	rt_timer_init(&timer_cntl_pwm, "timer_cntl_pwm",
					timer_cntl_pwm_update,
					RT_NULL,
					6,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_cntl_pwm);

    while(1)
	{
		res = rt_event_recv(&event_cntl_pwm, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK){
			if(recv_set & EVENT_CN_PWM_LOOP){
				cntl_pwm_loop(dev);
			}
		}
	}

_exit:    
    pca9685_deinit(dev);
}

int pwm_gen_init(void)
{
    rt_thread_t thread = RT_NULL;
    thread = rt_thread_create("pwm_gen", pwm_gen_entry, RT_NULL, 2*512, 14, 10);

    if (thread == RT_NULL)
    {
        return RT_ERROR;
    }
    rt_thread_startup(thread);

	return RT_EOK;
}
INIT_APP_EXPORT(pwm_gen_init);

