#include <rtthread.h>
#include <rtdef.h>
#include <stdlib.h>
// #include "lvgl.h" 
#include "controller.h"

#define FRM_DEBUG
#define LOG_TAG             "frm.semsor"
#include <frm_log.h>

#define CAM_DEVICE_NAME       "uart2"

static rt_device_t camera;
// static uint32_t adc_value;
// static lv_obj_t * my_label;

static char uart_rx_buffer[32];
static int uart_pos = 0;

// static void label_refresher_task(void * p)
// {
//     // static uint32_t prev_value = 0;

//     // if(prev_value != adc_value) {

//     //     if(lv_obj_get_screen(my_label) == lv_scr_act()) {
//     //         char buf[32];
//     //         sprintf(buf, "Voltage: %d", adc_value);
//     //         lv_label_set_text(my_label, buf);
//     //     }
//     //     prev_value = adc_value;
//     // }
//     lv_label_set_text(my_label, uart_rx_buffer);
// }

static rt_err_t camera_rx_ind(rt_device_t dev, rt_size_t size)
{
	char ch;

    rt_device_read(dev, -1, &ch, size);
	if (ch != 'e')
	{
		uart_rx_buffer[uart_pos] = ch;
		uart_pos++;
	}
	else
	{
		uart_rx_buffer[uart_pos] = ch;
		// rt_memcpy(camera_msg_str, uart_rx_buffer, 32);
		// // rt_kprintf("%s\n",camera_msg_str);
		// camera_msg_update(camera_msg_str);
		camera_msg_update(uart_rx_buffer);

		uart_pos = 0;
	}
	
    return RT_EOK;
}

int device_sensor_init(void)
{
    // rt_err_t res = RT_EOK;

    camera = rt_device_find(CAM_DEVICE_NAME);
	if(camera == RT_NULL)
	{
		LOG_E("can't find camera device");
		return RT_EEMPTY;
	}
    rt_device_set_rx_indicate(camera, camera_rx_ind);

    // while (!rt_thread_find("lv_demo"))
    // {
    //     my_label = lv_label_create(lv_scr_act(), NULL);
    //     lv_task_create(label_refresher_task, 100, LV_TASK_PRIO_MID, NULL);
    //     rt_thread_delay(100);
    // }

    rt_device_open(camera, RT_DEVICE_FLAG_INT_RX);

    return RT_EOK;
}
INIT_APP_EXPORT(device_sensor_init);
