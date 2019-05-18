#include <rtthread.h>

#define EVENT_FAST_LOOP		(1<<0)

static struct rt_timer timer_tremball;
static struct rt_event event_tremball;

static void timer_tremball_update(void* parameter)
{
	rt_event_send(&event_tremball, EVENT_FAST_LOOP);
}

void tremball_loop(void)
{

}

void tremball_entry(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_FAST_LOOP;

	/* initial codes .. */
	
	/* create event */
	res = rt_event_init(&event_tremball, "event_tremball", RT_IPC_FLAG_FIFO);

	/* register timer event */
	rt_timer_init(&timer_tremball, "timer_tremball",
					timer_tremball_update,
					RT_NULL,
					1,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_tremball);
	
	while(1)
	{
		res = rt_event_recv(&event_tremball, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK){
			if(recv_set & EVENT_FAST_LOOP){
				tremball_loop();
			}
		}
	}
}
