/*

  Port of FreeRTOS to NRF51822 - Pertti Kasanen <pertti.kasanen@hitseed.com>
  6th May 2015

  FreeRTOS 8.2.1 version is used and FreeRTOSConfig.h has quite full FreeRTOS setup intended for the newer nFR51 chips with 32KB of RAM.
  The port.c is based on the FreeRTOS ARM_CM0 code. The main modification to it is the use of a nRF51 soft device timer for the FreeRTOS SysTick.

 */

/* NRF SDK Includes */
#include "softdevice_handler.h"
#include "app_timer.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"


/* Timers */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define SYSTICK_INTERVAL                 APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)  /**< Systick interval (ticks) 10 ms. */

#define BSP_APP_TIMERS_NUMBER            2
#define APP_TIMER_MAX_TIMERS             (6+BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

static app_timer_id_t                    m_systick_timer_id;


static void systick_timeout_handler(void * p_context)
{
    void xPortSysTickHandler( void );
    UNUSED_PARAMETER(p_context);

    xPortSysTickHandler();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */


void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_systick_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                systick_timeout_handler);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for starting application timers.
 */
void application_timers_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_systick_timer_id, SYSTICK_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}


void vApplicationIdleHook( void ) {
  uint32_t err_code = sd_app_evt_wait();
    /*
       This signals the softdevice handler that we want the CPU to
       sleep until an event/interrupt occurs. During this time the
       softdevice will do what it needs to do; in our case: send
       adverts
    */
    APP_ERROR_CHECK(err_code);
}

void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed char *pcTaskName ) {
//    Log_Error("Application stack overflow: %s !", pcTaskName);

        while(1);
}

void vApplicationMallocFailedHook() {
//    Log_Error("Application malloc failed!");

        while(1);
}
