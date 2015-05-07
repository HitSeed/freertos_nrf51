/*

  Port of FreeRTOS to NRF51822 - Pertti Kasanen <pertti.kasanen@hitseed.com>
  6th May 2015

  This demo is based on the earlier port from Shawn Nock <nock@nocko.se> 21 Aug. 2014, the actual port has been rewritten.

  Makefile structure matches the current nRF_SDK_8 and tries to reuse the hte SDK setup as much as possible.

  FreeRTOS 8.2.1 version is used and FreeRTOSConfig.h has quite full FreeRTOS setup intended for the newer nFR51 chips with 32KB of RAM.
  The port.c is based on the FreeRTOS ARM_CM0 code. The main modification to it is the use of a nRF51 soft device timer for the FreeRTOS SysTick.

 */

/* NRF SDK Includes */
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//
void timers_init(void);
void application_timers_start(void);

#define LED_0 10

uint8_t led_status = 0;

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY         ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_SEND_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1 )
/* The rate at which data is sent to the queue.  The 1500ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS                     ( 1500 / portTICK_PERIOD_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH                                        ( 1 )

/* Values passed to the two tasks just to check the task parameter
functionality. */
#define mainQUEUE_SEND_PARAMETER                        ( 0x1111UL )
#define mainQUEUE_RECEIVE_PARAMETER                     ( 0x22UL )
/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/

/* Function called by APP_ERROR_HANDLER via APP_ERROR_CHECK macro if
   err_code isn't NRF_SUCCESS */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {
  for(;;);
}

/* Function called if an assert fails in the softdevice */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/*-----------------------------------------------------------*/

/*
 * Latch the LED that indicates that an error has occurred.
 */
void vMainToggleLED( void );

/*
 * Sets up the hardware used by the demo.
 */
static void prvSetupHardware( void );

/* Tasks */

static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );


/*-----------------------------------------------------------*/

/* Error flag set to pdFAIL if an error is encountered in the tasks/co-routines
defined within this file. */
unsigned portBASE_TYPE uxErrorStatus = pdPASS;

/*-----------------------------------------------------------*/

int main( void )
{

  /* Setup the ports used by the demo and the clock. */
  prvSetupHardware();

  timers_init();
  application_timers_start();

  /* Create the queue. */
  xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

  if( xQueue != NULL )
    {
      /* Start the two tasks as described in the comments at the top of this
	 file. */
      xTaskCreate( prvQueueReceiveTask, /* The function that implements the task. */
		   "Rx", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
		   configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
		   ( void * ) mainQUEUE_RECEIVE_PARAMETER, /* The parameter passed to the task - just to check the functionality. */
		   mainQUEUE_RECEIVE_TASK_PRIORITY,   /* The priority assigned to the task. */
		   NULL ); /* The task handle is not required, so NULL is passed. */
      xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, ( void * ) mainQUEUE_SEND_PARAMETER,
		   mainQUEUE_SEND_TASK_PRIORITY, NULL );

      /* Start the scheduler running the tasks and co-routines just created. */

      vTaskStartScheduler();

      /* Should not get here unless we did not have enough memory to start the
	 scheduler. */
      vMainToggleLED();
      for( ;; );
    }
  return 0;
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
#ifdef USE_LFCLK_RC
  SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION, false);
#else
  SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
#endif // USE_LFCLK_RC
  nrf_gpio_pin_set(LED_0);
  nrf_gpio_cfg_output(LED_0);
  nrf_gpio_pin_clear(LED_0);
}


/*-----------------------------------------------------------*/

void vMainToggleLED( void ) {
  nrf_gpio_pin_toggle(LED_0);
}

/*-----------------------------------------------------------*/

// just for debugging the systick code, this can be removed
//#define DEBUG_SYSTICK
#ifdef DEBUG_SYSTICK
static TickType_t ticks;
static uint16_t _prev = 0;
#define TICK_CHECK 1000
static void _debug_systick() {
    // just for debugging the systick code
    ticks = xTaskGetTickCount();
    if((ticks / TICK_CHECK) > _prev) {
        _prev = ticks / TICK_CHECK; // nice line for a debugger breakpont
    }
    _prev = ticks / TICK_CHECK;
}
#endif // DEBUG_SYSTICK

void prvQueueSendTask( void *pvParameters )
{
TickType_t xNextWakeTime;
const unsigned long ulValueToSend = 100UL;

        /* Check the task parameter is as expected. */
        configASSERT( ( ( unsigned long ) pvParameters ) == mainQUEUE_SEND_PARAMETER );

        /* Initialise xNextWakeTime - this only needs to be done once. */
        xNextWakeTime = xTaskGetTickCount();

        for( ;; )
        {
                /* Place this task in the blocked state until it is time to run again.
                The block time is specified in ticks, the constant used converts ticks
                to ms.  While in the Blocked state this task will not consume any CPU
                time. */
                vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

#ifdef DEBUG_SYSTICK
                // just for debugging the systick code, this can be removed
                _debug_systick();
#endif // DEBUG_SYSTICK

                /* Send to the queue - causing the queue receive task to unblock and
                toggle the LED.  0 is used as the block time so the sending operation
                will not block - it shouldn't need to block as the queue should always
                be empty at this point in the code. */
                xQueueSend( xQueue, &ulValueToSend, 0U );
        }
}
/*-----------------------------------------------------------*/
void prvQueueReceiveTask( void *pvParameters )
{
unsigned long ulReceivedValue;

        /* Check the task parameter is as expected. */
        configASSERT( ( ( unsigned long ) pvParameters ) == mainQUEUE_RECEIVE_PARAMETER );

        for( ;; )
        {
                /* Wait until something arrives in the queue - this task will block
                indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
                FreeRTOSConfig.h. */
                xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

                /*  To get here something must have been received from the queue, but
                is it the expected value?  If it is, toggle the LED. */


                if( ulReceivedValue == 100UL )  {
                        vMainToggleLED();
                        ulReceivedValue = 0UL;
                }
        }
}

