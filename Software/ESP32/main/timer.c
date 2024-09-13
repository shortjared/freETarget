/*-------------------------------------------------------
 *
 * file: timer_ISR.c
 *
 * Timer interrupt file
 *
 *-------------------------------------------------------
 *
 * The timer interrupt is used to generate internal timers
 * and poll the sensor inputs looking for shot detection
 *
 * See:
 * https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/timer.html
 *
 * ----------------------------------------------------*/
#include "stdbool.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freETarget.h"
#include "diag_tools.h"
#include "driver/gpio.h"
#include "json.h"
#include "esp_attr.h"

/*
 * Definitions
 */
#define FREQUENCY       1000ul // 1000 Hz
#define N_TIMERS        32     // Keep space for 32 timers
#define PORT_STATE_IDLE 0      // There are no sensor inputs
#define PORT_STATE_WAIT 1      // Some sensor inputs are present, but not all
#define PORT_STATE_DONE 2      // All of the inmputs are present

#define MAX_WAIT_TIME   10 // Wait up to 10 ms for the input to arrive
#define MAX_RING_TIME   50 // Wait 50 ms for the ringing to stop

#define TICK_10ms       1                 // vTaskDelay in 10 ms
#define BAND_100ms      (TICK_10ms * 10)  // vTaskDelay in 100 ms
#define BAND_500ms      (TICK_10ms * 50)  // vTaskDelay in 500 ms
#define BAND_1000ms     (TICK_10ms * 100) // vTaskDelay in 1000 ms

/*
 * Local Variables
 */
static volatile unsigned long *timers[N_TIMERS]; // Active timer list
unsigned int                   isr_state;        // What sensor state are we in
static volatile unsigned long  isr_timer;        // Interrupt timer

/*
 *  Function Prototypes
 */
static bool __attribute__((section(".iram1")))
freeETarget_timer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

/*-----------------------------------------------------
 *
 * @function: freeETarget_timer_init
 *
 * @brief:    Initialize the timer interrupt
 *
 * @return:   None
 *
 *-----------------------------------------------------
 *
 * The FreeETarget software uses the FreeRTOS system calls
 * to generate the cycle times needed to run the software
 * Unfortunatly, the FreeRTOS cycle time is 10 ms which is
 * too slow (infrequent) to manage the shot sensors
 * correctly.  For this reason the sensor polling is done
 * by a 1 ms timer interrupt directly from the operating
 * system
 *
 *-----------------------------------------------------*/
#include "freETarget.h"
#include "timer.h"
#include "mfs.h"
#include "token.h"

#define TIMER_DIVIDER (16)                   //  Hardware timer clock divider
#define TIMER_SCALE   (1000 / TIMER_DIVIDER) // convert counter value to seconds
#define ONE_MS        (80 * TIMER_SCALE)     // 1 ms timer interrupt

// Configure the GPTimer
gptimer_config_t timer_config = {
    .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
    .direction     = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, // 1MHz, 1 tick=1us
};

// Create a new GPTimer instance
gptimer_handle_t gptimer = NULL;

void freeETarget_timer_init(void)
{
    DLT(DLT_CRITICAL, printf("freeETarget_timer_init()");)

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // Set up the alarm callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = freeETarget_timer_isr_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // Configure the alarm
    gptimer_alarm_config_t alarm_config = {
        .reload_count               = 0,
        .alarm_count                = ONE_MS, // Assuming ONE_MS is defined elsewhere
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    // Enable and start the timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // Initialize other variables
    timer_new(&isr_timer, 0);
    shot_in  = 0;
    shot_out = 0;

    // Timer running, return
    return;
}

void freeETarget_timer_pause(void)
{
    ESP_ERROR_CHECK(gptimer_stop(gptimer));
}

void freeETarget_timer_start(void)
{
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

/*-----------------------------------------------------
 *
 * @function: freeETarget_timer_isr_callback
 *
 * @brief:    High speed synchronous task
 *
 * @return:   None
 *
 *-----------------------------------------------------
 *
 * This task is called every 1 ms from the timer
 * interrupt
 *
 * Timer 1 samples the inputs and when all of the
 * sendor inputs are present, the counters are
 * read and made available to the software
 *
 * There are three data aquisition states
 *
 * IDLE - No inputs are present
 * WAIT - Inputs are present, but we have to wait
 *        for all of the inputs to be present or
 *        timed out
 * DONE - We have read the counters but need to
 *        wait for the ringing to stop
 *
 *
 *-----------------------------------------------------*/
static bool __attribute__((section(".iram1")))
freeETarget_timer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t   high_task_awoken = pdFALSE;
    unsigned int pin; // Value read from the port

    IF_NOT(IN_OPERATION)
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR

    // Decide what to do based on what inputs are present
    pin = is_running(); // Read in the RUN bits

    // Read the shot based on the ISR state
    switch (isr_state)
    {
        case PORT_STATE_IDLE: // Idle, Wait for something to show up
            if (pin != 0)     // Something has triggered
            {
                isr_timer = MAX_WAIT_TIME;   // Start the wait timer
                isr_state = PORT_STATE_WAIT; // Got something wait for all of the sensors to trigger
            }
            break;

        case PORT_STATE_WAIT:        // Something is present, wait for all of the inputs
            if ((pin == RUN_MASK)    // We have all of the inputs
                || (isr_timer == 0)) // or ran out of time.  Read the timers and restart
            {
                aquire();                       // Read the counters
                isr_timer = json_min_ring_time; // Reset the timer
                isr_state = PORT_STATE_DONE;    // and wait for the all clear
            }
            break;

        case PORT_STATE_DONE: // Waiting for the ringing to stop
            if (pin != 0)     // Something got latched
            {
                isr_timer = json_min_ring_time;
                stop_timers(); // Reset and try later
            }
            else
            {
                if (isr_timer == 0) // Make sure there is no ringing
                {
                    arm_timers();                // and arm for the next time
                    isr_state = PORT_STATE_IDLE; // and go back to idle
                }
            }
            break;
    }

    // Return from interrupt
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/*-----------------------------------------------------
 *
 * @function: freeETarget_timers
 *
 * @brief:    Update the free running timers
 *
 * @return:   Never
 *
 *-----------------------------------------------------
 *
 * This task runs every 10ms.
 *
 * The free running timers are decrimented and when they
 * hit zero, the individual timer is deleted
 *
 *-----------------------------------------------------*/
void freeETarget_timers(void *pvParameters)
{
    unsigned int i;

    DLT(DLT_CRITICAL, printf("freeETarget_timers()");)
    /*
     *  Decrement the timers on a 10ms (100Hz) interval
     */
    while (1)
    {
        for (i = 0; i != N_TIMERS; i++) // Refresh the timers.  Decriment in 10ms increments
        {
            if ((timers[i] != 0) && (*timers[i] != 0))
            {
                (*timers[i])--; // Decriment the timer
            }
        }
        vTaskDelay(TICK_10ms);
    }
    /*
     * Never get here
     */
    return;
}

/*-----------------------------------------------------
 *
 * @function: freeETarget_synchronous
 *
 * @brief:    Synchronous task scheduler
 *
 * @return:   None
 *
 *-----------------------------------------------------
 *
 * This task runs every 10ms.
 *
 * Synchronous tasks are called on a 10ms time band
 *
 * Unless you don't care about the time, always use
 * timer_new(&timer, ONE_SECOND * duration).
 *
 * IMPORTANT
 *
 * timers are NOT deleted when they expire.
 *
 *-----------------------------------------------------*/
void freeETarget_synchronous(void *pvParameters)
{
    unsigned int cycle_count = 0;
    unsigned int toggle      = 0;

    DLT(DLT_CRITICAL, printf("freeETarget_synchronous()");)

    while (1)
    {

        /*
         *  10 ms band
         */
        token_cycle();
        multifunction_switch_tick();
        multifunction_switch();
        paper_drive_tick();

        /*
         *  500 ms band
         */
        if ((cycle_count % BAND_500ms) == 0)
        {
            toggle ^= 1;
            commit_status_LEDs(toggle);
            tabata_task();
            rapid_fire_task();
        }

        /*
         * 1000 ms band
         */
        if ((cycle_count % BAND_1000ms) == 0)
        {
            bye(); // Dim the lights
            check_12V();
            send_keep_alive();
        }

        /*
         * All done, prepare for the next cycle
         */
        cycle_count++;
        vTaskDelay(TICK_10ms); // Delay 10ms
    }
}

/*-----------------------------------------------------
 *
 * @function: timer_new()
 *            timer_delete()
 *
 * @brief:    Add or remove timers
 *
 * @return:   TRUE if the operation was a success
 *
 *-----------------------------------------------------
 *
 *
 * These functions add or remove a timer from the active
 * timer list
 *
 * IMPORTANT
 *
 * The timers must be static variables, otherwise they
 * will overflow the available space every time they are
 * instantiated.
 *
 * Calling timer_new with the same timer address will
 * overwrite the previous timer value with the new one.
 * timer_new can be called any number of times with the
 * same timer addess without creating a problem
 *
 *-----------------------------------------------------*/
int timer_new(volatile unsigned long *new_timer, // Pointer to new down counter
              unsigned long           duration   // Duration of the timer
)
{
    unsigned int i;

    if (new_timer == NULL)
    {
        return 0;
    }

    for (i = 0; i != N_TIMERS; i++) // Look through the space
    {
        if ((timers[i] == 0)             // Got an empty timer slot
            || (timers[i] == new_timer)) // or it already exists
        {
            timers[i]  = new_timer; // Add it in
            *new_timer = duration;
            return 1;
        }
    }
    DLT(DLT_CRITICAL, printf("No space for new timer");)

    return 0;
}

int timer_delete(volatile unsigned long *old_timer // Pointer to new down counter
)
{
    unsigned int i;

    if (old_timer == 0)
    {
        return 0;
    }

    *old_timer = 0; // Set the timer to zero

    for (i = 0; i != N_TIMERS; i++) // Look through the space
    {
        if (timers[i] == old_timer) // Found the existing timer
        {
            timers[i] = 0; // Remove the pointer
            return 1;
        }
    }

    /*
     *  The timer doesn't exist, return an error
     */
    return 0;
}
