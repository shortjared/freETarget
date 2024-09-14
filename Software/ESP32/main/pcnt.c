/*************************************************************************
 *
 * file: pcnt.c
 *
 * description:  Pulse counter control
 *
 **************************************************************************
 *
 * This file sets up the timers and routing for the PCNT control
 *
 * See:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
 * https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#pcnt
 * https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#pcnt
 *
 ***************************************************************************/
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/timer.h"
#include "nonvol.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "stdbool.h"

#include "dac.h"
#include "diag_tools.h"
#include "freETarget.h"
#include "gpio_define.h"
#include "json.h"
#include "pcnt.h"
#include "serial_io.h"
#include "timer.h"

/*
 *  Working variables
 */
static pcnt_unit_config_t          unit_config[SOC_PCNT_UNITS_PER_GROUP];   // Single unit configuration
static pcnt_unit_handle_t          pcnt_unit[SOC_PCNT_UNITS_PER_GROUP];
static pcnt_glitch_filter_config_t filter_config[SOC_PCNT_UNITS_PER_GROUP]; // Glitch Filter

static pcnt_chan_config_t    chan_a_config[SOC_PCNT_UNITS_PER_GROUP];       // Counter Configuration A
static pcnt_channel_handle_t pcnt_chan_a[SOC_PCNT_UNITS_PER_GROUP];

static pcnt_chan_config_t    chan_b_config[SOC_PCNT_UNITS_PER_GROUP];       // Counter Configuration B
static pcnt_channel_handle_t pcnt_chan_b[SOC_PCNT_UNITS_PER_GROUP];

static int north_pcnt_hi, east_pcnt_hi, south_pcnt_hi, west_pcnt_hi;

/*
 *  Function prototypes
 */
static bool north_hi_pcnt_isr_callback(void *args);
static bool east_hi_pcnt_isr_callback(void *args);
static bool south_hi_pcnt_isr_callback(void *args);
static bool west_hi_pcnt_isr_callback(void *args);

/*************************************************************************
 *
 * @function: pcnt_init()
 *
 * description:  Set the pulse counter control
 *
 * @return:  Nope
 *
 **************************************************************************
 *
 * The PCNT control for FreeETarget is set up as
 *
 * Channel A Count connected to 10MHz Clock.  Count on rising edge
 * Channel A Control connected to FlipFlop.   Count on high level
 *
 * Channel B Count disabled
 * Channel B Control disabled
 *
 **************************************************************************/
void pcnt_init(int unit,    // What unit to use
               int run,     // GPIO associated with PCNT control
               int clock    // GPIO associated with PCNT signal
) {
  static bool is_first = 1; // Set to 0 on subsequent passes

  /*
   * Make sure everything is turned off
   */
  gpio_set_level(STOP_N, 0);
  gpio_set_level(STOP_N, 1);
  gpio_set_level(CLOCK_START, 0);

  /*
   * Setup the unit
   */
  unit_config[unit].low_limit  = -0x7fff;
  unit_config[unit].high_limit = 0x7fff;
  pcnt_unit[unit]              = NULL;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config[unit], &pcnt_unit[unit]));

  /*
   *  Setup the glitch filter
   */
  filter_config[unit].max_glitch_ns = 10;
  pcnt_unit_set_glitch_filter(pcnt_unit[unit], &filter_config[unit]);

  /*
   *  Setup the channel.  Only Channel A is used.  B is left idle
   */
  pcnt_chan_a[unit]                  = NULL;
  chan_a_config[unit].edge_gpio_num  = clock; // Counter
  chan_a_config[unit].level_gpio_num = run;   // Enable
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit[unit], &chan_a_config[unit], &pcnt_chan_a[unit]));

  pcnt_chan_b[unit]                  = NULL;
  chan_b_config[unit].edge_gpio_num  = -1;
  chan_b_config[unit].level_gpio_num = -1;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit[unit], &chan_b_config[unit], &pcnt_chan_b[unit]));

  /*
   *  Setup the control.  Count only when the control is HIGH.
   */
  //                                Channel                       Rising Edge                        Falling Edge
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(pcnt_chan_a[unit], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD)); // Counter
  //                                Channel                        When High                          When Low
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(pcnt_chan_a[unit], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD)); // Control

  /*
   *  Setup the GPIO interrupts for the PCNT hi counts
   */
  if (is_first) {
    gpio_install_isr_service(0);                                          // Per GPIO interrupt handler
    gpio_set_intr_type(RUN_NORTH_HI, GPIO_INTR_POSEDGE);                  // RUN_XXX_HI interrupt on
    gpio_set_intr_type(RUN_EAST_HI, GPIO_INTR_POSEDGE);                   // rising edge
    gpio_set_intr_type(RUN_SOUTH_HI, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(RUN_WEST_HI, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(RUN_NORTH_HI, north_hi_pcnt_isr_callback, NULL); // Collect PCNT for North trigger
    gpio_isr_handler_add(RUN_EAST_HI, east_hi_pcnt_isr_callback, NULL);
    gpio_isr_handler_add(RUN_SOUTH_HI, south_hi_pcnt_isr_callback, NULL);
    gpio_isr_handler_add(RUN_WEST_HI, west_hi_pcnt_isr_callback, NULL);
    is_first = false;
  }

  /*
   *  All done, Clear the counter and return
   */
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit[unit]));
  pcnt_unit_clear_count(pcnt_unit[unit]);
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit[unit]));
  return;
}

/*************************************************************************
 *
 * @function: pcnt_read()
 *
 * description:  Read the pcnt register
 *
 * @return:   32 bit timer count
 *
 **************************************************************************
 *
 * The PCNT registers are 16 bits long with an overflow counter
 *
 **************************************************************************/
#define PCNT_BASE 0x60017000 + 0x00030 // Pointer to start of PCNT registers

int pcnt_read(unsigned int unit        // What timer to read
) {
  int value;

  unit &= 7;

  switch (unit) {
  case 0:
  case 1:
  case 2:
  case 3:
    pcnt_unit_get_count(pcnt_unit[unit], &value);
    break;
  case 4:
    value = north_pcnt_hi;
    break;
  case 5:
    value = east_pcnt_hi;
    break;
  case 6:
    value = south_pcnt_hi;
    break;
  case 7:
    value = west_pcnt_hi;
    break;
  }

  return value;
}

/*************************************************************************
 *
 * @function: pcnt_clear()
 *
 * description:  Clear the pcnt registers
 * `
 * @return:   Nothing
 *
 **************************************************************************
 *
 * Clear all of the registers at once
 *
 **************************************************************************/
void pcnt_clear(void) {
  int i;

  for (i = 0; i != SOC_PCNT_UNITS_PER_GROUP; i++) {
    pcnt_unit_clear_count(pcnt_unit[i]);
  }

  north_pcnt_hi = 0;
  east_pcnt_hi  = 0;
  south_pcnt_hi = 0;
  west_pcnt_hi  = 0;

  return;
}

/*************************************************************************
 *
 * @function:     pcnt_test()
 *
 * @description:  Put the PCNT into a knonw state and observe the results
 * `
 * @return:       Nothing
 *
 **************************************************************************
 *
 * This drives the PCNT registers in the expected modes
 *
 * 1 - Verify that the counters can be cleared
 * 2 - Verify that the counters can be started and stopped together
 * 3 - Turn on the counters and verify that they increment in the right direction
 * 4 - Stop the timers before going into servcie
 *
 **************************************************************************/

void pcnt_test(int which_test) {
  int          array[10][8]; // Test result storage
  unsigned int i, j;

  switch (which_test) {
    /*
     * Test 1, verify that the counters can be cleared
     */
  case 1:
    printf("\r\nPCNT-1  Counters cleared and not running.  Low counters should all be 0. High Counters 11, 22, 33, 44");
    arm_timers();                   // Arm the timers
    gpio_set_level(CLOCK_START, 0); // Do not trigger the clock
    printf("\r\nis_running: %02X", is_running());
    for (i = 0; i != 10; i++) {
      for (j = 0; j != 8; j++) {
        array[i][j] = pcnt_read(j);
      }
    }
    for (i = 0; i != 10; i++) {
      printf("\r\n");
      for (j = 0; j != 8; j++) {
        if (j == 4) {
          printf("    ");
        }
        printf("%s: %d   ", find_sensor(1 << i)->long_name, array[i][j]);
      }
    }
    printf("\r\nis_running: %02X  ", is_running());
    break;

    /*
     *  Test 2 - Verify that the counters can be started and stopped together
     */
  case 2:
    printf("\r\n\r\nPCNT-2  Start/stop counters together. Should all be the same");
    arm_timers();
    trigger_timers();
    vTaskDelay(1);
    printf("\r\nis_running(): %02X", is_running());
    stop_timers();
    for (i = 0; i != 10; i++) {
      for (j = 0; j != 8; j++) {
        array[i][j] = pcnt_read(j);
      }
    }
    for (i = 0; i != 10; i++) {
      printf("\r\n");
      for (j = 0; j != 8; j++) {
        if (j == 4) {
          printf("      ");
        }
        printf("%s: %d   ", find_sensor(1 << i)->long_name, array[i][j]);
      }
    }
    printf("\r\nis_running(): %02X\r\n", is_running());
    break;
    /*
     * Test 3 - Turn on the counters and verify that they increment in the right direction
     */
  case 3:
    printf("\r\n\r\nPCNT-3  Start counters and do not stop. Should increase left-right, top-bottom");
    arm_timers();
    trigger_timers();
    printf("\r\nis_running(): %02X  ", is_running());
    for (i = 0; i != 10; i++) {
      for (j = 0; j != 8; j++) {
        array[i][j] = pcnt_read(j);
      }
    }
    for (i = 0; i != 10; i++) {
      printf("\r\n");
      for (j = 0; j != 8; j++) {
        if (j == 4) {
          printf("    ");
        }
        printf("%s: %d  ", find_sensor(1 << i)->long_name, array[i][j]);
      }
    }
    printf("\r\nis_running(): %02X  ", is_running());
    break;

    /*
     *  Test 4 - Stop the timers before going into servcie
     */
  case 4:
    printf("\r\n\r\nPCNT-4  Turn off all timers");
    stop_timers();
    printf("\r\nis_running(): %02X  ", is_running());
    break;
  }
  /*
   * Test Over
   */
  printf("\r\ndone");
  return;
}

/*************************************************************************
 *
 * @function:     pcnt_cal()
 *
 * @description:  Trigger the North sensor from a function generator
 *                and observe pcnt
 * `
 * @return:       Nothing
 *
 **************************************************************************
 *
 * The function sets the high and low trip points to the same value.  When
 * a trip occurs, the high value is read a tiny bit after the start, with
 * the time delay corresponding to the interrupt latencey. Thus the latency
 * and the time correction can be read directly as the average NORTH_HI time.
 *
 * The NORTH sensor is driven from a triangle wave and the time delay in
 * the NORTH_HI counter is printed on the display.
 *
 **************************************************************************/
#define NORTH_HI_DIP 0x80

void pcnt_cal(void) {
  int   north_min, north_max;           // Running statistics
  int   north_hi, north_average;        // Read from counters
  int   count;                          // Number of samples in average
  char  ch;
  float value[] = {2.0, 2.0, 0.0, 0.0}; // Set both trip points to 2 volts

  north_min     = 10000;
  north_max     = 0;
  north_average = 0;
  count         = 0;

  DAC_write(value); // Set both trip points to the same value

  printf("\r\nPCNT-CAL  Display triggering jitter.");
  printf("\r\n! to exit, R to reset");

  /*
   *  Setup the hardware
   */
  gpio_set_level(CLOCK_START, 0); // Turn off the test start
  gpio_intr_enable(RUN_NORTH_HI); // Turn on the interrupts
  gpio_intr_enable(RUN_EAST_HI);
  gpio_intr_enable(RUN_SOUTH_HI);
  gpio_intr_enable(RUN_WEST_HI);
  gpio_set_level(OSC_CONTROL, OSC_ON); // Turn on the oscillator
  vTaskDelay(1);                       // Let the oscillator start up

  /*
   * Loop, arm the counters and see what comes back
   */
  while (1) {
    pcnt_clear();                                         // and clear the counters
                                                          /*
                                                           * Wait for a reading to be triggered
                                                           */
    while ((is_running() & NORTH_HI_DIP) != NORTH_HI_DIP) // Should trigger almost instantly
    {
      if (serial_available(CONSOLE) != 0)                 // See if there is any console iput
      {

        ch = serial_getch(CONSOLE);
        if (ch == '!') // Exit
        {
          printf("\r\nSave this correction (y/n)?");
          while (serial_available(CONSOLE) == 0) {
            continue;
          }
          ch = serial_getch(CONSOLE);
          if ((ch == 'Y') || (ch == 'y')) {
            json_pcnt_latency = north_average / count;
            nvs_set_i32(my_handle, NONVOL_PCNT_LATENCY, json_pcnt_latency);
            printf("\r\nSaved");
          }

          printf("\r\nDone\r\n");
          return;
        }
        north_min     = 10000; // Anything else start over
        north_max     = 0;
        north_average = 0;
        count         = 0;
      }
    }

    /*
     *  Read in the counters and find the min and max
     */
    north_hi = pcnt_read(NORTH_HI);
    north_average += north_hi;
    count++;
    if (north_hi < north_min) {
      north_min = north_hi;
    }

    if (north_hi > north_max) {
      north_max = north_hi;
    }

    /*
     *  Display the results and wait before turning on the timers again
     */
    printf("\r\nnorth_hi: %d  north_avg: %d  north_min: %d   north_max: %d   north_hi-avg: %d", north_hi, north_average / count, north_min,
           north_max, (north_hi - (north_average / count)));
    vTaskDelay(ONE_SECOND / 2);
  }

  /*
   * Test Over
   */
  printf("\r\nDone");
  return;
}

/*************************************************************************
 *
 * @function:     pcnt_high_isr()
 *
 * @description:  Fake high PCNT units
 * `
 * @return:       Nothing
 *
 **************************************************************************
 *
 * The board was originally made for eight PCNT units, but th S3 only has
 * four.  This function (ISR) fakes the 4 top counters by reading the PCNT
 * register on the rising edge of the RUN_XX_HI control.  THis gives the
 * time interval from when the signal crosses VREF_LO to VREF_HI and hence
 * to the start of the pulse.
 *
 * During development this function has been simplified and it now only
 * captures the timer values.  The disable interrupts has been removed
 * because the signals are latched and a second interrupt will not occur
 *
 **************************************************************************/
#define PCNT_NORTH_HI (int *)(0x60017000 + 0x0030) // PCNT unit 1 count
#define PCNT_EAST_HI  (int *)(0x60017000 + 0x0034) // PCNT unit 2 count
#define PCNT_SOUTH_HI (int *)(0x60017000 + 0x0038) // PCNT unit 3 count
#define PCNT_WEST_HI  (int *)(0x60017000 + 0x003C) // PCNT unit 4 count

static bool IRAM_ATTR north_hi_pcnt_isr_callback(void *args) {
  north_pcnt_hi = *PCNT_NORTH_HI;
  return pdFALSE;
}

static bool IRAM_ATTR east_hi_pcnt_isr_callback(void *args) {
  east_pcnt_hi = *PCNT_EAST_HI;
  return pdFALSE;
}

static bool IRAM_ATTR south_hi_pcnt_isr_callback(void *args) {
  south_pcnt_hi = *PCNT_SOUTH_HI;
  return pdFALSE;
}

static bool IRAM_ATTR west_hi_pcnt_isr_callback(void *args) {
  west_pcnt_hi = *PCNT_WEST_HI;
  return pdFALSE;
}