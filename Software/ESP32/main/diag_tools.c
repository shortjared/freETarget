
/*******************************************************************************
 *
 * diag_tools.c
 *
 * Debug and test tools 
 * 
 * See
 * https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
 *
 ******************************************************************************/

#include "stdbool.h"
#include "stdio.h"
#include "serial_io.h"
#include "gpio_types.h"
#include "driver\gpio.h"
#include "ctype.h"

#include "freETarget.h"
#include "gpio.h"
#include "diag_tools.h"
#include "analog_io.h"
#include "json.h"
#include "timer.h"
#include "esp_timer.h"
#include "dac.h"
#include "pwm.h"
#include "pcnt.h"
#include "analog_io.h"
#include "gpio_define.h"
#include "WiFi.h"
#include "compute_hit.h"

#define TICK(x) (((x) / 0.33) * OSCILLATOR_MHZ)   // Distance in clock ticks
#define RX(Z,X,Y) (16000 - (sqrt(sq(TICK(x)-s[(Z)].x) + sq(TICK(y)-s[(Z)].y))))
#define GRID_SIDE 25                              // Should be an odd number
#define TEST_SAMPLES ((GRID_SIDE)*(GRID_SIDE))

extern volatile unsigned long paper_time;

/*******************************************************************************
 *
 * @function: void self_test
 *
 * @brief: Execute self tests based on the jumper settings
 * 
 * @return: None
 *
 *******************************************************************************
 *   
 *   This function is a large case statement with each element
 *   of the case statement 
 * 
 *   Supports a test mode (TEST:99) that emulates an old style Zapple monitor
 *   
 ******************************************************************************/
unsigned int tick;
void zapple(unsigned int test)
{ 
  int run_test;                 // Running Semephore
  char ch;                      // Input character

  run_test = 1;

  while (run_test != 0)
  {
    printf("\r\nTest >");
    test = 0;

    while (1)
    {
      if ( serial_available(CONSOLE) != 0 )
      {
        ch = serial_getch(CONSOLE);
        ch = toupper(ch);
        printf("%c", ch);
        if ( ch != '\r' )
        {
            test = ((test * 10) + (ch -'0')) % 100;
          }
          if ( (ch == '\r') || (ch == '\n') )
          {
            self_test(test);
            break;
          }
          if ( ch == 'X' )
          {
            run_test = 0;
            break;
          }
        }
        vTaskDelay(1);
      }
    }

/*
 *  GO back to normal operation
 */
  return;
}


void self_test
(
  unsigned int test                 // What test to execute
)
{
  int   i;

/*
 *  Switch over to test mode 
 */
  run_state |= IN_TEST;             // Show the test is running 
  
  while ( run_state & IN_OPERATION )
  {
    vTaskDelay(10);                 // Wait for everyone else to turn off
  }
  freeETarget_timer_pause();        // Stop interrupts

/*
 * Figure out what test to run
 */
  switch (test)
  {
/*
 * Test 0, Display the help
 */
    default:                // Undefined, show the tests
    case T_HELP:  
      printf("\r\n 1 - Factory test");
      printf("\r\n");              
      printf("\r\n10 - Digital inputs");
      printf("\r\n11 - Advance paper backer");
      printf("\r\n12 - LED brightness test");
      printf("\r\n13 - Status LED driver");
      printf("\r\n14 - Temperature and sendor test");
      printf("\r\n15 - DAC test");
      printf("\r\n16 - Rapid fire LED test");
      printf("\r\n"); 
      printf("\r\n20 - PCNT Test");
      printf("\r\n21 - pcnt(1) - Timers not running"); 
      printf("\r\n22 - pcnt(2) - Timers start - stop together"); 
      printf("\r\n23 - pcnt(3) - Timers free running"); 
      printf("\r\n24 - pcnt(4) - Timers cleared"); 
      printf("\r\n25 - Turn the oscillator on and off");
      printf("\r\n26 - Turn the RUN lines on and off");
      printf("\r\n27 - Trigger NORTH from a function generator");
      printf("\r\n"); 
      printf("\r\n30 - AUX Port loopback");
      printf("\r\n31 - Test WiFi as an Access Point");
      printf("\r\n32 - Test WiFI as a station"); 
      printf("\r\n33 - Enable the WiFi Server");
      printf("\r\n34 - Loopback the TCPIP data");
      printf("\r\n35 - Loopback WiFi");
      printf("\r\n");
      printf("\r\n40 - Sensor POST test");
      printf("\r\n41 - Polled Target Test");
      printf("\r\n42 - Interrupt Target Test");
      printf("\r\n");
      printf("\r\n99 - Zapple Debug Monitor");
      printf("\r\n");
    break;

/*
 * Test all of the hardware
 */
    case T_FACTORY: 
      factory_test();
      break;
/*
 * Display GPIO inputs
 */
    case T_DIGITAL: 
      digital_test();
      break;

/*
 * Advance the paper
 */
    case T_PAPER:
      paper_test();
      break;

/*
 * Set the LED bightness
 */
    case T_LED:
      printf("\r\nCycling the LED");
      for (i=0; i <= 100; i+= 5)
      {
        printf("%d   ", i);
        pwm_set(LED_PWM, i);       
        vTaskDelay(ONE_SECOND/10);
      }
      for (i=100; i >= 0; i-=5)
      {
        printf("%d   ", i);
        pwm_set(LED_PWM,i);       
        vTaskDelay(ONE_SECOND/10);
      }
      printf("\r\nDone\r\n");
      break;

/*
 * Set status LEDs
 */
    case T_STATUS:
      status_LED_test();
      break;

/*
 * Analog In
 */
    case T_TEMPERATURE:
      analog_input_test();
      break;

/*
 * DAC
 */
    case T_DAC:
      DAC_test();
      break;

/*
 * Set Rapid Fire LEDs
 */
    case T_RAPID_LEDS:
      rapid_LED_test();
      break;

/*
 * PCNT test
 */
    case T_PCNT:
      pcnt_test(1);
      pcnt_test(2);
      pcnt_test(3);
      pcnt_test(4);
      break;

    case T_PCNT_CAL:
      pcnt_cal();
      break;

/*
 *  Sensor Trigger
 */
    case T_SENSOR:
      POST_counters();
      break;

/*
 *  AUX Serial Port
 */
    case T_AUX_SERIAL:
      serial_port_test();
      break;

/*
 *  Polled Target Test
 */
    case T_TARGET:
      polled_target_test();
      break;      
/*
 *  Interrupt Target Test
 */
    case T_TARGET_2:
      interrupt_target_test();
      break; 

/*
 *  Start WiFi AP
 */
    case T_WIFI_AP:
      WiFi_AP_init();
      break; 

/*
 *  Start WiFi station
 */
    case T_WIFI_STATION:
      WiFi_station_init();
      break; 

/*
 *  Enable the WiFi Server
 */
    case T_WIFI_SERVER:
      xTaskCreate(WiFi_tcp_server_task,    "WiFi_tcp_server",      4096, NULL, 5, NULL);
      break; 

/*
 *  Send and receive something
 */
    case T_WIFI_STATION_LOOPBACK:
      WiFi_station_init();
      xTaskCreate(WiFi_tcp_server_task,    "WiFi_tcp_server",      4096, NULL, 5, NULL);
      xTaskCreate(tcpip_accept_poll,       "tcpip_accept_poll",    4096, NULL, 4, NULL);
      WiFi_loopback_test();
      break; 

/*
 *  Send and receive something
 */
    case T_WIFI_AP_LOOPBACK:
      WiFi_AP_init();
      xTaskCreate(WiFi_tcp_server_task,    "WiFi_tcp_server",      4096, NULL, 5, NULL);
      xTaskCreate(tcpip_accept_poll,       "tcpip_accept_poll",    4096, NULL, 4, NULL);
      WiFi_loopback_test();
      break; 

/*
 *  Cycle the 10MHz clock input
 */
    case T_CYCLE_CLOCK:
      printf("\r\nCycle 10MHz Osc 2:1 duty cycle\r\n");
      while (serial_available(CONSOLE) == 0)
      {
        gpio_set_level(OSC_CONTROL, OSC_ON);       // Turn off the oscillator
        vTaskDelay(ONE_SECOND/2);                  // The oscillator should be on for 1/2 second
        gpio_set_level(OSC_CONTROL, OSC_OFF);      // Turn off the oscillator
        vTaskDelay(ONE_SECOND/4);                  // The oscillator shold be off for 1/4 seocnd
      }
      break; 
/*
 *  Turn the RUN lines on and off
 */
    case T_RUN_ALL:
      printf("\r\nCycle RUN lines at 2:1 duty cycle\r\n");
      while (serial_available(CONSOLE) == 0)
      {
        gpio_set_level(STOP_N, 1);                  // Let the clock go
        gpio_set_level(CLOCK_START, 0);   
        gpio_set_level(CLOCK_START, 1);   
        gpio_set_level(CLOCK_START, 0);             // Strobe the RUN linwes
        vTaskDelay(ONE_SECOND/2);                   // The RUN lines should be on for 1/2 second
        gpio_set_level(STOP_N, 0);                  // Stop the clock
        vTaskDelay(ONE_SECOND/4);                   // THe RUN lines shold be off for 1/4 second
      }
      break; 

/*
 *  Single PCNT test
 */
    case T_PCNT_STOP:
    case T_PCNT_SHORT:
    case T_PCNT_FREE:
    case T_PCNT_CLEAR:
      pcnt_test(test - (T_PCNT_STOP - 1));
      break;
  }

 /* 
  *  All done, return;
  */
    run_state &= ~IN_TEST;              // Exit the test 
    freeETarget_timer_start();          // Start interrupts
    return;
}
/*-----------------------------------------------------
 * 
 * @function: factory_test()
 * 
 * @brief:    Test all the things we can test
 * 
 * @return:   None
 * 
 *-----------------------------------------------------
 *
 * This is the factory test to test all of the circuit 
 * elements.
 * 
 *-----------------------------------------------------*/

#define PASS_RUNNING 0x00FF
#define PASS_A       0x0100
#define PASS_B       0x0200
#define PASS_C       0X4000
#define PASS_D       0x8000
#define PASS_MASK    (PASS_RUNNING | PASS_A | PASS_B)
#define PASS_TEST    (PASS_RUNNING | PASS_C)

bool factory_test(void)
{
  int i, percent;
  int running;            // Bit mask from run flip flops
  int dip;                // Input from DIP input
  char ch;
  char ABCD[] = "DCBA";   // DIP switch order
  int  pass;              // Pass YES/NO
  bool passed_once;       // Passed all of the tests at least once
  float volts[4];
  int  motor_toggle;      // Toggle motor on an off

/*
 *  Force the refernce voltages - Incase the board has been uninitialized
 */
  if ( (json_vref_lo == 0) || (json_vref_hi == 0))
  {
    volts[VREF_LO] = 1.25;
    volts[VREF_HI] = 2.00;
    volts[VREF_2]  = 0.00;
    volts[VREF_3]  = 0.00;
    DAC_write(volts);
  }
/*
 * Ready to start the test
 */
  printf("\r\nFactory Test");
  printf("\r\nHas the tape seal been removed from the temperature sensor?");
  printf("\r\nPress 1 & 2 or ! to continue\r\n");
  while ( (DIP_SW_A == 0) || !(DIP_SW_B == 0) )
  {
    if ( serial_available(ALL) )
    {
      if ( serial_getch(ALL) == '!' )
      {
        break;
      }
    }
    continue;
  }

/*
 *  Begin test
 */
  arm_timers();
  pass = 0;
  passed_once = false;
  percent = 0;
  motor_toggle = 0;
/*
 * Loop and poll the various inputs and output
 */
  while (1)
  {
    running = is_running();
    if ( running == 0x00FF )
    {
      pass |= PASS_RUNNING;
    }
    printf("\r\nSens: ");
    for (i=0; i != 8; i++)
    {
      if ( i == 4 )
      {
        printf(" ");
      }
      if (running & (1<<i))
      {
        printf("%c", find_sensor(1<<i)->short_name );
      }
      else
      {
        printf("-");
      }
    }

    dip = read_DIP();
    printf("  DIP: "); 
    if ( DIP_SW_A )
    {
      set_status_LED("-W-");
      pass |= PASS_A;
    }
    else
    {
      set_status_LED("- -");
    }

    if ( DIP_SW_B )
    {
      set_status_LED("--W");
      pass |= PASS_B;
    }
    else
    {
      set_status_LED("-- ");
    }

    if ( DIP_SW_C )
    {
      pass |= PASS_C;
    }

    if ( DIP_SW_D )
    {
      pass |= PASS_D;
    }

    for (i=3; i >= 0; i--)
    {
      if ((dip & (1<<i)) == 0)
      {
        printf("%c", ABCD[i] );
      }
      else
      {
        printf("-");
      }
    }

    printf("  12V: %4.2fV", v12_supply());
    printf("  Rev: %d", revision());
    printf("  Temp: %4.2fC", temperature_C());
    printf("  Humd: %4.2f%%", humidity_RH());

    printf("  M");
    if ( motor_toggle )
    {
      printf("+");
      DCmotor_on_off(true, ONE_SECOND);
    }
    else
    {
      printf("-");
      DCmotor_on_off(false, 0);
    }
    motor_toggle ^= 1;
    
    set_LED_PWM_now(percent);
    printf("  LED: %3d%% ", percent);
    percent = percent + 25;
    if ( percent > 100 )
    {
      percent = 0;
    }

    printf("V:%s:",SOFTWARE_VERSION);

    if ( (pass == PASS_MASK) || (pass == PASS_TEST) ) 
    {
      set_status_LED(LED_GOOD);
      printf("  PASS");
      vTaskDelay(ONE_SECOND);
      arm_timers();
      pass = 0;
      passed_once = true;
    }

/*
 *  See if there is any user controls 
 */
    if ( serial_available(CONSOLE) )
    {
      ch = serial_getch(CONSOLE);
      switch (ch)
      {
        default:
        case 'R':               // Reset the test
        case 'r':
          pass = 0;             // Reset the pass/fail
          arm_timers();
          break;

        case 'X':               // Exit
        case 'x':
        case '!':
          DCmotor_on_off(false, 0);
          if ( passed_once == true )
          {
            printf("\r\nTest completed successfully\r\n");
          }
          else
          {
            printf("\r\nTest finished without PASS\r\n");
          }
          return passed_once;
      }
    }

    vTaskDelay(ONE_SECOND / 2);

  }

/*
 *  The test has been terminated
 */
  return passed_once;
}

/*******************************************************************************
 * 
 * @function: POST_version()
 * 
 * @brief: Show the Version String
 * 
 * @return: None
 * 
 *******************************************************************************
 *
 *  Common function to show the version. Routed to the selected
 *  port(s)
 *  
 *******************************************************************************/
 void POST_version(void)
 {
  SEND(sprintf(_xs, "\r\n{\"VERSION\": %s}\r\n", SOFTWARE_VERSION);)

/*
 * All done, return
 */
  return;
}
 
/*----------------------------------------------------------------
 * 
 * @function: void POST_counters()
 * 
 * @brief: Verify the counter circuit operation
 * 
 * @return: TRUE if the tests pass
 *          Never if the tests fail
 * 
 *----------------------------------------------------------------
 *
 *  Trigger the counters from inside the circuit board and 
 *  read back the results and look for an expected value.
 *  
 *  Return only if all of the tests pass
 *  
 *  Test 1, Make sure the 10MHz clock is running
 *  Test 2, Clear the flip flops and make sure the run latches are clear
 *  Test 3, Trigger the flip flops and make sure that no run latche are set
 *  Test 4, Trigger a run and verify that the counters change
 * 
 *--------------------------------------------------------------*/
bool POST_counters(void)
{
  unsigned int i;                          // Iteration counter
  unsigned int count, toggle, running;     // Cycle counter
  DLT(DLT_CRITICAL, printf("POST_counters()");)
  
/*
 *  Test 1, Make sure we can turn off the reference clock
 */
  count = 0;
  gpio_set_level(OSC_CONTROL, OSC_OFF);   // Turn off the oscillator
  toggle = gpio_get_level(REF_CLK);
  for  (i=0; i != 1000; i++)               // Try 1000 times
  {
    if ( (gpio_get_level(REF_CLK) ^ toggle) != 0 )  // Look for a change
    {
      count++;
      toggle = gpio_get_level(REF_CLK);
    }
  }
  
  if ( count != 0 )
  {
    DLT(DLT_CRITICAL, printf("Reference clock cannot be stopped");)
    set_diag_LED(LED_FAIL_CLOCK_STOP, 0);
  }

/*
 *  Test 2, Make sure we can turn the reference clock on
 */
  count = 0;
  gpio_set_level(OSC_CONTROL, OSC_ON);
  toggle = gpio_get_level(REF_CLK);
  for  (i=0; i != 1000; i++)               // Try 1000 times
  {
    if ( (gpio_get_level(REF_CLK) ^ toggle) != 0 )  // Look for a change
    {
      count++;
      toggle = gpio_get_level(REF_CLK);
    }
  }

  if ( count == 0  )
  {
    DLT(DLT_CRITICAL, printf("Reference clock cannot be started");)
    set_diag_LED(LED_FAIL_CLOCK_START, 0);
  }

/*
 *  Test 3, Make sure we can turn the triggers off
 */
  gpio_set_level(STOP_N, 0);        // Clear the latch
  gpio_set_level(STOP_N, 1);        // and reenable it
  running = is_running();
  if ( running != 0  )
  {
    DLT(DLT_CRITICAL, printf("Stuck bit in run latch: ");)
    for (i=N; i <= W; i++)
    {
      if ( running & s[i].low_sense.run_mask ) 
      {
        set_diag_LED(s[i].low_sense.diag_LED,0);
      }
      if ( running & s[i].high_sense.run_mask ) 
      {
        set_diag_LED(s[i].high_sense.diag_LED,0);
      }

    }
  }      
  vTaskDelay(ONE_SECOND);

/*
 * Test 4, Trigger the timers
 */
  gpio_set_level(STOP_N, 0);          // Clear the latch
  gpio_set_level(STOP_N, 1);
  gpio_set_level(CLOCK_START, 1);     // Triger the run latch
  gpio_set_level(CLOCK_START, 0);
  gpio_set_level(CLOCK_START, 1);
  if ( is_running() != 0xFF  )
  {
    DLT(DLT_CRITICAL, printf("Failed to start clock in run latch: %02X", is_running());)
    set_diag_LED(LED_FAIL_RUN_STUCK, 0);
  }

/*
 * We get here only if all of the tests pass
 */
  return 1;
}

/*----------------------------------------------------------------
 *
 * @function: show_sensor_status()
 *
 * @brief:    Show which sensor flip flops were latched
 *
 * @return:   Nothing
 * 
 *----------------------------------------------------------------
 * 
 * The sensor state NESW or .... is shown for each latch
 * The clock values are also printed
 *   
 *--------------------------------------------------------------*/
void show_sensor_status
  (
  unsigned int   sensor_status
  )
{
  unsigned int i;
  
  printf(" Latch:");

  for (i=N; i<=W; i++)
  {
    if ( sensor_status & (1<<i) )   printf("%c", find_sensor(i<<i)->short_name);
    else                            printf(".");
  }

  printf("  Face Strike: %d", face_strike);
  
  printf("  Temperature: %4.2f", temperature_C());
  
  printf("  Switch:");
  
  if ( DIP_SW_A == 0 )
  {
    printf("--");
  }
  else
  {
    printf("A1");
  }
  printf(" ");
  if ( DIP_SW_B == 0 )
  {
    printf("--");
  }
  else
  {
    printf("B2");
  }

  if (( sensor_status & 0x0f) == 0x0f)
  {
    printf(" PASS");
    vTaskDelay(ONE_SECOND);                // Wait for click to go away
  }    

/*
 * All done, return
 */
  return;
}

/*----------------------------------------------------------------
 *
 * @function: show_sensor_fault()
 * 
 * @brief:    Use the LEDs to show if a sensor failed
 *
 * @return:   Nothing
 * 
 *----------------------------------------------------------------
 * 
 * This function is intended as a diagnostic to show if a sensor
 * failed to detect a show
 *   
 *--------------------------------------------------------------*/
void show_sensor_fault
(
  unsigned int   sensor_status
)
{
  unsigned int i;
  
  for (i=N; i<=W_HI; i++)
  {
    if ( (sensor_status & (1<<i)) == 0)
    {
      set_diag_LED(find_sensor(1<<i)->diag_LED, 2);
      return;
    }
  }

/*
 * All done, return
 */
  return;
}

/*----------------------------------------------------------------
 *
 * @function: do_dlt
 *
 * @brief:    Check for a DLT log and print the time
 *
 * @return:   TRUE if the DLT should be printed
 * 
 *----------------------------------------------------------------
 * 
 * is_trace is compared to the log level and if valid the
 * current time stamp is printed
 * 
 * DLT_CRItiCAL levels are always printed
 *   
 *--------------------------------------------------------------*/
bool do_dlt
  (
  unsigned int level
  )
{ 
  if ((level & (is_trace | DLT_CRITICAL)) == 0 )
  {
    return false;      // Send out if the trace is higher than the level 
  }

  printf("\r\nI (%d) ", (int)(esp_timer_get_time()/1000) );

  return true;
}

/*----------------------------------------------------------------
 *
 * @function: set_diag_LED
 *
 * @brief:    Set the state of the diagnostics LED
 *
 * @return:   Nothing
 * 
 *----------------------------------------------------------------
 * 
 * This function is similar to set_status_LED() except that the
 * duration of the LED is configurable
 *   
 *--------------------------------------------------------------*/
void set_diag_LED
(
  char* new_LEDs,           // NEW LED display
  unsigned int duration     // How long the display is present for in seconds
)
{
  set_status_LED(new_LEDs);

/*
 *  Test for infinit wait
 */
  if ( duration == 0 )      // Wait here forever 
  {
    while (1)
    {
      vTaskDelay(ONE_SECOND);
    }
  }

/*
 *  The fault is displayed for a short time
 */
  vTaskDelay(ONE_SECOND * duration);

/*
 *  All done, return
 */
  return;

}

/*----------------------------------------------------------------
 *
 * @function: check_12V
 *
 * @brief:    Make sure the 12 Volt supply is within limits
 *
 * @return:   TRUE if the 12V is within spec
 * 
 *----------------------------------------------------------------
 * 
 * The 12V supply is read and compared against limits.
 * 
 * V >= 10 V  Green LED          Working
 * 5 < V <=   10 Yellow LED      Caution
 * V <= 5     Red LED            Disabld
 * 
 * Return TRUE if the motor can be driven
 *   
 * The LEDs are only set on a change from (say) working to caution
 * this is done to prevent the LEDs from flickering
 * 
 *--------------------------------------------------------------*/
#define NONE  0
#define SOME  1
#define V12OK 2
#define UNKNOWN 99

#define WORKING  10.0
#define CAUTION  5.0

bool check_12V(void)
{
  static unsigned int fault_V12 = UNKNOWN;
  float  v12;

  v12 = v12_supply();

  if ( v12 <= CAUTION )
  {
    if ( fault_V12 != NONE)
    {
      set_status_LED(LED_NO_12V);
      fault_V12 = NONE;
    }
    return false;
  }

  if ( v12 <= WORKING )
  {
    if ( fault_V12 != SOME )
    {
      set_status_LED(LED_LOW_12V);
      fault_V12 = SOME;
    }
    return false;
  }

  if ( fault_V12 != V12OK )              // Did we have an error last time?
  {
    set_status_LED(LED_OK_12V);         // Gone, clear the error
    fault_V12 = V12OK;
  }

  return true;
}