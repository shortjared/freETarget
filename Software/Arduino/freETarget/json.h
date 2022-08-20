/*----------------------------------------------------------------
 *
 * json.h
 *
 * Header file for JSON functions and called routines
 *
 *---------------------------------------------------------------*/
#ifndef _JSON_H_
#define _JSON_H_

typedef struct  {
  char*             token;    // JSON token string, ex "RADIUS": 
  int*              value;    // Where value is stored 
  double*         d_value;    // Where value is stored 
  byte            convert;    // Conversion type
  void         (*f)(int x);   // Function to execute with message
  unsigned int    non_vol;    // Storage in NON-VOL
  unsigned int init_value;    // Initial Value
} json_message;

#define IS_VOID       0       // Value is a void
#define IS_INT16      1       // Value is a 16 bit int
#define IS_FLOAT      2       // Value is a floating point number
#define IS_DOUBLE     3       // Value is a double
#define IS_FIXED      4       // The value cannot be changed
    
void reset_JSON(void);            // Clear the JSON input buffer
bool read_JSON(void);             // Scan the serial port looking for JSON input
void show_echo(int v);            // Display the settings

extern int    json_dip_switch;    // DIP switch overwritten by JSON message
extern double json_sensor_dia;    // Sensor radius overwitten by JSON message
extern int    json_sensor_angle;  // Angle sensors are rotated through
extern int    json_paper_time;    // Time to turn on paper backer motor
extern int    json_echo;          // Value to ech
extern int    json_calibre_x10;   // Pellet Calibre
extern int    json_north_x;       // North Adjustment
extern int    json_north_y;
extern int    json_east_x;        // East Adjustment
extern int    json_east_y;
extern int    json_south_x;       // South Adjustment
extern int    json_south_y;
extern int    json_west_x;        // WestAdjustment
extern int    json_west_y;
extern int    json_spare_1;       // Not used
extern int    json_name_id;       // Name Identifier
extern int    json_1_ring_x10;    // Size of 1 ring in mmx10
extern int    json_LED_PWM;       // PWM Setting (%)
extern int    json_power_save;    // How long to run target before turning off LEDs
extern int    json_send_miss;     // Sent the miss message when TRUE
extern int    json_serial_number; // EIN 
extern int    json_step_count;    // Number of times paper motor is stepped
extern int    json_step_time;     // Duration of step pulse
extern int    json_multifunction; // Multifunction switch operation
extern int    json_z_offset;      // Distance between paper and sensor plane (1mm / LSB)
extern int    json_paper_eco;     // Do not advance witness paper if shot is greater than json_paper_eco
extern int    json_target_type;   // Modify the location based on a target type (0 == regular 1 bull target)
#define FIVE_BULL_AIR_RIFLE_74 1  // Target is a five bull air rifle target 74mm centres
#define FIVE_BULL_AIR_RIFLE_79 2  // Target is a five bull air rifle target 79mm centres
extern int    json_tabata_enable; // Tabata Enabled
extern int    json_tabata_on;     // Tabata ON timer
extern int    json_tabata_rest;   // Tabata OFF timer
extern int    json_tabata_cycles; // Number of Tabata cycles
extern int    json_rapid_enable;  // Rapid Fire enabled
extern int    json_rapid_on;      // Rapid Fire ON timer
extern int    json_rapid_rest;    // Rapid Fire OFF timer
extern int    json_rapid_cycles;  // Number of Rapid Fire cycles
extern int    json_rapid_type;    // Type of RApid vire event
extern int    json_vset_PWM;      // Voltage PWM count
extern double json_vset;          // Desired voltage setpont
extern int    json_follow_through;// Follow through timer
extern int    json_keep_alive;    // Keepalive period
extern int    json_tabata_warn_on;  // Time to turn on the warning
extern int    json_tabata_warn_off; // Time to go dark until we start
extern int    json_face_strike;   // Number of cycles to accept a face strike

extern int    json_wifi_channel;  // Channel assigned to this SSID

#endif _JSON_H_
