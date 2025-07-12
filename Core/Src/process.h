/*
 * process.h
 *
 *  Created on: Jul 6, 2025
 *      Author: dpk77
 */

#ifndef SRC_PROCESS_H_
#define SRC_PROCESS_H_
#include "main.h"
#include<string.h>
extern UART_HandleTypeDef huart1;
void do_storm_alert(void);             // P01 - Flash LED & Buzzer
void do_heatwave_response(void);       // P02 - Turn on fan
void do_rain_protection(void);         // P03 - Close valve
void do_normal_mode(void);             // P04 - Exit alert state
void do_data_log(void);                // P05 - Log to SD card
void do_cloud_upload_prep(void);       // P06 - Prepare JSON for cloud
void do_emergency_shutdown(void);      // P07 - Disable outputs
void do_display_readings(void);        // P08 - Show sensor values
void do_wind_advisory(void);           // P09 - Antenna & blink LED
void do_recalibrate(void);             // P10 - Self-check sensors


#endif /* SRC_PROCESS_H_ */
