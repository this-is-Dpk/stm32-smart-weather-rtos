#include "process.h"
void do_storm_alert(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Storm Alert: Flash RED LED & Buzz\n", strlen("Storm Alert: Flash RED LED & Buzz\n"), 1000);
}

void do_heatwave_response(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Heatwave: Turn ON fan\n", strlen("Heatwave: Turn ON fan\n"), 1000);
}

void do_rain_protection(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Rain Alert: Close irrigation valve\n", strlen("Rain Alert: Close irrigation valve\n"), 1000);
}

void do_normal_mode(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Normal Mode: Turn off alerts & enter low power\n", strlen("Normal Mode: Turn off alerts & enter low power\n"), 1000);
}

void do_data_log(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Data Log: Save sensor data to SD card\n", strlen("Data Log: Save sensor data to SD card\n"), 1000);
}

void do_cloud_upload_prep(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Cloud Upload: Prepare JSON packet\n", strlen("Cloud Upload: Prepare JSON packet\n"), 1000);
}

void do_emergency_shutdown(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Emergency Override: Disable all outputs\n", strlen("Emergency Override: Disable all outputs\n"), 1000);
}

void do_display_readings(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Display Mode: Show temp, pressure, humidity\n", strlen("Display Mode: Show temp, pressure, humidity\n"), 1000);
}

void do_wind_advisory(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Wind Advisory: Blink yellow LED & deploy antenna\n", strlen("Wind Advisory: Blink yellow LED & deploy antenna\n"), 1000);
}

void do_recalibrate(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Recalibration: Self-check sensors\n", strlen("Recalibration: Self-check sensors\n"), 1000);
}
