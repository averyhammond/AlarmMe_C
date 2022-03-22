#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sensors.h"


// Main control flow for AlarmMe
void app_main(void)
{
    init();  // Initialize peripherals
    setup_timer();  // Setup timer for 1Hz operation
    run();  // Regular Operation

    // Should never reach here
    printf("Reached return in main... error\n");
    return;
}


// Regular operation
void run(void)
{
    while(1) {

        // Loop to display sensor data
        display_sensor_data();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return;  // Should never return
}