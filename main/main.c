#include <stdio.h>
#include <stdlib.h>
#include "sensors.h"


// Main control flow for AlarmMe
void app_main(void)
{
    vTaskDelay(500 / portTICK_RATE_MS);

    // Initialize all peripherals
    UART_init();
    ADC_init();
    GPIO_init();
    I2C_init();

    // Regular Operation
    run();

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
        get_sensor_data();
        process_data();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return;  // Should never return
}