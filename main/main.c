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

    // Initialize sensor tasks
    xTaskCreate(get_temp_data, "get_temp_data", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(get_pres_data, "get_pres_data", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    xTaskCreate(get_co2_data, "get_co2_data", 2048, NULL, 6, NULL);

    // Regular Operation
    //run();

    while(1) {

        // Loop to display sensor data
        display_sensor_data();
        //get_sensor_data();
        //process_data();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

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
        //get_sensor_data();
        //process_data();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return;  // Should never return
}