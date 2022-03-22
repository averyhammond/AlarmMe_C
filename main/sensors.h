#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/timer.h"

// Sensor_Data Struct to hold all necessary sensor values
struct Sensor_Data {
  uint32_t curr_temp;
  uint32_t curr_pres;
};


// Global Sensor struct declaration
struct Sensor_Data Sensors = {
    .curr_temp = 0,
    .curr_pres = 0
};


// Function Declarations
void adc_init(void);
void setup_timer(void);
void display_sensor_data(void);
void run(void);
void init(void);
static void IRAM_ATTR get_sensor_data(void *args);
uint32_t get_temp_data(void);
uint32_t get_pres_data(void);


// Caller function to initialize all necessary peripherals
void init(void)
{
    adc_init();

    return;
}

// Function to setup ADC1 Channel 6 and ADC1 Channel 3 to take outputs from the temperature and pressure sensors, respectively
void adc_init(void) 
{
    adc1_config_width(ADC_WIDTH_BIT_12);  // Configure for 12 bit resolution
    adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_11);  // IO7 on ADC 1, 11 DB Attenuation, temperature sensor
    adc1_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN_DB_11);  // IO39 on ADC1, 11 DB, Attenuation, pressure sensor

    return;
}


// Function to configure a timer to sample at 1Hz
void setup_timer(void)
{
    // Initialize timer config structure
    timer_config_t config = {
        .divider = 8000,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = false,
        .alarm_en = true,
        .auto_reload = true,
        .intr_type = TIMER_INTR_LEVEL,
    };

    // Initialize timer attributes
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 10000);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, get_sensor_data, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    
    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);

    return;
}


// Debugging function to print current sensor data to terminal
void display_sensor_data()
{
    printf("Temp: %d\nPres: %d\n", Sensors.curr_temp, Sensors.curr_pres);

    return;
}


// Funtion to sample all sensors and upload results to global Sensors data struct
static void IRAM_ATTR get_sensor_data(void *args)
{
    // TODO: Clear Flags

    Sensors.curr_temp = get_temp_data();
    Sensors.curr_pres = get_pres_data();

    // TODO: Process Data

    Sensors = Sensors;  // Errors otherwise

    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;
    return;
}


// Function to sample the temperature sensor via ADC
uint32_t get_temp_data(void)
{
    uint32_t adc1_ch6_temp = 0;
    adc1_ch6_temp += adc1_get_raw((adc1_channel_t) ADC_CHANNEL_6);
    
    return ((((1.8663 - ((adc1_ch6_temp * 3.3) / 4095)) / .01169) * 9) / 5) + 32 - 20;
}


// Function to sample the pressure sensor via ADC
uint32_t get_pres_data(void)
{
    uint32_t adc1_ch3_pres = 0;
    adc1_ch3_pres += adc1_get_raw((adc1_channel_t) ADC_CHANNEL_3);
        
    return adc1_ch3_pres;
}