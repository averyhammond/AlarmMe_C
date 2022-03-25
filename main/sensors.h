#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/timer.h"
#include "esp_sleep.h"
#include "soc/rtc.h"

#define SLEEP_TIME_THRESH 10


// Function Declarations
void ADC_init(void);
void setup_timer(void);
void display_sensor_data(void);
void run(void);
void init(void);
static void IRAM_ATTR get_sensor_data(void *args);
uint32_t get_temp_data(void);
uint32_t get_pres_data(void);
void clear_flags(void);
void process_data(void);
void GPIO_init(void);
void alarm(void);
void set_alarms(void);


// Sensor_Data Struct to hold all necessary sensor values
struct Sensor_Data {
  uint32_t curr_temp;
  bool invalid_temp;
  bool alarm_temp;
  uint32_t curr_pres;
  bool invalid_pres;
  uint32_t no_pres_time;
  uint32_t curr_co2;
  bool alarm_co2;
};


// Global Sensor struct declaration
struct Sensor_Data Sensors = {
    .curr_temp = 0,
    .invalid_temp = false,
    .alarm_temp = false,
    .curr_pres = 0,
    .invalid_pres = false,
    .no_pres_time = 0,
    .curr_co2 = 0,
    .alarm_co2 = false
};


// Caller function to initialize all necessary peripherals
void init(void)
{
    ADC_init();
    // TODO: UART_init();
    // TODO: I2C_init();
    GPIO_init();

    return;
}

// Function to setup ADC1 Channel 6 and ADC1 Channel 3 to take outputs from the temperature and pressure sensors, respectively
void ADC_init(void) 
{
    adc1_config_width(ADC_WIDTH_BIT_12);  // Configure for 12 bit resolution
    adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_11);  // IO7 on ADC 1, 11 DB Attenuation, temperature sensor
    adc1_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN_DB_11);  // IO39 on ADC1, 11 DB, Attenuation, pressure sensor

    return;
}


// TODO: UART_init()


// TODO: I2C_init()


// Function to initialize all the necessary GPIO ports
void GPIO_init()
{
    // Configure IO26 as an input for external wakeup
    gpio_pad_select_gpio(26);
    gpio_set_direction(26, GPIO_MODE_INPUT);
    esp_sleep_enable_ext1_wakeup(1LL << 26, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Configure IO4 as an output for LED alarm
    gpio_pad_select_gpio(4);
    gpio_set_direction(4, GPIO_MODE_OUTPUT);

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
    printf("Temp: %d\nPres: %d\nTime w/ No Pres: %d\n", Sensors.curr_temp, Sensors.curr_pres, Sensors.no_pres_time);
    printf("alarm_temp: %d\n", Sensors.alarm_temp);

    return;
}


// Funtion to sample all sensors and upload results to global Sensors data struct
static void IRAM_ATTR get_sensor_data(void *args)
{
    clear_flags();

    Sensors.curr_temp = get_temp_data();
    Sensors.curr_pres = get_pres_data();
    // TODO: get co2 data

    process_data();

    Sensors = Sensors;  // Errors otherwise

    // Prepare for next timer iteration
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    return;
}


// Function to clear any flags before next timer fires
void clear_flags(void)
{
    // Clear flags
    Sensors.invalid_temp = false;
    Sensors.invalid_pres = false;
    Sensors.alarm_temp = false;
    Sensors.alarm_co2 = false;

    // Reset LED to OFF
    gpio_set_level(4, 0);
    // TODO: Reset buzzer to OFF

    return;
}


// Function to interpret sensor data to set alarm flags and enter alarm states
void process_data(void)
{
    // TODO: set_alarms()
    set_alarms();

    // Case: Alarm flags have been set, trigger alarm state
    if ((Sensors.alarm_temp || Sensors.alarm_co2) && Sensors.curr_pres > 2000)
    {
        alarm();
    }

    // Case: No child in seat, increment no_pres_time
    if (Sensors.curr_pres < 2000)
    {
        Sensors.no_pres_time += 1;

        // Case: No child in seat for 60 seconds, put AlarmMe into deep sleep
        if (Sensors.no_pres_time >= SLEEP_TIME_THRESH)
        {
            esp_deep_sleep_start();
            return;
        }
    }

    // Case: Child detected in seat, set no_pres_time to 0
    else if (Sensors.curr_pres > 0)
    {
        Sensors.no_pres_time = 0;
    }

    return;
}


// Function to check the current readings and determine if alarm state should be triggered
void set_alarms()
{
    if ((Sensors.curr_temp > 70) && (Sensors.curr_pres > 2000))
    {
        Sensors.alarm_temp = 1;
    }

    // TODO: Check CO2 readings

    return;
}


// Alarm state function: triggers buzzer and LED light and sends SMS messages
void alarm()
{
    // Enable the red LED
    gpio_set_level(4, 1);

    // TODO: Enable buzzer

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