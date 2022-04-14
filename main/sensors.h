#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/timer.h"
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "driver/i2c.h"
#include "soc/rtc_wdt.h"

#define SLEEP_TIME_THRESH 10


// Function Declarations
void init(void);
void ADC_init(void);
void GPIO_init(void);
void I2C_init(void);
void display_sensor_data(void);
void run(void);
void get_sensor_data(void);
uint32_t get_temp_data(void);
uint32_t get_pres_data(void);
uint32_t get_co2_data(void);
void process_data(void);
void clear_flags(void);
void set_alarms(void);
void alarm(void);


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
    GPIO_init();
    I2C_init();

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

    // Configure IO25 as an output for buzzer alarm
    gpio_pad_select_gpio(25);
    gpio_set_direction(25, GPIO_MODE_OUTPUT);

    return;
}


// Function to initialize all the necessary I2C connections
void I2C_init()
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 23,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    i2c_param_config(I2C_NUM_0, &config);
    i2c_driver_install(I2C_NUM_0, config.mode, 0, 0, 0);

    uint8_t init[2] = {0x20, 0x03};
    uint8_t meas[2] = {0x20, 0x08};
    
    i2c_master_write_to_device(I2C_NUM_0, 0x58, init, 2, 1000 / portTICK_RATE_MS);
    vTaskDelay(10 / portTICK_RATE_MS);

    i2c_master_write_to_device(I2C_NUM_0, 0x58, meas, 2, 1000 / portTICK_RATE_MS);
    vTaskDelay(100 / portTICK_RATE_MS);

    return;
}


// Debugging function to print current sensor data to terminal
void display_sensor_data()
{
    printf("Temp: %d\nPres: %d\nCO2: %d\n", Sensors.curr_temp, Sensors.curr_pres, Sensors.curr_co2);

    return;
}


// Funtion to sample all sensors and upload results to global Sensors data struct
void get_sensor_data(void)
{
    Sensors.curr_temp = get_temp_data();
    Sensors.curr_pres = get_pres_data();
    Sensors.curr_co2 = get_co2_data();

    Sensors = Sensors;  // Errors otherwise

    return;
}


// Function to interpret sensor data to set alarm flags and enter alarm states
void process_data(void)
{
    clear_flags();
    set_alarms();

    // Case: Alarm flags have been set, trigger alarm state
    if ((Sensors.alarm_temp || Sensors.alarm_co2) && Sensors.curr_pres == 4095)
    {
        alarm();
    }

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

    // Reset LED and buzzer to OFF
    gpio_set_level(4, 0);
    gpio_set_level(25, 0);

    return;
}


// Function to check the current readings and determine if alarm state should be triggered
void set_alarms()
{
    if ((Sensors.curr_temp > 70) && (Sensors.curr_pres > 2000))
    {
        Sensors.alarm_temp = 1;
    }

    return;
}


// Alarm state function: triggers buzzer and LED light and sends SMS messages
void alarm()
{
    // Enable the red LED
    gpio_set_level(4, 1);
    gpio_set_level(25, 1);
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


// Function to read the CO2 in ppm via I2C
uint32_t get_co2_data()
{
    uint32_t co2_data = 0;
    uint8_t meas[2] = {0x20, 0x08};
    uint8_t res[6];

    i2c_master_write_to_device(I2C_NUM_0, 0x58, meas, 2, 1000 / portTICK_RATE_MS);
    vTaskDelay(100 / portTICK_RATE_MS);

    i2c_master_read_from_device(I2C_NUM_0, 0x58, res, 6, 1000 / portTICK_RATE_MS);
    vTaskDelay(6 / portTICK_RATE_MS);

    uint8_t temp_res[2] = {res[0], res[1]};
    co2_data = temp_res[0] << 8 | temp_res[1];

    return co2_data;
}