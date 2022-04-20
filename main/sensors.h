#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_sleep.h"
#include "driver/i2c.h"
#include "driver/uart.h"


// Function Declarations
void ADC_init(void);
void GPIO_init(void);
void I2C_init(void);
void UART_init(void);
void display_sensor_data(void);
void get_temp_data(void *argv);
void get_pres_data(void *argv);
void get_co2_data(void *argv);
void process_data(void);
void clear_flags(void);
void set_alarms(void);
void alarm(void);
void print_SIM_Response(char * response, int start, int len);


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


// Function to setup ADC1 Channel 6 and ADC1 Channel 3 to take outputs from the temperature and pressure sensors, respectively
void ADC_init(void) 
{
    printf("Beginning ADC_init()...\n\n");

    adc1_config_width(ADC_WIDTH_BIT_12);  // Configure for 12 bit resolution
    adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_11);  // IO7 on ADC 1, 11 DB Attenuation, temperature sensor
    adc1_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN_DB_11);  // IO39 on ADC1, 11 DB, Attenuation, pressure sensor

    printf("ADC peripherals successfully initialized!\n\n");

    return;
}



// Function to initialize all the necessary GPIO ports
void GPIO_init()
{
    printf("Beginning GPIO_init()...\n\n");

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

    printf("GPIO pins successfully initialized!\n\n");

    return;
}


// Function to initialize all the necessary I2C connections
void I2C_init()
{
    printf("Beginning I2C_init()...\n\n");

    // Configure I2C bus
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

    // Commands to CO2 sensor
    uint8_t init[2] = {0x20, 0x03};
    uint8_t meas[2] = {0x20, 0x08};
    
    // Write initialization command to CO2 sensor
    i2c_master_write_to_device(I2C_NUM_0, 0x58, init, 2, 1000 / portTICK_RATE_MS);
    vTaskDelay(10 / portTICK_RATE_MS);

    // Write measure command to CO2 sensor
    i2c_master_write_to_device(I2C_NUM_0, 0x58, meas, 2, 1000 / portTICK_RATE_MS);
    vTaskDelay(100 / portTICK_RATE_MS);

    printf("I2C devices successfully initialized!\n\n");

    return;
}


// Function to initialize the SIM module though UART
void UART_init(void)
{
    printf("Beginning UART_init()...\n\n");
    
    // Configure TX as IO17 and RX as IO16
    gpio_set_direction(17, GPIO_MODE_OUTPUT);
	gpio_set_direction(16, GPIO_MODE_INPUT);

    // Configure UART parameters
    const uart_port_t uart_num = UART_NUM_2; // UART channel 2 
    uart_config_t uart_config = {
        //.baud_rate = 115200,
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
    };

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, 1024 * 2, 0, 0, NULL, 0);
    
    char data[256];
    int len = 0;

    // Send AT command to SIM
    printf("Sending \'AT\' to SIM Module...\n");
    uart_write_bytes(uart_num, "AT\r\n", sizeof("AT\r\n") - 1);
	uart_wait_tx_done(uart_num, 100 / portTICK_RATE_MS); 
    len = uart_read_bytes(uart_num, (uint8_t *)data, 256, 3000 / portTICK_RATE_MS);
    print_SIM_Response(data, 1, len);

    // Send AT+CSQ command to SIM
    printf("Sending \'AT=CSQ\' to SIM Module...\n");
    uart_write_bytes(uart_num, "AT+CSQ\r\n", sizeof("AT+CSQ\r\n") - 1); 
	uart_wait_tx_done(uart_num, 100 / portTICK_RATE_MS); 
    len = uart_read_bytes(uart_num, (uint8_t *)data, 256, 3000 / portTICK_RATE_MS);
    print_SIM_Response(data, 0, len);

    printf("UART Peripheral successfully initialized!\n\n");

    return; 
}


// Debugging function to print current sensor data to terminal
void display_sensor_data()
{
    printf("Temp: %d\nPres: %d\nCO2: %d\nNo Pres Time: %d\n\n", Sensors.curr_temp, Sensors.curr_pres, Sensors.curr_co2, Sensors.no_pres_time);

    return;
}


// Function to interpret sensor data to set alarm flags and enter alarm states
void process_data(void)
{
    clear_flags();
    set_alarms();

    if (Sensors.curr_pres == 0)
    {
        Sensors.no_pres_time += 1;

        if (Sensors.no_pres_time == 20)
        {
            esp_deep_sleep_start();
        }
    }
    else
    {
        Sensors.no_pres_time = 0;
    }

    // Case: Alarm flags have been set, trigger alarm state
    if (Sensors.alarm_temp || Sensors.alarm_co2)
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
    if ((Sensors.curr_temp > 70) && (Sensors.curr_pres == 4095))
    {
        Sensors.alarm_temp = 1;
    }

    if ((Sensors.curr_co2 < 400) && (Sensors.curr_pres == 4095))
    {
        Sensors.alarm_co2 = 1;
    }

    return;
}


// Alarm state function: triggers buzzer and LED light and sends SMS messages
void alarm()
{
    // Enable the red LED
    gpio_set_level(4, 1);

    // Enable buzzer
    gpio_set_level(25, 1);

    return;
}


void get_temp_data(void *argv)
{
    uint32_t temp = 0;

    while(1)
    {
        temp = adc1_get_raw((adc1_channel_t) ADC_CHANNEL_6);
        Sensors.curr_temp = ((((1.8663 - ((temp * 3.3) / 4095)) / .01169) * 9) / 5) + 32 - 20;
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    return;  // Should never reach
}


void get_pres_data(void *argv)
{
    uint32_t pres = 0;
    
    while(1)
    {
        pres = adc1_get_raw((adc1_channel_t) ADC_CHANNEL_3);
        Sensors.curr_pres = pres;
        vTaskDelay(100 / portTICK_RATE_MS);
    }
        
    return;
}


void get_co2_data(void *argv)
{
    uint32_t co2_data = 0;
    uint8_t meas[2] = {0x20, 0x08};
    uint8_t res[6] = {0x00, 0x00};

    while(1)
    {
        i2c_master_write_to_device(I2C_NUM_0, 0x58, meas, 2, 1000 / portTICK_RATE_MS);
        vTaskDelay(50 / portTICK_RATE_MS);

        i2c_master_read_from_device(I2C_NUM_0, 0x58, res, 6, 1000 / portTICK_RATE_MS);
        vTaskDelay(6 / portTICK_RATE_MS);

        uint8_t temp_res[2] = {res[0], res[1]};
        co2_data = temp_res[0] << 8 | temp_res[1];

        Sensors.curr_co2 = co2_data;
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    return;
}


void print_SIM_Response(char * response, int start, int len)
{
    
    printf("Cellular Module Response:\n");  
    printf("-----------------------------------------------------------\n"); 

    int i; 
    for(i = start; i < len; i++)
    {
        printf("%c", response[i]); 
    }

    printf("-----------------------------------------------------------\n"); 
}