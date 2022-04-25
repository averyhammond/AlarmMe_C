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

// Task Declarations
#define TASK_STACK_SIZE 2048
#define TASK_DELAY 100 / portTICK_RATE_MS
#define TIMER_DELAY 1000 / portTICK_RATE_MS

// UART Declarations
#define UART_2_TX 17
#define UART_2_RX 16
#define UART_2_BR 9600
#define TX_BUFFER_SIZE 2048
#define AT_CMD "AT\r\n"
#define AT_CMD_LEN sizeof("AT\r\n") - 1
#define AT_CSQ_CMD "AT+CSQ\r\n"
#define AT_CSQ_CMD_LEN sizeof("AT+CSQ\r\n") - 1
#define AT_TX_DELAY 100 / portTICK_RATE_MS
#define AT_RX_DELAY 3000 / portTICK_RATE_MS

// GPIO Declarations
#define EXTERNAL_WAKEUP_PIN_NUM 26
#define EXTERNAL_WAKEUP_PIN_BITS 1LL << 26
#define LED_PIN_NUM 4
#define BUZZER_PIN_NUM 25

// I2C Declarations
#define I2C_BUS_SDA 23
#define I2C_BUS_SCL 22
#define I2C_CLK 400000
#define SGP30_ADDR 0x58
#define I2C_TRANS_DELAY 1000 / portTICK_RATE_MS
#define SGP30_INIT_DELAY 10 / portTICK_RATE_MS
#define SGP30_MEAS_DELAY 100 / portTICK_RATE_MS
#define SGP30_MEAS_READ_DELAY 50 / portTICK_RATE_MS
#define SGP30_MEAS_SIZE 2
#define SGP30_READ_DELAY 6
#define SGP30_READ_SIZE 6

// Regular Operation Declarations
#define SLEEP_TIME_THRESH 90
#define UNSAFE_TEMP_TRESH 70
#define UNSAFE_C02_THRESH 400
#define MAX_PRESSURE 4095


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
    gpio_pad_select_gpio(EXTERNAL_WAKEUP_PIN_NUM);
    gpio_set_direction(EXTERNAL_WAKEUP_PIN_NUM, GPIO_MODE_INPUT);
    esp_sleep_enable_ext1_wakeup(EXTERNAL_WAKEUP_PIN_BITS, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Configure IO4 as an output for LED alarm
    gpio_pad_select_gpio(LED_PIN_NUM);
    gpio_set_direction(LED_PIN_NUM, GPIO_MODE_OUTPUT);

    // Configure IO25 as an output for buzzer alarm
    gpio_pad_select_gpio(BUZZER_PIN_NUM);
    gpio_set_direction(BUZZER_PIN_NUM, GPIO_MODE_OUTPUT);

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
        .sda_io_num = I2C_BUS_SDA,
        .scl_io_num = I2C_BUS_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK,
    };

    i2c_param_config(I2C_NUM_0, &config);
    i2c_driver_install(I2C_NUM_0, config.mode, 0, 0, 0);

    // Commands to CO2 sensor
    uint8_t init[2] = {0x20, 0x03};
    uint8_t meas[2] = {0x20, 0x08};
    
    // Write initialization command to CO2 sensor
    i2c_master_write_to_device(I2C_NUM_0, SGP30_ADDR, init, 2, I2C_TRANS_DELAY);
    vTaskDelay(SGP30_INIT_DELAY);

    // Write measure command to CO2 sensor
    i2c_master_write_to_device(I2C_NUM_0, SGP30_ADDR, meas, 2, I2C_TRANS_DELAY);
    vTaskDelay(SGP30_MEAS_DELAY);

    printf("I2C devices successfully initialized!\n\n");

    return;
}


// Function to initialize the SIM module though UART
void UART_init(void)
{
    printf("Beginning UART_init()...\n\n");
    
    // Configure TX as IO17 and RX as IO16
    gpio_set_direction(UART_2_TX, GPIO_MODE_OUTPUT);
	gpio_set_direction(UART_2_RX, GPIO_MODE_INPUT);

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_2_BR,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
    };

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART_2_TX, UART_2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, TX_BUFFER_SIZE, 0, 0, NULL, 0);
    
    char data[256];
    int len = 0;

    // Send AT command to SIM
    printf("Sending \'AT\' to SIM Module...\n");
    uart_write_bytes(UART_NUM_2, AT_CMD, AT_CMD_LEN);
	uart_wait_tx_done(UART_NUM_2, AT_TX_DELAY); 
    len = uart_read_bytes(UART_NUM_2, (uint8_t *)data, 256, AT_RX_DELAY);
    print_SIM_Response(data, 1, len);

    // Send AT+CSQ command to SIM
    printf("Sending \'AT=CSQ\' to SIM Module...\n");
    uart_write_bytes(UART_NUM_2, AT_CSQ_CMD, AT_CSQ_CMD_LEN); 
	uart_wait_tx_done(UART_NUM_2, AT_TX_DELAY); 
    len = uart_read_bytes(UART_NUM_2, (uint8_t *)data, 256, AT_RX_DELAY);
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
    // Reset alarms from previous iteration, set alarms for this iteration
    clear_flags();
    set_alarms();

    // Case: Pressure reads 0, no child in car seat, increment no_pres_time
    if (Sensors.curr_pres == 0)
    {
        Sensors.no_pres_time += 1;

        // Case: No child detected in seat for SLEEP_TIME_THRESH seconds, put AlarmMe into deep sleep
        if (Sensors.no_pres_time == SLEEP_TIME_THRESH)
        {
            esp_deep_sleep_start();
        }
    }
    
    // Case: Pressure reads non_zero, child detected in seat
    else
    {
        // Reset no_pres_time since a non-zero pressure was read
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
    gpio_set_level(LED_PIN_NUM, 0);
    gpio_set_level(BUZZER_PIN_NUM, 0);

    return;
}


// Function to check the current readings and determine if alarm state should be triggered
void set_alarms()
{
    // Case: Unsafe temperature reading and child detected in seat, trigger temperature alarm
    if ((Sensors.curr_temp > UNSAFE_TEMP_TRESH) && (Sensors.curr_pres == MAX_PRESSURE))
    {
        Sensors.alarm_temp = 1;
    }

    // Case: Unsafe CO2 reading and child detected in seat, trigger CO2 alarm
    if ((Sensors.curr_co2 < UNSAFE_C02_THRESH) && (Sensors.curr_pres == MAX_PRESSURE))
    {
        Sensors.alarm_co2 = 1;
    }

    return;
}


// Alarm state function: triggers buzzer and LED light and sends SMS messages
void alarm()
{
    // Enable the red LED
    gpio_set_level(LED_PIN_NUM, 1);

    // Enable buzzer
    gpio_set_level(BUZZER_PIN_NUM, 1);

    return;
}


// Function to sample the temperature sensor via ADC
void get_temp_data(void *argv)
{
    // Current temperature value
    uint32_t temp = 0;

    // Task loop
    while(1)
    {
        // Sample ADC
        temp = adc1_get_raw((adc1_channel_t) ADC_CHANNEL_6);
        Sensors.curr_temp = (((((1.8663 - ((temp * 3.3) / 4095)) / .01169) * 9) / 5) + 32 - 20) - 7;
        
        // Delay before next iteration
        vTaskDelay(TASK_DELAY);
    }

    return;  // Should never reach
}


// Function to sample the pressure sensor via ADC
void get_pres_data(void *argv)
{
    // Current pressure value
    uint32_t pres = 0;
    
    // Task loop
    while(1)
    {
        // Sample ADC
        pres = adc1_get_raw((adc1_channel_t) ADC_CHANNEL_3);
        Sensors.curr_pres = pres;

        // Delay before next iteration
        vTaskDelay(TASK_DELAY);
    }
        
    return;
}


// Function to sample the CO2 sensor via I2C bus
void get_co2_data(void *argv)
{
    // Current co2 measurements
    uint32_t co2_data = 0;
    
    // Command bytes
    uint8_t meas[2] = {0x20, 0x08};
    uint8_t res[6] = {0x00, 0x00};

    // Task loop
    while(1)
    {
        // Send measure command to device and wait
        i2c_master_write_to_device(I2C_NUM_0, SGP30_ADDR, meas, SGP30_MEAS_SIZE, I2C_TRANS_DELAY);
        vTaskDelay(SGP30_MEAS_READ_DELAY);

        // Send read command to device and wait
        i2c_master_read_from_device(I2C_NUM_0, SGP30_ADDR, res, SGP30_READ_SIZE, I2C_TRANS_DELAY);
        vTaskDelay(SGP30_READ_DELAY);

        // Process return bytes
        uint8_t temp_res[2] = {res[0], res[1]};
        co2_data = temp_res[0] << 8 | temp_res[1];
        Sensors.curr_co2 = co2_data;

        // Delay before next iteration
        vTaskDelay(TASK_DELAY);
    }

    return;
}


// Function to print the responses from the cellular module
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