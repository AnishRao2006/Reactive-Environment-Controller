#if defined (CY_USING_HAL)
#include "cyhal.h"
#endif
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_adc.h"
#include "cyhal_pwm.h"
#include "cyhal_uart.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>

// thresholds for the sensor value
#define high_threshold 60000
#define low_threshold 10000
#define alpha_shift 3

// Sensor pins
#define SENSOR_PIN_VREF P0_0
#define SENSOR_PIN_VNEG P0_1
#define SENSOR_PIN_PWM P0_2

// every single global variable used 
cyhal_timer_t timer_obj;  // timer object
cyhal_adc_t adc_obj;      // adc object
cyhal_adc_channel_t adc_channel_obj;   // adc channel object
uint16_t adc_value; // adc value read from the sensor
volatile bool adc_read_flag = false; // flag to indicate if adc value has been read
const cyhal_adc_channel_config_t adc_channel_cfg = {true, false, 220};
cyhal_pwm_t pwm_obj; // pwm object
volatile bool uart_read_flag = false; // flag to indicate if uart has been read
char uart_read_data[32]; // buffer to store uart read data
volatile uint16_t uart_index = 0; // index to keep track of uart read data
volatile uint16_t tick_counter = 0; // counter to keep track of timer ticks
uint8_t stream = 0;
volatile int32_t ema_value = 0; // variable to store the EMA value
volatile uint16_t adc_filtered_value = 0;
uint16_t high = high_threshold;
uint16_t low = low_threshold;

// funtion declaration
void parse_uart_data(void);
void help(void);
void status(void);
void reset(void);
void config(uint8_t stat);
uint16_t ema_filter(uint16_t new_sample);
void parse_command(char data[]);

// timer ISR
static void timer_isr(void *timer_arg, cyhal_timer_event_t event);

// uart ISR
static void uart_isr(void *callback_arg, cyhal_uart_event_t event);

int main(void)
{
    cy_rslt_t result;
    cy_rslt_t uart_result;

#if defined (CY_DEVICE_SECURE) && defined (CY_USING_HAL)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    // LED
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    // init uart retarget io
    uart_result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CYBSP_DEBUG_UART_CTS, CYBSP_DEBUG_UART_RTS, 115200);

    if(uart_result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    
    // Timer configuration
    cy_rslt_t timer_result;
    timer_result = cyhal_timer_init(&timer_obj, NC, NULL);
    
    if(timer_result != CY_RSLT_SUCCESS)
    {
        printf("Timer init failed!\r\n");
    }

    const cyhal_timer_cfg_t timer_cfg = {
        true,
        CYHAL_TIMER_DIR_UP,
        false,
        1,
        0,
        0
    };

    cy_rslt_t timer_cfg_result = cyhal_timer_configure(&timer_obj, &timer_cfg);

    if(timer_cfg_result != CY_RSLT_SUCCESS)
    {
        printf("Timer configuration failed!\r\n");
    }

    cy_rslt_t timer_freq_result = cyhal_timer_set_frequency(&timer_obj, 100);

    if(timer_freq_result != CY_RSLT_SUCCESS)
    {
        printf("Timer freq set failed!\r\n");
    }

    cyhal_timer_register_callback(&timer_obj, timer_isr, NULL);

    cyhal_timer_enable_event(&timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 10, true);

    cy_rslt_t timer_start_result = cyhal_timer_start(&timer_obj);

    if(timer_start_result != CY_RSLT_SUCCESS)
    {
        printf("Timer start set failed!\r\n");
    }

    // SENSOR ADC configuration
    cy_rslt_t adc_result;
    adc_result = cyhal_adc_init(&adc_obj, SENSOR_PIN_VREF, NULL);

    if(adc_result != CY_RSLT_SUCCESS)
    {
        printf("adc init failed!\r\n");
    }

    cy_rslt_t adc_config_result = cyhal_adc_configure(&adc_obj, NULL);

    if(adc_config_result != CY_RSLT_SUCCESS)
    {
        printf("adc config failed!\r\n");
    }

    cy_rslt_t adc_channel_init = cyhal_adc_channel_init_diff(&adc_channel_obj, &adc_obj, SENSOR_PIN_VREF, CYHAL_ADC_VNEG, &adc_channel_cfg);

    if(adc_channel_init != CY_RSLT_SUCCESS)
    {
        printf("adc channel init failed!\r\n");
    }

    // PWM configuration
    cy_rslt_t pwm_result = cyhal_pwm_init_adv(&pwm_obj, SENSOR_PIN_PWM, NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);

    if(pwm_result != CY_RSLT_SUCCESS)
    {
        printf("pwm init failed!\r\n");
    }

    cy_rslt_t pwm_start = cyhal_pwm_start(&pwm_obj);

    if(pwm_start != CY_RSLT_SUCCESS)
    {
        printf("pwm start failed!\r\n");
    }

    // UART configuration
    cy_rslt_t uart_baud_result = cyhal_uart_set_baud(&cy_retarget_io_uart_obj, 115200, NULL);

    if(uart_baud_result != CY_RSLT_SUCCESS)
    {
        printf("uart baud set failed!\r\n");
    }

    cy_rslt_t uart_set_fifo_result = cyhal_uart_set_fifo_level(&cy_retarget_io_uart_obj, CYHAL_UART_FIFO_RX, 4);

    if(uart_set_fifo_result != CY_RSLT_SUCCESS)
    {
        printf("uart set fifo level failed!\r\n");
    }

    cyhal_uart_register_callback(&cy_retarget_io_uart_obj, uart_isr, NULL);

    cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY, 9, true);

    while(1)
    {
        // read the adc value
        if(adc_read_flag)
        {
            adc_read_flag = false;
            adc_value = cyhal_adc_read_u16(&adc_channel_obj);
            // filtering done using ema filter
            adc_filtered_value = ema_filter(adc_value);
        }

        // hysteris control for the pwm output based on adc value
        if(adc_filtered_value > high)
        {
            // set the pwm value using the adc value
            cyhal_pwm_set_duty_cycle(&pwm_obj, (cy_float32_t)1000, 1000);
        }
        else if(adc_filtered_value < low)
        {
            // set the pwm value using the adc value
            cyhal_pwm_set_duty_cycle(&pwm_obj, (cy_float32_t)0, 1000);
        }

        if(uart_read_flag)
        {
            uart_read_flag = false;
            parse_uart_data();
        }

        // decides to stream on or off
        config(stream);

        if(stream == 0) 
        {
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        }
        else if(stream == 1)
        {
            // 100ms off 100ms on
            if(tick_counter % 20 < 10)
            {
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
            }
            else 
            {
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            }
        }
        else if(stream == 2)
        {
            // 500ms off 500ms on
            if(tick_counter % 100 < 50)
            {
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
            }
            else 
            {
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            }
        }
    }
    
    return 0;
}

static void timer_isr(void *timer_arg, cyhal_timer_event_t event)
{
    tick_counter++; // 10ms
    adc_read_flag = true;
}

static void uart_isr(void *callback_arg, cyhal_uart_event_t event)
{
    uart_read_flag = true;
}

void parse_uart_data(void)
{
    size_t lenght;
    unsigned char ch;

    while(cyhal_uart_readable(&cy_retarget_io_uart_obj) > 0)
    {
        if(cyhal_uart_read(&cy_retarget_io_uart_obj, &ch, &lenght) == CY_RSLT_SUCCESS && lenght == 1)
        {
            if(ch == '\n' || ch == '\r')
            {
                uart_read_data[uart_index] = '\0'; // null terminate the string
                if(uart_index > 0) 
                {
                    parse_command(uart_read_data);
                    uart_index = 0; // reset index for next read
                }
            }
            else if(uart_index < sizeof(uart_read_data) - 1)
            {
                uart_read_data[uart_index++] = ch; // store the character and increment index
            }
        }
    }
}

void help(void)
{
    printf("Available commands:\r\n");
    printf("HELP - Show this help message\r\n");
    printf("STATUS - Show the current status\r\n");
    printf("RESET - Reset the system\r\n");
    printf("STREAM OFF / STREAM ON - Configure the system to enable or disable streaming\r\n");
    printf("SET THRESH <n> - helps to change the higher threshold\r\n");
}

void status(void)
{
    printf("ADC value = %lu\r\n", (uint32_t)adc_value);
    printf("PWM duty cycle = %s\r\n", (adc_filtered_value > high) ? "100%" : (adc_filtered_value < low) ? "0%" : "hold");
    printf("ADC filtered value = %u\r\n", adc_filtered_value);
    printf("high and low thresholds are %u and %u\r\n", high, low);
}

void reset(void)
{
    printf("System reset command received. Performing reset...\r\n");
    ema_value = 0;
    stream = 1;
    uart_index = 0;
    low = low_threshold;
    high = high_threshold;
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
}

void config(uint8_t stat)
{
    if(stat == 0)
    {
        if(tick_counter >= 100)
        {
            printf("ADC value = %lu\r\n", (uint32_t)adc_value);
            printf("PWM duty cycle = %s\r\n", (adc_filtered_value > high) ? "100%" : (adc_filtered_value < low) ? "0%" : "hold");
            printf("ADC filtered value = %u\r\n", adc_filtered_value);
            printf("high and low thresholds are %u and %u\r\n", high, low);
            tick_counter = 0;
        }
    }
}

uint16_t ema_filter(uint16_t new_sample)
{
    ema_value += ((int32_t)new_sample - (int32_t)ema_value) / (1 << alpha_shift);
    return (uint16_t)ema_value;
}

void parse_command(char data[])
{
    if(strcasecmp(data, "HELP") == 0)
    {
        help();
        return;
    }
    else if(strcasecmp(data, "STATUS") == 0)
    {
        status();
        return;
    }
    else if(strcasecmp(data, "RESET") == 0)
    {
        reset();
        return;
    }
    else if(strcasecmp(data, "STREAM ON") == 0)
    {
        stream = 0;  //stream is on
        return;
    }
    else if(strcasecmp(data, "STREAM OFF") == 0)
    {
        stream = 1;  //going to idle cause stream off is idle
        return;
    }
    else if(strncasecmp(data, "SET THRESH ", 11) == 0)
    {
        long val = strtol(data + 11, NULL, 10);
        if (val >= low && val <= 65535) 
        {
            high = (uint16_t)val;
        }
        else 
        { 
            printf("ERR: threshold out of range\r\n"); 
        }
    }
    else 
    {
        stream = 2;  //error command
        printf("unknown command, please enter a valid one\r\n");
        return;
    }
}

/* [] END OF FILE */
