#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// Pin definitions
#define PWM_INPUT_PIN 15  // GPIO pin to read the PWM signal (digital signal)
#define ADC_PIN 26        // GPIO pin for ADC input (analog signal)

// Variables to store pulse width and period timings
absolute_time_t rising_edge_time;
absolute_time_t falling_edge_time;
absolute_time_t previous_rising_edge_time;
uint64_t pulse_width_us = 0;  // Pulse width measured in microseconds
uint64_t period_us = 0;       // Period of the PWM signal in microseconds
float duty_cycle = 0.0f;      // Duty cycle as a percentage
float frequency_hz = 0.0f;    // Frequency in Hz

// GPIO Interrupt Handler to measure the pulse width and period
void gpio_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        // Rising edge detected
        rising_edge_time = get_absolute_time();

        // If it's not the first rising edge, calculate the period
        if (absolute_time_diff_us(previous_rising_edge_time, rising_edge_time) > 0) {
            // Calculate the period (time between two rising edges)
            period_us = absolute_time_diff_us(previous_rising_edge_time, rising_edge_time);

            // Calculate the frequency (1 / period)
            if (period_us > 0) {
                frequency_hz = 1e6 / period_us;  // Convert to Hz (1 second = 1e6 microseconds)
            }

            // Calculate the duty cycle (pulse width / period) * 100 to get percentage
            if (period_us > 0) {
                duty_cycle = (pulse_width_us * 100.0) / period_us;
            }

            // Print duty cycle and frequency
            printf("Duty Cycle: %.2f%%, Frequency: %.2f Hz\n", duty_cycle, frequency_hz);
        }

        // Save the current rising edge time for the next period calculation
        previous_rising_edge_time = rising_edge_time;
    }

    if (events & GPIO_IRQ_EDGE_FALL) {
        // Falling edge detected, calculate the pulse width (time signal is high)
        falling_edge_time = get_absolute_time();
        pulse_width_us = absolute_time_diff_us(rising_edge_time, falling_edge_time);
    }
}

// Function to configure GPIO to measure pulse width and period
void configure_gpio_for_pwm() {
    gpio_init(PWM_INPUT_PIN);
    gpio_set_dir(PWM_INPUT_PIN, GPIO_IN);
    gpio_pull_down(PWM_INPUT_PIN);

    // Set GPIO interrupt on rising and falling edges
    gpio_set_irq_enabled_with_callback(PWM_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Function to configure the ADC
void configure_adc() {
    adc_init();  // Initialize ADC hardware
    adc_gpio_init(ADC_PIN);  // Initialize GPIO 26 for ADC input
    adc_select_input(0);  // Select ADC input 0 (corresponding to GP26)
}

int main() {
    stdio_init_all();

    // 1. Configure GPIO to measure pulse width and frequency from digital signals
    configure_gpio_for_pwm();

    // 2. Configure the ADC to read analog signals
    configure_adc();

    while (true) {
        // Read the analog signal (converted to a 12-bit digital value)
        uint16_t raw_adc = adc_read();
        
        // Convert the raw ADC value to a voltage (assuming 3.3V reference voltage)
        float voltage = raw_adc * 3.3f / (1 << 12);  // Convert ADC value to voltage (0-3.3V)

        // Print the ADC value and the corresponding voltage
        printf("ADC Value: %d, Voltage: %.2fV\n", raw_adc, voltage);

        sleep_ms(500);  // Sample every 500ms
    }

    return 0;
}