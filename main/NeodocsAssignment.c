#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <string.h>

#define LED_GPIO GPIO_NUM_2
#define BUTTON_GPIO GPIO_NUM_0
#define UART_PORT UART_NUM_1

// Timer handle
esp_timer_handle_t blink_timer;

static volatile bool button_event_detected = false;  // Flag for button press

// Interrupt Service Routine for Button Press
static void IRAM_ATTR button_isr_handler(void *arg) {
  static uint32_t last_isr_time = 0;
  //static volatile bool button_state = false;
  uint32_t current_isr_time = esp_timer_get_time();
  if (current_isr_time - last_isr_time > 20000) {  // 20 ms debounce time
    
    button_event_detected = true;  // Set flag when button is pressed
    last_isr_time = current_isr_time;
  }
}

void blink_timer_callback(void *args){
  static bool led_state = false;
  led_state = !led_state;  // Toggle LED state
  gpio_set_level(LED_GPIO, led_state);
}

void app_main(void) {
    // Configure LED GPIO as output
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_config);

    // Configure Button GPIO as input with pull-down and interrupt
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,// triggered on the positive edge
    };
    gpio_config(&button_config);

    // Install GPIO ISR service and add button ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);// receive buffer size of 1024 and a transmit buffer size of 0 (non-buffered mode)

    //configure preiodic timer
    const esp_timer_create_args_t timer_args = {
      .callback = &blink_timer_callback,
      .name = "blink_timer"
    };

    // Create the timer
    esp_timer_create(&timer_args, &blink_timer);
    // Start the timer with a 500-microsecond interval
    esp_timer_start_periodic(blink_timer, 500);  // Time in microseconds

    while (1) {
        // Check if the button was pressed
        if (button_event_detected) {
            const char *message = "Button 51 Pressed\n";
            //printf("%s", message);
            uart_write_bytes(UART_PORT, message, strlen(message));  // Send message via UART
            button_event_detected = false;  // Reset the flag after processing
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Avoid busy-looping
    }
}