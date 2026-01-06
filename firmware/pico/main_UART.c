// main.c - Pico PIO quadrature encoder interface (UART version)
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "quadrature.pio.h"

// === Pin configuration ===
#define PIN_A       14
#define PIN_B       15
#define SM          0
#define PIO_INST    pio0
#define UART_ID     uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BAUD_RATE   115200
#define LED_PIN     25

// === Globals ===
static volatile int32_t position = 0;
static volatile uint8_t prev_state = 0xFF;

// Gray code lookup table
static const int8_t step_lut[16] = {
    /*0000*/  0, /*0001*/ +1, /*0010*/ -1, /*0011*/  0,
    /*0100*/ -1, /*0101*/  0, /*0110*/  0, /*0111*/ +1,
    /*1000*/ +1, /*1001*/  0, /*1010*/  0, /*1011*/ -1,
    /*1100*/  0, /*1101*/ -1, /*1110*/ +1, /*1111*/  0
};

// === IRQ handler ===
void __isr pio0_irq0_handler(void) {
    while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM)) {
        uint8_t curr = (uint8_t)(pio_sm_get(PIO_INST, SM) & 0x3);
        if (prev_state != 0xFF) {
            int idx = ((prev_state & 3) << 2) | curr;
            position += step_lut[idx];
        }
        prev_state = curr;
    }
}

// === Main ===
int main(void) {
    stdio_init_all();

    // UART setup
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_ID, true);

    // LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Pull-ups on encoder lines
    gpio_pull_up(PIN_A);
    gpio_pull_up(PIN_B);

    // Load and start PIO
    uint offset = pio_add_program(PIO_INST, &quadrature_program);
    quadrature_program_init(PIO_INST, SM, offset, PIN_A);

    // Configure IRQ
    enum pio_interrupt_source src = pis_sm0_rx_fifo_not_empty;
    irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(PIO_INST, src, true);

    // Watchdog
    watchdog_enable(4000, 1);

    // Main loop
    bool led_state = false;
    absolute_time_t last_blink = get_absolute_time();
    bool connected = false;
    while (true) {
        watchdog_update();
        tight_loop_contents();

        // Blink until connected
        if (!connected && absolute_time_diff_us(last_blink, get_absolute_time()) > 250000) {
            led_state = !led_state;
            gpio_put(LED_PIN, led_state);
            last_blink = get_absolute_time();
        }

        // Command handling
        if (uart_is_readable(UART_ID)) {
            connected = true;
            gpio_put(LED_PIN, 1);

            char buf[16] = {0};
            int idx = 0;
            while (idx < (int)sizeof(buf) - 1) {
                if (!uart_is_readable(UART_ID)) break;
                char c = uart_getc(UART_ID);
                if (c == '\n' || c == '\r') break;
                buf[idx++] = c;
            }

            int cmd = atoi(buf);
            char msg[32];
            switch (cmd) {
                case 1: {  // Read position
                    uint32_t save = save_and_disable_interrupts();
                    int32_t pos = position;
                    restore_interrupts(save);
                    snprintf(msg, sizeof(msg), "%ld\r\n", (long)pos);
                    uart_puts(UART_ID, msg);
                    break;
                }

                case 2: {  // Reset position
                    uint32_t save = save_and_disable_interrupts();
                    position = 0;
                    prev_state = 0xFF;
                    while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM)) (void)pio_sm_get(PIO_INST, SM);
                    restore_interrupts(save);
                    uart_puts(UART_ID, "OK\r\n");
                    break;
                }

                default:
                    uart_puts(UART_ID, "ERR\r\n");
                    break;
            }
        }
    }
}