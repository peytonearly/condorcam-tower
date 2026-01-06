// main.c - Pico PIO quadrature encoder interface (USB CDC version)
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "quadrature.pio.h"

// === Pin configuration ===
#define PIN_A    14
#define PIN_B    15
#define SM       0
#define PIO_INST pio0
#define LED_PIN  25

// === Globals ===
static volatile int32_t position = 0;
static volatile uint32_t prev_state = 0xFF;

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
    sleep_ms(2000);  // Give USB CDC time to enumerate

    // LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Pull-ups
    gpio_pull_up(PIN_A);
    gpio_pull_up(PIN_B);

    // PIO setup
    uint offset = pio_add_program(PIO_INST, &quadrature_program);
    quadrature_program_init(PIO_INST, SM, offset, PIN_A);

    // IRQ config
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

        // Blink until host talks
        if (!connected && absolute_time_diff_us(last_blink, get_absolute_time()) > 250000) {
            led_state = !led_state;
            gpio_put(LED_PIN, led_state);
            last_blink = get_absolute_time();
        }

        // Command handling
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT && c != EOF) {
            connected = true;
            gpio_put(LED_PIN, 1);

            char buf[16] = {0};
            int idx = 0;
            buf[idx++] = (char)c;

            while (idx < (int)sizeof(buf) - 1) {
                int next = getchar_timeout_us(500);
                if (next == '\n' || next == '\r' || next == PICO_ERROR_TIMEOUT) break;
                buf[idx++] = (char)next;
            }

            int cmd = atoi(buf);
            switch (cmd) {
                case 1: {  // Read position
                    uint32_t save = save_and_disable_interrupts();
                    int32_t pos = position;
                    restore_interrupts(save);
                    printf("%ld\r\n", (long)pos);
                    fflush(stdout);
                    break;
                }

                case 2: {  // Reset position
                    uint32_t save = save_and_disable_interrupts();
                    position = 0;
                    prev_state = 0xFF;
                    while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM)) (void)pio_sm_get(PIO_INST, SM);
                    restore_interrupts(save);
                    printf("OK\r\n");
                    fflush(stdout);
                    break;
                }

                default:
                    printf("ERR\r\n");
                    fflush(stdout);
                    break;
            }
        }
    }
}