// main.c -- UART + PIO (IRQ) x4 quadrature decode, zsero-miss deltas

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "quadrature.pio.h"

// ===== Pins / UART =====
#define PIN_A       14
#define PIN_B       15
#define SM          0
#define PIO_INST    pio0
#define UART_ID     uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BAUD_RATE   115200
#define LED_PIN     25

// ===== Globals (ISR touched) =====
static volatile int32_t position = 0;
static volatile uint8_t prev_state = 0xFF;  // Invalid initially

// Gray-code setup LUT: (prev<<2)|curr -> delta (-1, 0, +1)
static const int8_t step_lut[16] = {
    /*0000*/  0, /*0001*/ +1, /*0010*/ -1, /*0011*/  0,
    /*0100*/ -1, /*0101*/  0, /*0110*/  0, /*0111*/ +1,
    /*1000*/ +1, /*1001*/  0, /*1010*/  0, /*1011*/ -1,
    /*1100*/  0, /*1101*/ -1, /*1110*/ +1, /*1111*/  0
};

// ===== PIO IRQ handler: drain FIFO *fast* and update position =====
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

// ===== Helpers =====
static inline void led_blink(bool *state) {
    *state = !*state;
    gpio_put(LED_PIN, *state);
}

int main(void) {
    stdio_init_all();

    // --- UART setup ---
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_ID, true);

    // --- LED ---
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // --- PIO: load + start program ---
    uint offset = pio_add_program(PIO_INST, &quadrature_program);
    quadrature_program_init(PIO_INST, SM, offset, PIN_A);

    // --- Ensure pins are stable before SM starts --- 
    gpio_pull_up(PIN_A);
    gpio_pull_up(PIN_B);

    uint32_t pins_state = (gpio_get(PIN_B) << 1) | gpio_get(PIN_A);
    char dbg[32];
    snprintf(dbg, sizeof(dbg), "A=%d B=%d\r\n", gpio_get(PIN_A), gpio_get(PIN_B));
    uart_puts(UART_ID, dbg);

    // --- Fully reset and drain the PIO SM/FIFO ---
    pio_sm_set_enabled(PIO_INST, SM, false);
    pio_sm_clear_fifos(PIO_INST, SM);
    pio_sm_restart(PIO_INST, SM);
    pio_sm_exec(PIO_INST, SM, pio_encode_jmp(offset));  // Jump to start of program
    pio_sm_set_enabled(PIO_INST, SM, true);
    sleep_us(50);  // Allow one cycle

    // --- Drain again (SM may have pushed one word immediately) ---
    while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM))
        (void)pio_sm_get(PIO_INST, SM);

    // --- Confirm FIFO level truly zero before enabling IRQ ---
    int fifo_level = pio_sm_get_rx_fifo_level(PIO_INST, SM);
    uart_puts(UART_ID, "FIFO level before IRQ enable: ");
    char msg2[16];
    snprintf(msg2, sizeof(msg2), "%d\r\n", fifo_level);
    uart_puts(UART_ID, msg2);

    // --- Select correct RX FIFO interrupt source for this SM ---
    enum pio_interrupt_source src = pis_sm0_rx_fifo_not_empty;
    if (SM == 1) src = pis_sm1_rx_fifo_not_empty;
    else if (SM == 2) src = pis_sm2_rx_fifo_not_empty;
    else if (SM == 3) src = pis_sm3_rx_fifo_not_empty;

    uart_puts(UART_ID, "Here1");

    // --- Attach handler before enabling source ---
    irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    uart_puts(UART_ID, "Here2");

    // Heartbeat before IRQ enable
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);

    uart_puts(UART_ID, "Here3");

    // --- Finally enable the chosen source ---
    pio_set_irq0_source_enabled(PIO_INST, src, true);

    uart_puts(UART_ID, "Here4");

    // Heartbeat after IRQ enable
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);

    uart_puts(UART_ID, "Here5");

    // --- Watchdog ---
    watchdog_enable(4000, 1);

    uart_puts(UART_ID, "Pico quadrature (UART + PIO IRQ x4) ready\r\n");

    // --- Command loop state ---
    int c, idx, next, cmd;
    char buf[16], msg[32];
    bool led_state = false;
    absolute_time_t last_blink = get_absolute_time();
    bool connected = false;

    while (true) {
        watchdog_update();
        tight_loop_contents();

        // Blink until first command seen
        if (!connected && absolute_time_diff_us(last_blink, get_absolute_time()) > 250000) {
            led_blink(&led_state);
            last_blink = get_absolute_time();
        }

        // Non-blocking UART line read
        if (uart_is_readable(UART_ID)) {
            connected = true;
            gpio_put(LED_PIN, 1);  // Solid once active
            idx = 0;

            c = uart_getc(UART_ID);
            buf[idx++] = (char)c;

            while (idx < (int)sizeof(buf) - 1) {
                if (!uart_is_readable(UART_ID)) break;
                next = uart_getc(UART_ID);
                if (next == '\n' || next == '\r') break;
                buf[idx++] =(char)next;
            }
            buf[idx] = '\0';

            cmd = atoi(buf);
            switch (cmd) {
                case 1: {  // Read position
                    uint32_t save = save_and_disable_interrupts();
                    int32_t pos = position;
                    restore_interrupts(save);

                    int n = snprintf(msg, sizeof(msg), "%ld\r\n", (long)pos);
                    if (n > 0) uart_puts(UART_ID, msg);
                } break;

                case 2: {  // Reset position
                    uint32_t save = save_and_disable_interrupts();
                    position = 0;
                    prev_state = 0xFF;  // Force resync on next sample
                    // Drain any stale FIFO samples
                    while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM)) (void)pio_sm_get(PIO_INST, SM);
                    restore_interrupts(save);
                    uart_puts(UART_ID, "OK\r\n");
                } break;

                default:
                    uart_puts(UART_ID, "ERR\r\n");
                    break;
            }
        }
    }
}