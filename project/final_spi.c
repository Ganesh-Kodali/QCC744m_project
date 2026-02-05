#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "qcc74x_mtimer.h"
#include "qcc74x_spi.h"
#include "qcc74x_gpio.h"
#include "board.h"

// SLAVE PINS (12,13,14,15)
 #define SPI_CS_PIN      GPIO_PIN_0
#define SPI_SCK_PIN     GPIO_PIN_1
#define SPI_MISO_PIN    GPIO_PIN_2
#define SPI_MOSI_PIN    GPIO_PIN_3

static struct qcc74x_device_s *spi0;
static struct qcc74x_device_s *gpio;

void spi_init(void) {
    gpio = qcc74x_device_get_by_name("gpio");
    qcc74x_gpio_init(gpio, SPI_SCK_PIN,  4 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_MISO_PIN, 4 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_MOSI_PIN, 4 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_CS_PIN,   4 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    struct qcc74x_spi_config_s cfg = {
        .freq = 500000, .role = SPI_ROLE_SLAVE, .mode = SPI_MODE3,
        .data_width = 8, .bit_order = SPI_BIT_MSB, .byte_order = SPI_BYTE_LSB
    };
    spi0 = qcc74x_device_get_by_name("spi0");
    qcc74x_spi_init(spi0, &cfg);
}

int main(void) {
    board_init();
    spi_init();
    printf("=== SLAVE LISTENING ===\r\n");

    uint8_t rx;
    char buffer[64];
    int idx = 0;
    int capturing = 0;

    while (1) {
        // Wait for 1 byte
        if (qcc74x_spi_poll_exchange(spi0, NULL, &rx, 1) == 0) {

            if (rx == '<') {
                // START OF PACKET
                capturing = 1;
                idx = 0;
                memset(buffer, 0, 64);
            }
            else if (rx == '>') {
                // END OF PACKET
                capturing = 0;
                printf("[RX] %s\r\n", buffer);
            }
            else if (capturing && idx < 63) {
                // DATA
                buffer[idx++] = (char)rx;
            }
        }
    }
}
