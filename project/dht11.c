#include "board.h"
#include "qcc74x_mtimer.h"
#include "qcc74x_i2c.h"
#include "qcc74x_uart.h"
#include "qcc74x_gpio.h"
#include <stdio.h>
#include <string.h>

/* ===================== DS1307 ===================== */
#define DS1307_ADDR 0x68
#define DS1307_REG  0x00

/* ===================== DHT11 ====================== */
#define DHT11_PIN GPIO_PIN_3

struct qcc74x_device_s *uart0;
static struct qcc74x_device_s *i2c0;
static struct qcc74x_device_s *gpio;

/* ===================== UTIL ======================= */
static uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

static void uart_send_string(const char *str)
{
    while (*str) {
        qcc74x_uart_putchar(uart0, *str++);
    }
}

/* ===================== UART ======================= */
static void uart0_init(void)
{
    board_uartx_gpio_init();
    uart0 = qcc74x_device_get_by_name(DEFAULT_TEST_UART);

    struct qcc74x_uart_config_s cfg = {
        .baudrate = 115200,
        .data_bits = UART_DATA_BITS_8,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_NONE,
        .flow_ctrl = UART_FLOWCTRL_NONE,
        .tx_fifo_threshold = 0,
        .rx_fifo_threshold = 0,
        .bit_order = UART_LSB_FIRST,
    };

    qcc74x_uart_init(uart0, &cfg);
}

/* ===================== I2C ======================== */
static void ds1307_write(uint8_t reg, uint8_t *data, uint8_t len)
{
    struct qcc74x_i2c_msg_s msgs[2];

    msgs[0].addr   = DS1307_ADDR;
    msgs[0].flags  = I2C_M_NOSTOP;
    msgs[0].buffer = &reg;
    msgs[0].length = 1;

    msgs[1].addr   = DS1307_ADDR;
    msgs[1].flags  = 0;
    msgs[1].buffer = data;
    msgs[1].length = len;

    qcc74x_i2c_transfer(i2c0, msgs, 2);
}

static void ds1307_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    struct qcc74x_i2c_msg_s msgs[2];

    msgs[0].addr   = DS1307_ADDR;
    msgs[0].flags  = I2C_M_NOSTOP;
    msgs[0].buffer = &reg;
    msgs[0].length = 1;

    msgs[1].addr   = DS1307_ADDR;
    msgs[1].flags  = I2C_M_READ;
    msgs[1].buffer = data;
    msgs[1].length = len;

    qcc74x_i2c_transfer(i2c0, msgs, 2);
}

static void ds1307_set_time(
    uint8_t sec, uint8_t min, uint8_t hour,
    uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
    uint8_t data[7];

    data[0] = dec_to_bcd(sec) & 0x7F;
    data[1] = dec_to_bcd(min);
    data[2] = dec_to_bcd(hour);
    data[3] = dec_to_bcd(day);
    data[4] = dec_to_bcd(date);
    data[5] = dec_to_bcd(month);
    data[6] = dec_to_bcd(year);

    ds1307_write(DS1307_REG, data, 7);
}

/* ===================== DHT11 ====================== */
static void delay_us(uint32_t us)
{
    qcc74x_mtimer_delay_us(us);
}

int dht11_read(uint8_t *temp, uint8_t *hum)
{
    uint8_t data[5] = {0};
    uint32_t timeout;

    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_OUTPUT);
    qcc74x_gpio_reset(gpio, DHT11_PIN);
    qcc74x_mtimer_delay_ms(20);

    qcc74x_gpio_set(gpio, DHT11_PIN);
    delay_us(40);

    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_INPUT);

    timeout = 300;
    while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;

    timeout = 300;
    while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;

    timeout = 300;
    while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;

    for (int i = 0; i < 40; i++) {
        timeout = 300;
        while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);

        delay_us(50);

        if (qcc74x_gpio_read(gpio, DHT11_PIN))
            data[i / 8] |= (1 << (7 - (i % 8)));

        timeout = 300;
        while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    }

    if ((data[0] + data[1] + data[2] + data[3]) != data[4])
        return -1;

    *hum  = data[0];
    *temp = data[2];
    return 0;
}

/* ===================== MAIN ======================= */
int main(void)
{
    uint8_t temp, hum;
    uint8_t rtc[7];
    char msg[128];

    board_init();

    uart0_init();

    board_i2c0_gpio_init();
    i2c0 = qcc74x_device_get_by_name("i2c0");
    qcc74x_i2c_init(i2c0, 100000);

    gpio = qcc74x_device_get_by_name("gpio");

    uart_send_string("RTC + DHT11 STARTED\r\n");

    ds1307_set_time(30, 45, 10, 1, 3, 2, 26);

    while (1)
    {
        ds1307_read(DS1307_REG, rtc, 7);

        sprintf(msg,
            "TIME %02d:%02d:%02d  DATE %02d/%02d/20%02d\r\n",
            bcd_to_dec(rtc[2] & 0x3F),
            bcd_to_dec(rtc[1]),
            bcd_to_dec(rtc[0] & 0x7F),
            bcd_to_dec(rtc[4]),
            bcd_to_dec(rtc[5]),
            bcd_to_dec(rtc[6]));
        uart_send_string(msg);

        if (dht11_read(&temp, &hum) == 0) {
            sprintf(msg, "TEMP=%d C  HUM=%d %%\r\n", temp, hum);
        } else {
            sprintf(msg, "DHT11 ERROR\r\n");
        }
        uart_send_string(msg);

        uart_send_string("--------------------------------\r\n");
        qcc74x_mtimer_delay_ms(1000);
    }
}
