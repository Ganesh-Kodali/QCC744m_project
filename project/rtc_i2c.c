#include "qcc74x_mtimer.h"
#include "qcc74x_i2c.h"
#include "qcc74x_uart.h"
#include "board.h"
#include <stdio.h>
#include <string.h>
#define DS1307_ADDR 0x68
#define DS1307_REG  0x00

struct qcc74x_device_s *uart0;
static struct qcc74x_device_s *i2c0;

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
                uint8_t sec,
                uint8_t min,
                uint8_t hour,
                uint8_t day,
                uint8_t date,
                uint8_t month,
                uint8_t year)
{
        uint8_t data[7];

        data[0] = dec_to_bcd(sec)  & 0x7F;   // seconds (CH = 0)
        data[1] = dec_to_bcd(min);
        data[2] = dec_to_bcd(hour);          // 24-hour mode
        data[3] = dec_to_bcd(day);           // day (1–7)
        data[4] = dec_to_bcd(date);          // date
        data[5] = dec_to_bcd(month);         // month
        data[6] = dec_to_bcd(year);          // year (00–99)

        ds1307_write(DS1307_REG, data, 7);
}

static void ds1307_get_time(void)
{
        uint8_t b[7];
        char msg[100]={0};

        ds1307_read(DS1307_REG, b, 7);
                uart_send_string("hello3\r\n");
        uart_send_string("CITY : HYDERABAD\r\n");
        sprintf(msg,"TIME : %02d:%02d:%02d\r\n",
                        bcd_to_dec(b[2] & 0x3F),
                        bcd_to_dec(b[1]),
                        bcd_to_dec(b[0] & 0x7F));
        uart_send_string(msg);
        memset(msg,0,100);
        sprintf(msg,"DATE : %02d/%02d/20%02d\r\n",
                        bcd_to_dec(b[4]),
                        bcd_to_dec(b[5]),
                        bcd_to_dec(b[6]));

        uart_send_string(msg);
        memset(msg,0,100);
        uart_send_string("-----------------------------\r\n");
}

int main(void)
{
        board_init();

        uart0_init();

        board_i2c0_gpio_init();
        i2c0 = qcc74x_device_get_by_name("i2c0");
        qcc74x_i2c_init(i2c0, 100000);

                uart_send_string("hello\r\n");
        printf("RTC DISPLAY START\r\n");
	 ds1307_set_time(
                        30,   // sec
                        45,   // min
                        10,   // hour
                        1,    // day (Monday)
                        3,    // date
                        2,    // month
                        26    // year (2026)
                       );

        while (1)
        {
                ds1307_get_time();
                uart_send_string("hello2\r\n");
                qcc74x_mtimer_delay_ms(1000);
        }
}
