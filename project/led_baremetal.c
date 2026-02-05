
#include <stdint.h>
#include "board.h"
#define GPIO_CFG0 (*(volatile uint32_t *)0x200008C4)

void delay(volatile uint32_t d)
{
    while(d--);
}

int main(void)
{
	 board_init();
    uint32_t reg;

    reg = GPIO_CFG0;

    reg &= ~(0x1F << 8);
    reg |=  (0x0B << 8);

    reg &= ~(3 << 30);

    reg |= (1 << 6);

    reg &= ~(3 << 2);

    GPIO_CFG0 = reg;

    while(1)
    {
        GPIO_CFG0 |= (1 << 24);
        delay(3000000);

        GPIO_CFG0 &= ~(1 << 24);
        delay(3000000);
    }
}

