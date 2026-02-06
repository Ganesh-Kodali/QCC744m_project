#include <stdint.h>

#define GPIO_CFG0 (*(volatile uint32_t *)0x200008C4)

void delay(volatile uint32_t d)
{
    while(d--);
}

int main(void)
{

    uint32_t reg;

    reg = GPIO_CFG0;

    reg &= ~(0x1F << 8);
    reg |=  (0x0B << 8);

    reg &= ~(3 << 30);

    reg |= (1 << 6);

    reg &= ~(3 << 2);

    reg |=(3 << 2);

        reg &=~((3<<0)|(3<<4));
    GPIO_CFG0 = reg;

    while(1)
    {
        GPIO_CFG0 |= (1 << 24);
        delay(30000000);

        GPIO_CFG0 &= ~(1 << 24);
        delay(30000000);
    }
}

