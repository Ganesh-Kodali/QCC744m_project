void board_init(void);

void board_uartx_gpio_init();
void board_i2c0_gpio_init();
void board_spi0_gpio_init();
void board_pwm_gpio_init();
void board_adc_gpio_init();
void board_dac_gpio_init();
void board_emac_gpio_init();
void board_sdh_gpio_init();
void board_ir_gpio_init();
void board_dvp_gpio_init();
void board_i2s_gpio_init();
void board_timer_gpio_init();
void board_acomp_init();


struct qcc74x_device_s {
    const char *name;
    uint32_t reg_base;
    uint8_t irq_num;
    uint8_t idx;
    uint8_t sub_idx;
    uint8_t dev_type;
    void *user_data;
};


 * @brief Get device handle by name.
 *
 * @param [in] name device name
 * @return device handle
 */
struct qcc74x_device_s *qcc74x_device_get_by_name(const char *name);


/**
 * @brief I2C message structure
 *
 * @param addr          Slave address (7- or 10-bit)
 * @param flags         See I2C_M_* definitions
 * @param buffer        Buffer to be transferred
 * @param length        Length of the buffer in bytes, should be less than 256.
 */
struct qcc74x_i2c_msg_s {
    uint16_t addr;
    uint16_t flags;
    uint8_t *buffer;
    uint16_t length;
};

 * @brief Initialize i2c.
 *
 * @param [in] dev device handle
 * @param [in] frequency i2c frequency, range from 305Hz to 400KHz
 */
void qcc74x_i2c_init(struct qcc74x_device_s *dev, uint32_t frequency);

/**
 * @brief Deinitialize i2c.
 *
 * @param [in] dev device handle
 */
void qcc74x_i2c_deinit(struct qcc74x_device_s *dev);

/**
 * @brief Start transferring i2c message.
 *
 * @param [in] dev device handle
 * @param [in] msgs pointer to i2c message
 * @param [in] count message count
 * @return A negated errno value on failure.
 */
int qcc74x_i2c_transfer(struct qcc74x_device_s *dev, struct qcc74x_i2c_msg_s *msgs, int count);


GPIO
 * @brief Initialize gpio pin.
 *
 * @param [in] dev device handle
 * @param [in] pin gpio pin, use @ref GPIO_PIN
 * @param [in] cfgset gpio config mask
 */
void qcc74x_gpio_init(struct qcc74x_device_s *dev, uint8_t pin, uint32_t cfgset);



did spi board to board

master board init is going into infinite loop 

and done debugging with printf statement 

found spi init is not working
