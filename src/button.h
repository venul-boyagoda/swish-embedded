#ifndef BUTTON_H
#define BUTTON_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// Initialize button
void setup_button(void);

// Button pressed callback
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// Work queue handler for button press
void i2c_work_handler(struct k_work *work);

#endif // BUTTON_H