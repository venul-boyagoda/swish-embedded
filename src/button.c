#include "button.h"
#include "bno055_driver.h"
#include "onboard_imu.h"
#include <stdio.h>

#define BUTTON_NODE DT_NODELABEL(button0)

static const struct device *button_dev = DEVICE_DT_GET(DT_GPIO_CTLR(BUTTON_NODE, gpios));
static const gpio_pin_t button_pin = DT_GPIO_PIN(BUTTON_NODE, gpios);
static const gpio_flags_t button_flags = DT_GPIO_FLAGS(BUTTON_NODE, gpios);

/* Callback function for button press */
static struct gpio_callback button_cb_data;

/* Work queue definition */
K_WORK_DEFINE(i2c_work, i2c_work_handler);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_submit(&i2c_work);
}

void setup_button(void) {
    /* Configure button as input with pull-up and interrupt */
    int ret = gpio_pin_configure(button_dev, button_pin, GPIO_INPUT | button_flags);
    if (ret < 0) {
        printk("Error configuring button GPIO\n");
        return;
    }

    /* Set up interrupt */    
    ret = gpio_pin_interrupt_configure(button_dev, button_pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("Error configuring button interrupt\n");
        return;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button_pin));
    gpio_add_callback(button_dev, &button_cb_data);
}

/* Function to handle I2C operation (runs in work queue) */
void i2c_work_handler(struct k_work *work) {
    printk("Resetting Quaternions Reference\n");
    
    // Save current quaternions as reference
    q0_ref_bno = q0;
    q1_ref_bno = q1;
    q2_ref_bno = q2;
    q3_ref_bno = q3;

    q0_ref_onboard = qq0;
    q1_ref_onboard = qq1;
    q2_ref_onboard = qq2;
    q3_ref_onboard = qq3;
}