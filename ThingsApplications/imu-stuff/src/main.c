#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "bno055.h"

#define SLEEP_TIME_MS 1000
#define I2C2_NODE DT_NODELABEL(bno055)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C2_NODE);



int main(void)
{

        if (!device_is_ready(dev_i2c.bus)) {
		printf("Device %s is not ready\n", dev_i2c.bus->name);
		return 0;
	}

        printf("Device %p name is %s\n", &dev_i2c, dev_i2c.bus->name);

        uint8_t accel_data[6];
        int16_t accel_x, accel_y, accel_z;
        float accel_x_f, accel_y_f, accel_z_f;

        

        int ret;
        uint8_t config[2] = {BNO055_OPR_MODE_ADDR,BNO055_OPERATION_MODE_NDOF};
        ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
        if(ret != 0){
	        printf("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,config[0]);
        }

        while (true) {
                if (i2c_burst_read_dt(&dev_i2c, BNO055_ACCEL_DATA_X_LSB_ADDR, accel_data, sizeof(accel_data)) < 0) {
                        printf("Failed to read acceleration data\n");
                } else {

                        accel_x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
                        accel_y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
                        accel_z = (int16_t)((accel_data[5] << 8) | accel_data[4]);

                        accel_x_f = accel_x * 0.00980665;
                        accel_y_f = accel_y * 0.00980665;
                        accel_z_f = accel_z * 0.00980665;

                        printf("Accel (m/s^2): X=%.2f, Y=%.2f, Z=%.2f\n", accel_x_f, accel_y_f, accel_z_f);
                }

                k_msleep(500); 
        }
        
        return 0;
}
