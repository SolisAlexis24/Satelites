#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "pico/mutex.h"
#include "LSM9DS1.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define resolucion "%.4f"

LSM9DS1 lsm(I2C_PORT);


int main()
{
    stdio_init_all();
    sleep_ms(2000);

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    printf("Ejemplo sencillo de funcionamiento de LSM9DS1===============");
    bool succes;
    succes = lsm.init_accel( LSM9DS1::SCALE_GYRO_2000DPS,  LSM9DS1::SCALE_ACCEL_16G,  LSM9DS1::ODR_119HZ, LSM9DS1::ODR_119HZ) && 
              lsm.init_magnetometer(LSM9DS1::MAG_SCALE_16GAUSS, LSM9DS1::MAG_ODR_80HZ);
    if(!succes){
        return 0;
    }

    volatile absolute_time_t now;
    volatile uint64_t elapsed_ms;
    absolute_time_t start_time = get_absolute_time();


    while (true) {
        now = get_absolute_time();
        elapsed_ms = absolute_time_diff_us(start_time, now) / 1000;
        lsm.read_sensor(elapsed_ms);
        printf("Acelerometro-> X: "resolucion", Y: "resolucion", Z: "resolucion"\n", 
            lsm.last_measurement.accel[0],
            lsm.last_measurement.accel[1],
            lsm.last_measurement.accel[2]);
        printf("Giroscopio-> X: "resolucion", Y: "resolucion", Z: "resolucion"\n", 
            lsm.last_measurement.gyro[0],
            lsm.last_measurement.gyro[1],
            lsm.last_measurement.gyro[2]);
        printf("Magnetometro-> X: "resolucion", Y: "resolucion", Z: "resolucion"\n", 
            lsm.last_measurement.mag[0],
            lsm.last_measurement.mag[1],
            lsm.last_measurement.mag[2]);
        printf("@ %llu ms\n", lsm.last_measurement.time_ms);
        sleep_ms(10);
    }
    
}
