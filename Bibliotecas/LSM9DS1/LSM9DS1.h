#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "math.h"
#include "pico/mutex.h"

/**
 * @class LSM9DS1
 * @brief Clase para manejar el sensor LSM9DS1 (Acelerómetro, Giroscopio y Magnetómetro)
 * 
 * Esta clase proporciona una interfaz orientada a objetos para configurar y leer datos
 * del sensor LSM9DS1 a través del bus I2C.
 */
class LSM9DS1 {
public:
    /**
     * @enum GYRO_SCALES 
     * @brief Escalas configurables para las mediciones del giroscipio [dps].
     */
    enum GYRO_SCALES {
        SCALE_GYRO_245DPS = 0,
        SCALE_GYRO_500DPS = 1,
        SCALE_GYRO_2000DPS = 3
    };

    /**
     * @enum ACCEL_SCALES
     * @brief Escalas configurables para las mediciones del acelerometro [g].
     */
    enum ACCEL_SCALES {
        SCALE_ACCEL_2G = 0,
        SCALE_ACCEL_16G = 1,
        SCALE_ACCEL_4G = 2,
        SCALE_ACCEL_8G = 3
    };

    /**
     * @enum MAG_SCALES
     * @brief Escalas configurables para las mediciones del magnetometro [Gauss].
     */
    enum MAG_SCALES {
        MAG_SCALE_4GAUSS = 0,
        MAG_SCALE_8GAUSS = 1,
        MAG_SCALE_12GAUSS = 2,
        MAG_SCALE_16GAUSS = 3
    };

    /**
     * @enum AG_ORD
     * @brief Frecuencia con la que el sensor de aceleracion y 
     * giroscopio entregan mediciones (Output Data Rate) [Hz].
     */
    enum AG_ODR {
        ODR_POWER_DOWN = 0,
        ODR_14_9HZ = 1,
        ODR_59_5HZ = 2,
        ODR_119HZ = 3,
        ODR_238HZ = 4,
        ODR_476HZ = 5,
        ODR_952HZ = 6
    };

    /**
     * @enum MAG_ORD
     * @brief Frecuencia con la que el magnetometro entregan mediciones (Output Data Rate) [Hz].
     */
    enum MAG_ODR {
        MAG_ODR_0_625HZ = 0,
        MAG_ODR_1_25HZ = 1,
        MAG_ODR_2_5HZ = 2,
        MAG_ODR_5HZ = 3,
        MAG_ODR_10HZ = 4,
        MAG_ODR_20HZ = 5,
        MAG_ODR_40HZ = 6,
        MAG_ODR_80HZ = 7
    };

    /**
     * @struct LSM9DS1Data
     * @brief Estructura de datos que contiene la informacion que el sensor puede entregar,
     * ademas de una variable de tiempo que guarda el momento en el que la muestra es guardada.
     */
    typedef struct {
        float accel[3] = {};
        float gyro[3] = {};
        float mag[3] = {};
        uint64_t time_ms = 0;
    } LSM9DS1Data;

    /**
     * @brief Constructor de la clase LSM9DS1
     * 
     * @param i2c_port Puerto I2C a utilizar (i2c0 o i2c1)
     * @param i2c_mutex Mutex para proteger el acceso al bus I2C
     */
    LSM9DS1(i2c_inst_t* i2c_port, mutex_t* i2c_mutex);
    
    /**
     * @brief Inicializa el acelerómetro y giroscopio
     * 
     * Configura los registros de control del acelerómetro/giroscopio con los parámetros dados
     * y verifica si el sensor esta conectado
     * 
     * @param gyro_scale Escala del giroscopio
     * @param accel_scale Escala del acelerómetro
     * @param gyro_odr Tasa de entrega de datos del giroscopio
     * @param accel_odr Tasa de entrega de datos del acelerómetro
     * @return true si se pudo encontrar y configurar el sensor, false en otro caso
     */
    bool init_accel(GYRO_SCALES gyro_scale,
                   ACCEL_SCALES accel_scale,
                   AG_ODR gyro_odr,
                   AG_ODR accel_odr);
    
    /**
     * @brief Inicializa el magnetómetro
     * 
     * Configura los registros de control del magnetómetro con los parámetros dados
     * y verifica sei el sensor esta conectado
     * 
     * @param scale Escala del magnetómetro
     * @param odr Tasa de entrega de datos muestreo
     * @return true si se pudo encontrar y configurar el sensor, false en otro caso
     */
    bool init_magnetometer(MAG_SCALES scale , 
                         MAG_ODR odr);

    /**
     * @brief Configura un offset al magnetometro sobre cada eje para corregir mediciones dispares en ejes
     * @param offset_i offset en Gauss que se restara a los valores del magnetometro en cada eje
     */
    void calibrate_magnetometer(float offset_x, float offset_y, float offset_z);

    /**
     * @brief Configura un offset al giroscopio sobre cada eje para corregir mediciones dispares en ejes
     * @param offset_i offset en dps que se restara a los valores del giroscopio en cada eje
     */
    void set_offsets_gyro(float offset_x, float offset_y, float offset_z);


    /**
     * @brief Lee los datos del sensor y los guarda en la estructura de datos interna
     * @param time_ms Tiempo en ms en el cual se realizo la medicion
     */
    void read_sensor(uint64_t time_ms);

    LSM9DS1Data last_measurement{};      // Atributo de la clase donde se guarda la ultima medicion realizada

private:
    // Direcciones I2C
    static constexpr uint8_t AG_ADDR = 0x6B;  // Acelerómetro/Giroscopio
    static constexpr uint8_t M_ADDR = 0x1E;   // Magnetómetro

    // Valores de registros WhoAmI
    static constexpr uint8_t VAL_WHO_AM_I_AG = 0x68;
    static constexpr uint8_t VAL_WHO_AM_I_M = 0x3D;

    static constexpr uint8_t LECTURA_CONSECUTIVA = 0x80;


    // Registros
    static constexpr uint8_t WHO_AM_I_AG = 0x0F;
    static constexpr uint8_t CTRL_REG1_G = 0x10;
    static constexpr uint8_t CTRL_REG4 = 0x1E;
    static constexpr uint8_t CTRL_REG5_XL = 0x1F;
    static constexpr uint8_t CTRL_REG6_XL = 0x20;
    static constexpr uint8_t CTRL_REG7_XL = 0x21;
    static constexpr uint8_t CTRL_REG8 = 0x22;
    static constexpr uint8_t CTRL_REG9 = 0x23;
    static constexpr uint8_t OUT_X_L_XL = 0x28;
    static constexpr uint8_t OUT_X_L_G = 0x18;
    static constexpr uint8_t WHO_AM_I_M = 0x0F;
    static constexpr uint8_t CTRL_REG1_M = 0x20;
    static constexpr uint8_t CTRL_REG2_M = 0x21;
    static constexpr uint8_t CTRL_REG3_M = 0x22;
    static constexpr uint8_t CTRL_REG4_M = 0x23;
    static constexpr uint8_t CTRL_REG5_M = 0x24;
    static constexpr uint8_t OUT_X_L_M = 0x28;
    static constexpr uint8_t FIFO_CTRL_REG = 0x2E;
    static constexpr uint8_t OFFSET_X_REG_L_M = 0x05;
    static constexpr uint8_t OFFSET_X_REG_H_M = 0x06;
    static constexpr uint8_t OFFSET_Y_REG_L_M = 0x07;
    static constexpr uint8_t OFFSET_Y_REG_H_M = 0x08;
    static constexpr uint8_t OFFSET_Z_REG_L_M = 0x09;
    static constexpr uint8_t OFFSET_Z_REG_H_M = 0x0A;

    i2c_inst_t* i2c_port_;
    mutex_t* i2c_mutex_;
    
    float gyro_div_scale_factor_;
    float accel_div_scale_factor_;
    float mag_div_scale_factor_;
    float gyro_offset_x_;
    float gyro_offset_y_;
    float gyro_offset_z_;

    /**
     * @brief Lee un registro del sensor
     * 
     * @param addr Dirección I2C del dispositivo
     * @param reg Registro a leer
     * @return uint8_t Valor leído del registro
     */
    uint8_t read_register(uint8_t addr, uint8_t reg);
    
    /**
     * @brief Escribe en un registro del sensor
     * 
     * @param addr Dirección I2C del dispositivo
     * @param reg Registro a escribir
     * @param val Valor a escribir
     */
    void write_register(uint8_t addr, uint8_t reg, uint8_t val);
    
    /**
     * @brief Lee múltiples bytes consecutivos del sensor
     * @param addr Dirección I2C del dispositivo
     * @param reg Registro inicial a leer
     * @param buf Buffer donde almacenar los datos
     * @param len Número de bytes a leer
     */
    void read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
    
    /**
     * @brief Calcula los factores de escala para el acelerómetro y giroscopio
     * @param gyro_scale Escala del giroscopio
     * @param accel_scale Escala del acelerómetro
     */
    void calculate_accel_scale_factors(GYRO_SCALES gyro_scale, ACCEL_SCALES accel_scale);
    
    /**
     * @brief Calcula el factor de escala para el magnetómetro
     * 
     * @param scale Escala del magnetómetro
     */
    void calculate_mag_scale_factor(MAG_SCALES scale);

    /**
     * @brief Lee los datos del acelerómetro
     * 
     * @param accel Arreglo donde se almacenarán los datos [X,Y,Z] en g
     */
    void read_accelerometer(float (&accel)[3]);
    
    /**
     * @brief Lee los datos del giroscopio
     * 
     * @param gyro Arreglo donde se almacenarán los datos [X,Y,Z] en dps
     */
    void read_gyroscope(float (&gyro)[3]);
    
    /**
     * @brief Lee los datos del magnetómetro
     * @param mag Arreglo donde se almacenarán los datos [X,Y,Z] en Gauss
     */
    void read_magnetometer(float (&mag)[3]);

};